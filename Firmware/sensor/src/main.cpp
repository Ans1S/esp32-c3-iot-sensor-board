#include <Arduino.h>

#include <esp_random.h>
#include <esp_timer.h>
#include <esp_sleep.h>
#include <driver/gpio.h>

#include "adc_reader.h"
#include "live_acquisition.h"
#include "recording_store.h"
#include "ota_client.h"
#include "environmental_sensor.h"
#include "espnow_transport.h"
#include "hardware_profile.h"
#include "lil_protocol.h"
#include "power_controller.h"
#include "sensor_config_store.h"
#include "sensor_log.h"
#include "sleep_controller.h"
#include "logical_clock.h"
#include "low_power_wait.h"
#include "power_policy.h"
#include "power_options.h"

namespace {
// Discovery is deliberately limited to sensors which have not been added to a
// station yet. Configured nodes only use short recovery wakes for a bounded
// initialization attempt, then resume their energy-saving measurement interval.
// Revision E retains bounded startup recovery across deep sleep.
constexpr uint32_t kRtcSignature = 0x52544345UL;  // "RTCE"
constexpr uint32_t kPairingMeasurementSeconds = 5UL * 60UL;
constexpr uint32_t kInitialSleepPhaseWindowMs = 1000UL;
constexpr uint8_t kRecoveryChannelsPerReport = 3;
constexpr uint8_t kSensorStartAttempts = 3;
constexpr uint32_t kSensorRecoverySleepSeconds = 5;

struct RtcState {
  uint32_t signature;
  uint32_t bootCount;
  uint32_t sequence;
  uint32_t acknowledgedResetRevision;
  int8_t lastStationRssi;
  uint64_t lastReportLogicalMs;
  uint64_t discoveryStartedMs;
  uint64_t pairingMeasurementMs;
  uint16_t initialSleepPhaseMs;
  uint8_t nextRecoveryChannel;
  bool initialSleepPhaseApplied;
  bool fullChannelRecoveryPending;
  sensor::EnvironmentalReading cachedEnvironment;
  sensor::BatteryReading cachedBattery;
  bool hasAttemptedReport;
  bool hasPairingSnapshot;
  bool batteryPaused;
  bool sensorRecoveryPending;
  uint8_t sensorStartAttemptsRemaining;
  lil::protocol::EnvironmentalSensorType lastSensorType;
};

RTC_DATA_ATTR RtcState rtcState{};

sensor::SensorConfigStore configStore;
sensor::SensorRuntimeConfig runtimeConfig;
sensor::PowerController powerController;
sensor::AdcReader adcReader;
sensor::EnvironmentalSensor environmentalSensor;
sensor::EspNowTransport espNowTransport;
bool rtcStateInitializedThisBoot = false;

bool macIsUsable(const uint8_t mac[6]) {
  uint8_t combined = 0;
  for (size_t i = 0; i < 6; ++i) combined |= mac[i];
  return combined != 0;
}

enum class OperatingMode : uint8_t {
  kDiscovery,
  kEnergySaving,
};

constexpr OperatingMode operatingModeForAssignment(
    bool provisioned, bool stationKnown, bool channelValid, bool macValid) {
  return provisioned && stationKnown && channelValid && macValid
             ? OperatingMode::kEnergySaving
             : OperatingMode::kDiscovery;
}

static_assert(operatingModeForAssignment(false, false, true, false) ==
                  OperatingMode::kDiscovery,
              "A fresh sensor must use discovery mode");
static_assert(operatingModeForAssignment(false, true, true, true) ==
                  OperatingMode::kDiscovery,
              "A deleted sensor must use discovery mode");
static_assert(operatingModeForAssignment(true, false, true, true) ==
                  OperatingMode::kDiscovery,
              "An incomplete assignment must use discovery mode");
static_assert(operatingModeForAssignment(true, true, false, true) ==
                  OperatingMode::kDiscovery,
              "An invalid channel must use discovery mode");
static_assert(operatingModeForAssignment(true, true, true, true) ==
                  OperatingMode::kEnergySaving,
              "A complete assignment must use energy-saving mode");

OperatingMode operatingMode(const sensor::SensorRuntimeConfig& config) {
  // Only a complete, persisted station assignment may enable the short radio
  // path. Any incomplete or cleared assignment safely falls back to discovery.
  return operatingModeForAssignment(
      config.provisioned, config.stationKnown,
      config.wifiChannel >= 1 && config.wifiChannel <= 13,
      macIsUsable(config.stationMac));
}

uint64_t logicalNowMs() {
  return sensor::logicalTimeMs();
}

void initializeRtcState() {
  if (rtcState.signature != kRtcSignature) {
    rtcState = RtcState{};
    rtcState.signature = kRtcSignature;
    rtcState.sequence = esp_random();
    rtcState.initialSleepPhaseMs = static_cast<uint16_t>(
        esp_random() % (kInitialSleepPhaseWindowMs + 1UL));
    rtcState.nextRecoveryChannel = 1;
    rtcState.sensorRecoveryPending = true;
    rtcState.sensorStartAttemptsRemaining = kSensorStartAttempts;
    rtcStateInitializedThisBoot = true;
  }
  ++rtcState.bootCount;
  ++rtcState.sequence;
}

uint8_t takeNextRecoveryChannel(uint8_t savedChannel) {
  for (uint8_t checked = 0; checked < 13; ++checked) {
    if (rtcState.nextRecoveryChannel < 1 ||
        rtcState.nextRecoveryChannel > 13) {
      rtcState.nextRecoveryChannel = 1;
    }
    const uint8_t candidate = rtcState.nextRecoveryChannel;
    rtcState.nextRecoveryChannel =
        rtcState.nextRecoveryChannel == 13
            ? 1
            : static_cast<uint8_t>(rtcState.nextRecoveryChannel + 1);
    if (candidate != savedChannel) {
      return candidate;
    }
  }
  return savedChannel;
}

bool applyStationConfig(const lil::protocol::ConfigResponsePayload& response) {
  if ((response.flags & lil::protocol::kFactoryReset) != 0) {
    if (!environmentalSensor.clearIaqState() || !configStore.factoryReset()) {
      SENSOR_LOG_PRINTLN("[CONFIG] Factory reset could not be persisted; retrying on the next report");
      return false;
    }
    rtcState.acknowledgedResetRevision = response.revision;
    rtcState.initialSleepPhaseMs = static_cast<uint16_t>(
        esp_random() % (kInitialSleepPhaseWindowMs + 1UL));
    rtcState.initialSleepPhaseApplied = false;
    rtcState.nextRecoveryChannel = 1;
    rtcState.fullChannelRecoveryPending = false;
    sensor::SleepController::deepSleep(1, powerController);
  }

  const bool iaqResetRequested =
      (response.flags & lil::protocol::kResetIaqCalibration) != 0 &&
      response.revision != runtimeConfig.revision;

  const bool sensorTypeValid =
      response.sensorType ==
          lil::protocol::EnvironmentalSensorType::kAutoDetect ||
      response.sensorType == lil::protocol::EnvironmentalSensorType::kBme280 ||
      response.sensorType == lil::protocol::EnvironmentalSensorType::kLsm6dsox ||
      response.sensorType == lil::protocol::EnvironmentalSensorType::kTmp117 ||
      response.sensorType == lil::protocol::EnvironmentalSensorType::kMax30102 ||
      response.sensorType == lil::protocol::EnvironmentalSensorType::kBme680 ||
      response.sensorType ==
          lil::protocol::EnvironmentalSensorType::kDisabled;
  if (response.sleepIntervalSeconds >= 1 &&
      response.sleepIntervalSeconds <= 86400 && response.wifiChannel >= 1 &&
      response.wifiChannel <= 13 && macIsUsable(response.stationMac) &&
      sensorTypeValid && isfinite(response.temperatureOffsetC) &&
      response.temperatureOffsetC >= -10.0F &&
      response.temperatureOffsetC <= 10.0F &&
      isfinite(response.batteryCalibrationFactor) &&
      response.batteryCalibrationFactor >= 0.7F &&
      response.batteryCalibrationFactor <= 1.3F) {
    const bool sensorTypeChanged =
        runtimeConfig.environmentalSensorType != response.sensorType;
    const bool provisioningChanged =
        runtimeConfig.provisioned != (response.provisioned != 0);
    const bool measurementChanged = sensorTypeChanged || provisioningChanged ||
        runtimeConfig.sleepSeconds != response.sleepIntervalSeconds ||
        runtimeConfig.temperatureOffsetC != response.temperatureOffsetC;
    const bool newProvisioned = response.provisioned != 0;
    auto candidate = runtimeConfig;
    candidate.revision = response.revision;
    candidate.sleepSeconds = response.sleepIntervalSeconds;
    candidate.wifiChannel = response.wifiChannel;
    candidate.txPowerQuarterDbm = constrain(response.txPowerQuarterDbm, 8, 84);
    candidate.environmentalSensorType = response.sensorType;
    candidate.temperatureOffsetC = response.temperatureOffsetC;
    candidate.batteryCalibrationFactor = response.batteryCalibrationFactor;
    candidate.provisioned = newProvisioned;
    candidate.stationKnown = newProvisioned;
    candidate.bme680QuickStartComplete = true;
    if (newProvisioned) memcpy(candidate.stationMac, response.stationMac, 6);
    else memset(candidate.stationMac, 0, sizeof(candidate.stationMac));
    // Reset commands must finish durably before their revision can be ACKed,
    // including after reboot. A later config-write failure may repeat the reset,
    // but can never advertise an unsuccessful reset as completed.
    if ((iaqResetRequested || sensorTypeChanged || provisioningChanged) &&
        !environmentalSensor.clearIaqState()) {
      SENSOR_LOG_PRINTLN("[CONFIG] IAQ state could not be cleared; keeping the previous revision");
      return false;
    }
    // A telemetry revision acknowledges durable settings. Keep the previous
    // revision when NVS fails so the station retries the command.
    if (!configStore.saveIfChanged(candidate)) {
      SENSOR_LOG_PRINTLN("[CONFIG] Settings could not be persisted; keeping the previous revision");
      return false;
    }
    if (!newProvisioned) {
      rtcState.fullChannelRecoveryPending = false;
      rtcState.nextRecoveryChannel = 1;
    } else if (provisioningChanged) {
      rtcState.fullChannelRecoveryPending = true;
      rtcState.nextRecoveryChannel = 1;
    }
    if (newProvisioned &&
        (response.flags & lil::protocol::kStationChannelStable) != 0) {
      rtcState.fullChannelRecoveryPending = false;
    }
    // Compatibility field in the persisted V6 layout. ULP-only operation has
    // no separate quick-start phase.
    runtimeConfig = candidate;
    if (measurementChanged) {
      rtcState.hasPairingSnapshot = false;
      rtcState.cachedEnvironment = {};
      rtcState.sensorRecoveryPending = true;
      rtcState.sensorStartAttemptsRemaining = kSensorStartAttempts;
      if (sensorTypeChanged) rtcState.lastSensorType = lil::protocol::EnvironmentalSensorType::kAutoDetect;
    }
    if (rtcState.acknowledgedResetRevision == response.revision) {
      rtcState.acknowledgedResetRevision = 0;
    }
    return measurementChanged;
  }
  return false;
}

void updatePacketBattery(lil::protocol::TelemetryPacket& packet,
                         const sensor::BatteryReading& battery) {
  packet.payload.batteryMillivolts = battery.millivolts;
  packet.payload.capabilities &= ~lil::protocol::kBattery;
  packet.payload.flags &= ~lil::protocol::kBatteryReadFailed;
  if (battery.valid) packet.payload.capabilities |= lil::protocol::kBattery;
  else packet.payload.flags |= lil::protocol::kBatteryReadFailed;
}

lil::protocol::TelemetryPacket makeTelemetryPacket(
    const sensor::EnvironmentalReading& environment,
    const sensor::BatteryReading& battery,
    lil::protocol::SensorOperatingMode operatingMode) {
  lil::protocol::TelemetryPacket packet{};
  packet.payload.appliedConfigRevision =
      rtcState.acknowledgedResetRevision != 0
          ? rtcState.acknowledgedResetRevision
          : runtimeConfig.revision;
  packet.payload.bootCount = rtcState.bootCount;
  packet.payload.capabilities = environment.capabilities |
                                (battery.valid ? lil::protocol::kBattery : 0);
  if (sensor::kHardware.pcbVersion == 4) {
    packet.payload.capabilities |= lil::protocol::kPcbV4PowerGates;
  }
  if (!environment.valid) {
    packet.payload.flags |= lil::protocol::kSensorReadFailed;
  }
  if (!battery.valid) {
    packet.payload.flags |= lil::protocol::kBatteryReadFailed;
  }
  packet.payload.motion = environment.motion;
  packet.payload.motionFeedback = environment.motionFeedback;
  packet.payload.pulse = environment.pulse;
  packet.payload.live = environment.live;
  packet.payload.temperatureC = environment.temperatureC;
  packet.payload.humidityPercent = environment.humidityPercent;
  packet.payload.pressureHpa = environment.pressureHpa;
  packet.payload.iaq = environment.iaq;
  packet.payload.gasResistanceOhms = environment.gasResistanceOhms;
  packet.payload.batteryMillivolts = battery.millivolts;
  packet.payload.pcbVersion = sensor::kHardware.pcbVersion;
  packet.payload.operatingMode = operatingMode;
  packet.payload.lastStationRssi = rtcState.lastStationRssi;
  packet.payload.sensorType = environment.sensorType;
  packet.payload.iaqAccuracy = environment.iaqAccuracy;
  packet.payload.iaqCalibrationPhase = environment.iaqCalibrationPhase;
  packet.payload.iaqCalibrationElapsedMinutes =
      environment.iaqCalibrationElapsedMinutes;
  packet.payload.iaqCalibrationRemainingMinutes =
      environment.iaqCalibrationRemainingMinutes;
  if (operatingMode == lil::protocol::SensorOperatingMode::kDiscovery) {
    packet.payload.flags |= lil::protocol::kDiscoveryBeacon;
  }
  if (environment.bme680RawFallback && environment.valid) {
    packet.payload.flags |= lil::protocol::kBme680RawFallback;
  }
  if ((environment.capabilities & lil::protocol::kIaq) != 0 &&
      environment.iaqAccuracy < 1) {
    packet.payload.flags |= lil::protocol::kIaqCalibrating;
  }
  if (runtimeConfig.environmentalSensorType !=
          lil::protocol::EnvironmentalSensorType::kAutoDetect &&
      runtimeConfig.environmentalSensorType !=
          lil::protocol::EnvironmentalSensorType::kDisabled &&
      environment.sensorType != lil::protocol::EnvironmentalSensorType::kAutoDetect &&
      environment.sensorType != runtimeConfig.environmentalSensorType) {
    packet.payload.flags |= lil::protocol::kSensorTypeMismatch;
  }
  lil::protocol::finalize(packet, lil::protocol::MessageType::kTelemetry,
                          rtcState.sequence);
  return packet;
}

bool batteryProtectionRequired(const sensor::BatteryReading& battery) {
  return lil::power::batteryProtectionRequired(rtcState.batteryPaused,
      battery.valid, battery.millivolts, SENSOR_LOW_BATTERY_PAUSE_MV,
      SENSOR_LOW_BATTERY_RESUME_MARGIN_MV);
}

[[noreturn]] void reportBatteryAndSleep(const sensor::BatteryReading& battery) {
  rtcState.batteryPaused = true;
  rtcState.hasPairingSnapshot = false;
  powerController.prepareForDeepSleep();
  // No sensor initialization, cached measurements, OTA download, channel scan
  // or configuration command may turn this into a normal measurement wake.
  sensor::EnvironmentalReading suppressed{};
  suppressed.valid = true;
  suppressed.sensorType = runtimeConfig.environmentalSensorType ==
      lil::protocol::EnvironmentalSensorType::kAutoDetect ?
      rtcState.lastSensorType : runtimeConfig.environmentalSensorType;
  ++rtcState.sequence;
  auto packet = makeTelemetryPacket(suppressed, battery,
      lil::protocol::SensorOperatingMode::kBatteryProtection);
  packet.payload.flags |= lil::protocol::kBatteryProtectionActive;
  lil::protocol::finalize(packet, lil::protocol::MessageType::kTelemetry,
                         rtcState.sequence);
  if (operatingMode(runtimeConfig) == OperatingMode::kEnergySaving &&
      espNowTransport.begin()) {
    // One bounded reporting window on the saved channel, even if the station
    // is offline. A failed delivery must not trigger an early retry wake.
    const auto exchange = espNowTransport.exchangeLpChannel(
        packet, runtimeConfig, runtimeConfig.wifiChannel);
    if (exchange.configReceived) {
      rtcState.lastStationRssi = exchange.stationRssi;
      sensor::confirmOtaBootAfterContact();
    }
    espNowTransport.end();
  }
  rtcState.lastReportLogicalMs = logicalNowMs();
  rtcState.hasAttemptedReport = true;
  SENSOR_LOG_PRINTLN("[POWER] Battery protection: sensors off, next report in 24 hours");
  sensor::finishOtaBootGuard();
  sensor::SleepController::deepSleep(kBatteryProtectionSleepSeconds, powerController);
}

// Normal precision snapshots and manual sessions have independent clocks.
// Environmental sleep / BSEC paths never allocate or initialize these objects.
void runLiveMode(sensor::BatteryReading battery) {
  const auto activeType = environmentalSensor.detectedType();
  sensor::RecordingStore recordings;
  recordings.begin(activeType);
  sensor::LiveAcquisition acquisition;
  if (!acquisition.begin(environmentalSensor, powerController, activeType,
                         runtimeConfig.temperatureOffsetC)) {
    sensor::SleepController::deepSleep(5, powerController);
  }
  const auto mode = activeType == lil::protocol::EnvironmentalSensorType::kLsm6dsox ?
      lil::protocol::SensorOperatingMode::kContinuousMotion : lil::protocol::SensorOperatingMode::kContinuousPrecision;
  acquisition.normal(runtimeConfig.sleepSeconds);
  uint32_t handledPresses = 0, lastTelemetry = millis() - 5000, lastStatus = millis() - 1000;
  uint32_t lastBattery = millis(), batteryStarted = 0, lastOta = millis() - 30000;
  uint32_t lastRecovery = millis(), nextRadio = 0, nextSync = 0;
  bool batteryPending = false, radioReady = espNowTransport.begin(), haveLatest = false, haveMeasurement = false;
  lil::protocol::TelemetryPacket latest{};
  uint64_t latestCapturedMs = 0;
  auto collect = [&]() {
    sensor::LiveCapture capture{};
    while (acquisition.take(capture)) {
      ++rtcState.sequence;
      latest = makeTelemetryPacket(capture.reading, battery, capture.recording ? mode :
          lil::protocol::SensorOperatingMode::kEnergySaving);
      latestCapturedMs = capture.capturedMs; haveLatest = haveMeasurement = true;
      if (capture.recording && recordings.recording() &&
          !recordings.append(capture.capturedMs, latest.payload)) acquisition.stop();
    }
  };
  for (;;) {
    if (handledPresses != acquisition.presses()) {
      ++handledPresses;
      if (recordings.recording()) {
        acquisition.stop();
        while (acquisition.active()) { collect(); delay(1); }
        collect(); recordings.stop();
        acquisition.normal(runtimeConfig.sleepSeconds);
      } else {
        acquisition.stop();
        while (acquisition.active()) delay(1);
        sensor::LiveCapture discarded{};
        while (acquisition.take(discarded)) {}
        if (recordings.start()) acquisition.start();
        else acquisition.normal(runtimeConfig.sleepSeconds);
      }
      lastStatus = millis() - 1000;
    }
    collect();
    const uint32_t now = millis();
    if (!batteryPending && now - lastBattery >=
        (SENSOR_LOW_BATTERY_PAUSE_MV > 0 ? 10000UL : 60000UL)) {
      adcReader.startBatteryMeasurement(); batteryStarted = now; batteryPending = true;
    }
    if (batteryPending && now - batteryStarted >= sensor::kHardware.adcSettleMs) {
      battery = adcReader.finishBatteryMeasurement(runtimeConfig.batteryCalibrationFactor);
      batteryPending = false; lastBattery = now;
      if constexpr (SENSOR_LOW_BATTERY_PAUSE_MV > 0) {
        if (batteryProtectionRequired(battery)) {
          acquisition.stop(); while (acquisition.active()) { collect(); delay(1); }
          collect(); if (recordings.recording()) recordings.stop();
          if (radioReady) { espNowTransport.end(); radioReady = false; }
          reportBatteryAndSleep(battery);
        }
      }
    }
    if (static_cast<int32_t>(now - nextRadio) >= 0 &&
        (haveLatest || now - lastTelemetry >= 5000)) {
      if (!radioReady) radioReady = espNowTransport.begin();
      auto outgoing = latest;
      if (!haveMeasurement) {
        sensor::EnvironmentalReading idle{}; idle.sensorType = activeType;
        idle.valid = true;
        outgoing = makeTelemetryPacket(idle, battery, recordings.recording() ? mode :
            lil::protocol::SensorOperatingMode::kEnergySaving);
      } else {
        const uint64_t elapsed = uint64_t(esp_timer_get_time() / 1000) - latestCapturedMs;
        auto addAge = [elapsed](uint16_t age) { return age == UINT16_MAX ? age :
            static_cast<uint16_t>(min(uint64_t(65534), uint64_t(age) + elapsed)); };
        outgoing.payload.live.acquisitionAgeMs = addAge(outgoing.payload.live.acquisitionAgeMs);
        outgoing.payload.live.estimateAgeMs = addAge(outgoing.payload.live.estimateAgeMs);
        for (auto& frame : outgoing.payload.live.optical) frame.ageMs = addAge(frame.ageMs);
        if (!haveLatest) {
          outgoing.payload.live.flags &= ~(lil::protocol::kLiveSampleFresh |
              lil::protocol::kLiveEstimateFresh | lil::protocol::kLiveGap);
          outgoing.payload.live.count = 0;
          outgoing.payload.motion.fifoOverrun = 0;
        }
      }
      outgoing.payload.operatingMode = recordings.recording() ? mode :
          lil::protocol::SensorOperatingMode::kEnergySaving;
      updatePacketBattery(outgoing, battery);
      outgoing.payload.appliedConfigRevision = runtimeConfig.revision;
      lil::protocol::finalize(outgoing, lil::protocol::MessageType::kTelemetry, ++rtcState.sequence);
      haveLatest = false; lastTelemetry = now;
      sensor::ExchangeResult exchange{};
      if (radioReady) exchange = espNowTransport.exchange(outgoing, runtimeConfig, sensor::otaBootPending());
      if (radioReady && !exchange.configReceived && now - lastRecovery >= 5000) {
        lastRecovery = now;
        exchange = espNowTransport.exchangeLpChannel(outgoing, runtimeConfig,
            takeNextRecoveryChannel(runtimeConfig.wifiChannel), true);
      }
      if (exchange.configReceived) {
        rtcState.lastStationRssi = exchange.stationRssi;
        const auto previousType = runtimeConfig.environmentalSensorType;
        const float previousOffset = runtimeConfig.temperatureOffsetC;
        applyStationConfig(exchange.config);
        acquisition.setNormalInterval(runtimeConfig.sleepSeconds);
        if (!runtimeConfig.provisioned ||
            runtimeConfig.environmentalSensorType != previousType ||
            runtimeConfig.temperatureOffsetC != previousOffset ||
            (runtimeConfig.environmentalSensorType != lil::protocol::EnvironmentalSensorType::kAutoDetect &&
             runtimeConfig.environmentalSensorType != activeType)) {
          acquisition.stop(); while (acquisition.active()) { collect(); delay(1); }
          collect(); if (recordings.recording()) recordings.stop();
          esp_restart();
        }
        if (!recordings.recording() &&
            (sensor::otaBootPending() || now - lastOta >= 30000)) {
          lastOta = now;
          acquisition.pauseNormal(true);
          while (acquisition.active()) { collect(); delay(1); }
          sensor::checkOta(espNowTransport, runtimeConfig, adcReader, battery.millivolts);
          acquisition.pauseNormal(false);
        }
        nextRadio = millis();
      } else {
        nextRadio = millis() + 1000;
        if (radioReady) { espNowTransport.end(); radioReady = false; }
      }
    }
    if (radioReady && now - lastStatus >= 1000) {
      lil::recording::StatusPacket status{};
      status.payload = recordings.status(acquisition.dropped());
      lil::protocol::finalize(status, lil::recording::kStatusMessage, ++rtcState.sequence);
      lil::recording::Ack ack{}; const uint32_t started = millis();
      if (espNowTransport.recordingExchange(runtimeConfig, &status, sizeof(status),
          status.payload.session, status.payload.elapsedMs, 0, ack))
        recordings.anchor(ack.stationEpochMs, status.payload.elapsedMs, millis() - started);
      lastStatus = millis();
    }
    if (radioReady && !recordings.recording() && static_cast<int32_t>(now - nextSync) >= 0) {
      lil::recording::UploadPacket upload{};
      if (recordings.next(upload.payload)) {
        lil::protocol::finalize(upload, lil::recording::kRecordMessage, ++rtcState.sequence);
        lil::recording::Ack ack{}; const auto& record = upload.payload.record;
        if (espNowTransport.recordingExchange(runtimeConfig, &upload, sizeof(upload),
            record.session, record.sampleMs, lil::recording::checksum(record), ack) && ack.stored)
          recordings.acknowledge(record);
        else nextSync = millis() + 1000;
      }
    }
    if (!recordings.recording() && !acquisition.normal() && handledPresses == acquisition.presses())
      acquisition.normal(runtimeConfig.sleepSeconds);
    sensor::finishOtaBootGuard();
    if (!recordings.recording() && !acquisition.active() && !recordings.status(0).pending && !batteryPending) {
      if (radioReady) { espNowTransport.end(); radioReady = false; }
      // Publish the pause before sleep so a due normal window cannot start
      // I2C between the inactive check and light-sleep entry.
      acquisition.pauseNormal(true);
      while (acquisition.active()) { collect(); delay(1); }
      // GPIO9 can wake from light sleep, but not from C3 deep sleep. This path
      // exists only for manual live sensors; BME deep-sleep cadence is unchanged.
      gpio_wakeup_enable(GPIO_NUM_9, GPIO_INTR_LOW_LEVEL);
      esp_sleep_enable_gpio_wakeup(); esp_sleep_enable_timer_wakeup(100000);
      esp_light_sleep_start();
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
      gpio_wakeup_disable(GPIO_NUM_9);
      acquisition.pauseNormal(false);
    }
    delay(5);
  }
}

}  // namespace

void setup() {
  sensor::beginOtaBootGuard();
  sensor::beginDiagnosticLogging();
  initializeRtcState();

  if (!configStore.begin()) {
    powerController.begin();
    sensor::SleepController::deepSleep(sensor::kHardware.pcbVersion == 4 ?
        kBatteryProtectionSleepSeconds : 60, powerController);
  }
  if (configStore.firmwareChanged()) {
    SENSOR_LOG_PRINTLN(
        "[SETUP] New firmware image: retaining pairing and calibration");
  }
  runtimeConfig = configStore.load();
  const OperatingMode startupMode = operatingMode(runtimeConfig);
  if (rtcStateInitializedThisBoot &&
      startupMode == OperatingMode::kEnergySaving) {
    // A cold boot may happen in the middle of captive-portal commissioning.
    // Keep one complete recovery available until a changed channel is found.
    rtcState.fullChannelRecoveryPending = true;
  }

  powerController.begin();
  adcReader.begin(powerController);

  sensor::BatteryReading preflightBattery{};
  if constexpr (SENSOR_LOW_BATTERY_PAUSE_MV > 0) {
    preflightBattery = adcReader.readBattery(runtimeConfig.batteryCalibrationFactor);
    if (batteryProtectionRequired(preflightBattery)) reportBatteryAndSleep(preflightBattery);
    rtcState.batteryPaused = false;
  }
  sensor::EnvironmentalReading environment{};
  sensor::BatteryReading battery{};
  const bool energySavingMode =
      startupMode == OperatingMode::kEnergySaving;
  const bool discoveryMode = !energySavingMode;
  const bool sensorRecoveryWake = rtcState.sensorRecoveryPending;
  bool reportDue =
      sensor::otaBootPending() || sensorRecoveryWake || discoveryMode || lil::power::reportDue(
          rtcState.hasAttemptedReport, logicalNowMs(),
          rtcState.lastReportLogicalMs, runtimeConfig.sleepSeconds);
  const bool pairingMeasurementDue =
      discoveryMode &&
      (!rtcState.hasPairingSnapshot ||
       logicalNowMs() - rtcState.pairingMeasurementMs >=
           uint64_t(kPairingMeasurementSeconds) * 1000);
  const bool measurementDue = sensorRecoveryWake || !discoveryMode || pairingMeasurementDue;
  if (measurementDue) {
    // Reuse the protection preflight voltage. With protection disabled, start
    // the divider before the environmental conversion to overlap settling;
    // maintenance wakes without a report then need no battery conversion.
    bool batteryMeasurementStarted = false;
    if (reportDue && !preflightBattery.valid) {
      adcReader.startBatteryMeasurement();
      batteryMeasurementStarted = true;
    }
    const bool sensorStarted = environmentalSensor.begin(
        powerController, runtimeConfig.environmentalSensorType,
        runtimeConfig.temperatureOffsetC);
    if (sensorStarted) {
      rtcState.lastSensorType = environmentalSensor.detectedType();
      rtcState.sensorRecoveryPending = false;
      rtcState.sensorStartAttemptsRemaining = kSensorStartAttempts;
    } else if (rtcState.sensorStartAttemptsRemaining > 0) {
      --rtcState.sensorStartAttemptsRemaining;
      rtcState.sensorRecoveryPending = rtcState.sensorStartAttemptsRemaining > 0;
      // Report failed initialization and each retry promptly, including BME680
      // maintenance wakes that normally leave the radio off.
      reportDue = true;
    }
    environment = environmentalSensor.read();
    const bool continuousLive = sensorStarted && energySavingMode &&
        lil::protocol::isLiveSensor(environment.sensorType);
    if (!continuousLive) environmentalSensor.end();
    // Preserve the original report timing when a long BME680 conversion
    // crosses the configured deadline. This rare boundary case deliberately
    // pays the full ADC settling time instead of delaying data by five minutes.
    if (!reportDue) {
      reportDue = lil::power::reportDue(
          rtcState.hasAttemptedReport, logicalNowMs(),
          rtcState.lastReportLogicalMs, runtimeConfig.sleepSeconds);
    }
    if (reportDue) {
      battery = preflightBattery.valid ? preflightBattery : batteryMeasurementStarted
                    ? adcReader.finishBatteryMeasurement(
                          runtimeConfig.batteryCalibrationFactor)
                    : adcReader.readBattery(
                          runtimeConfig.batteryCalibrationFactor);
    }
    if (continuousLive) runLiveMode(battery);
    if (discoveryMode) {
      rtcState.cachedEnvironment = environment;
      rtcState.cachedBattery = battery;
      rtcState.hasPairingSnapshot = true;
      rtcState.pairingMeasurementMs = logicalNowMs();
    }
  } else {
    environment = rtcState.cachedEnvironment;
    battery = rtcState.cachedBattery;
  }

  const bool bme680Active =
      environment.sensorType ==
      lil::protocol::EnvironmentalSensorType::kBme680;
  auto packet = makeTelemetryPacket(
      environment, battery,
      discoveryMode ? lil::protocol::SensorOperatingMode::kDiscovery
                    : lil::protocol::SensorOperatingMode::kEnergySaving);

  sensor::ExchangeResult exchange{};
  // BME680 still wakes internally every five minutes to maintain BSEC/IAQ,
  // but once provisioned it must transmit strictly at the configured report
  // interval. Calibration must not silently increase the radio cadence.
  if (reportDue && discoveryMode) {
    sensor::lowPowerSensorWaitUs((esp_random() % 121U) * 1000U);
  }
  if (reportDue && espNowTransport.begin()) {
    exchange = espNowTransport.exchange(packet, runtimeConfig, sensor::otaBootPending());
    // A successful unicast MAC acknowledgement proves that the station was on
    // the configured channel. If only the application response was lost, a
    // maximum-power channel scan cannot help and merely extends the wake.
    if (energySavingMode && !exchange.configReceived && !exchange.delivered) {
      auto recoveryPacket = packet;
      recoveryPacket.payload.operatingMode =
          lil::protocol::SensorOperatingMode::kChannelRecovery;
      lil::protocol::finalize(
          recoveryPacket, lil::protocol::MessageType::kTelemetry,
          rtcState.sequence);
      const uint8_t recoveryChannelCount =
          rtcState.fullChannelRecoveryPending
              ? 12
              : kRecoveryChannelsPerReport;
      // Spend the full-scan allowance once, even if the station is offline.
      rtcState.fullChannelRecoveryPending = false;
      for (uint8_t index = 0; index < recoveryChannelCount; ++index) {
        const uint8_t recoveryChannel =
            takeNextRecoveryChannel(runtimeConfig.wifiChannel);
        auto recovery = espNowTransport.exchangeLpChannel(
            recoveryPacket, runtimeConfig, recoveryChannel, true);
        const bool delivered = exchange.delivered || recovery.delivered;
        if (recovery.configReceived) {
          exchange = recovery;
          exchange.delivered = delivered;
          rtcState.fullChannelRecoveryPending = false;
          break;
        }
        exchange.delivered = delivered;
        if (recovery.delivered) {
          runtimeConfig.wifiChannel = recoveryChannel;
          break;
        }
      }
    }
    if (exchange.configReceived && exchange.config.provisioned) {
      // Use the confirmed station/channel for the pull-based OTA exchange.
      auto otaConfig = runtimeConfig;
      otaConfig.provisioned = true;
      otaConfig.stationKnown = true;
      memcpy(otaConfig.stationMac, exchange.config.stationMac, 6);
      sensor::checkOta(espNowTransport, otaConfig, adcReader, battery.millivolts);
    }
    espNowTransport.end();
    if (exchange.delivered && !exchange.configReceived) {
      configStore.saveIfChanged(runtimeConfig);
    }
  }

  bool measurementChanged = false;
  if (exchange.configReceived) {
    rtcState.lastStationRssi = exchange.stationRssi;
    measurementChanged = applyStationConfig(exchange.config);
    rtcState.lastReportLogicalMs = logicalNowMs();
    rtcState.hasAttemptedReport = true;
  } else if (reportDue) {
    if (energySavingMode) {
      // A failed exchange is still a radio attempt. Start a fresh configured
      // interval instead of retrying at the BME680's internal 5-minute
      // measurement cadence.
      rtcState.lastReportLogicalMs = logicalNowMs();
      rtcState.hasAttemptedReport = true;
    }
  }

  const bool useBme680Cadence =
      runtimeConfig.environmentalSensorType ==
          lil::protocol::EnvironmentalSensorType::kBme680 ||
      (runtimeConfig.environmentalSensorType ==
           lil::protocol::EnvironmentalSensorType::kAutoDetect &&
       bme680Active);
  uint32_t sleepSeconds =
      useBme680Cadence
          ? environmentalSensor.bme680RecommendedSleepSeconds(300UL)
          : runtimeConfig.sleepSeconds;
  if (measurementChanged) {
    // Reprobe a changed type (including auto detection) and invalidate cached
    // discovery readings promptly, rather than waiting the previous interval.
    sleepSeconds = 1;
    rtcState.hasPairingSnapshot = false;
    rtcState.discoveryStartedMs = logicalNowMs();
    rtcState.pairingMeasurementMs = logicalNowMs();
  } else if (rtcState.sensorRecoveryPending) {
    sleepSeconds = kSensorRecoverySleepSeconds;
  } else if (operatingMode(runtimeConfig) == OperatingMode::kDiscovery) {
    sleepSeconds = lil::power::discoverySleepSeconds(
        (logicalNowMs() - rtcState.discoveryStartedMs) / 1000,
        SENSOR_STORAGE_DISCOVERY_MAX_SECONDS);
  }
  const bool energySavingModeAfterExchange =
      operatingMode(runtimeConfig) == OperatingMode::kEnergySaving;
  if (energySavingModeAfterExchange) {
    rtcState.discoveryStartedMs = logicalNowMs();
  }
  if (!discoveryMode) {
    rtcState.hasPairingSnapshot = false;
  }
  if (energySavingModeAfterExchange && !measurementChanged) {
    if (!rtcState.sensorRecoveryPending) {
      sleepSeconds = lil::power::scheduledSleepSeconds(
          rtcState.hasAttemptedReport, logicalNowMs(),
          rtcState.lastReportLogicalMs + uint64_t(runtimeConfig.sleepSeconds) * 1000,
          sleepSeconds);
    }
    environmentalSensor.prepareForDeepSleep(sleepSeconds,
                                             environment.sensorType);
  }
  const uint32_t sleepPhaseMs = rtcState.initialSleepPhaseApplied
                                    ? 0
                                    : rtcState.initialSleepPhaseMs;
  rtcState.initialSleepPhaseApplied = true;
  sensor::finishOtaBootGuard();
  sensor::SleepController::deepSleep(sleepSeconds, powerController,
                                     sleepPhaseMs);
}

void loop() {}

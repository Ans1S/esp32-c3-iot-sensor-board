#include <Arduino.h>
#include <memory>
#include <new>

#include <esp_random.h>
#include <esp_timer.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#if defined(ESP_PLATFORM) && defined(CONFIG_PM_ENABLE)
#include <esp_pm.h>
#endif

#include "adc_reader.h"
#include "live_acquisition.h"
#include "recording_store.h"
#include "ota_client.h"
#include "environmental_sensor.h"
#include "espnow_transport.h"
#include "hardware_profile.h"
#include "lil_protocol.h"
#include "power_controller.h"
#include "radio_benchmark.h"
#include "sensor_config_store.h"
#include "sensor_log.h"
#include "sleep_controller.h"
#include "logical_clock.h"
#include "low_power_wait.h"
#include "power_policy.h"
#include "power_options.h"
#include "precision_cadence.h"

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
    espNowTransport.end();
    rtcState.acknowledgedResetRevision = response.revision;
    rtcState.initialSleepPhaseMs = static_cast<uint16_t>(
        esp_random() % (kInitialSleepPhaseWindowMs + 1UL));
    rtcState.initialSleepPhaseApplied = false;
    rtcState.nextRecoveryChannel = 1;
    rtcState.fullChannelRecoveryPending = false;
    rtcState.discoveryStartedMs = 0;
    rtcState.pairingMeasurementMs = 0;
    rtcState.hasPairingSnapshot = false;
    rtcState.cachedEnvironment = {};
    rtcState.cachedBattery = {};
    rtcState.batteryPaused = false;
    rtcState.hasAttemptedReport = false;
    rtcState.sensorRecoveryPending = true;
    rtcState.sensorStartAttemptsRemaining = kSensorStartAttempts;
    rtcState.lastSensorType =
        lil::protocol::EnvironmentalSensorType::kAutoDetect;
    sensor::SleepController::deepSleep(1, powerController);
  }

  const bool iaqResetRequested =
      (response.flags & lil::protocol::kResetIaqCalibration) != 0 &&
      response.revision != runtimeConfig.revision;

  const bool sensorTypeValid =
      lil::protocol::isValidSensorType(response.sensorType);
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
  packet.payload.extended = environment.extended;
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
  // Payload preparation is separate from wire serialization. The transport
  // chooses the compatible length, refreshes ages and finalizes each send.
  packet.header.sequence = rtcState.sequence;
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
  if (operatingMode(runtimeConfig) == OperatingMode::kEnergySaving &&
      espNowTransport.begin()) {
    // One bounded reporting window on the saved channel, even if the station
    // is offline. A failed delivery must not trigger an early retry wake.
    WCH_RADIO_BENCHMARK_BEGIN();
    const auto exchange = espNowTransport.exchangeLpChannel(
        packet, runtimeConfig, runtimeConfig.wifiChannel);
    WCH_RADIO_BENCHMARK_FINISH(true, packet, exchange);
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
struct LiveModeState {
  sensor::RecordingStore recordings;
  sensor::LiveAcquisition acquisition;
  lil::protocol::TelemetryPacket latest{};
  lil::protocol::TelemetryPacket outgoing{};
  sensor::ExchangeResult exchange{};
  sensor::LiveCapture capture{};
  lil::recording::UploadPacket upload{};
};

[[gnu::noinline]] void runLiveMode(sensor::BatteryReading battery) {
  const auto activeType = environmentalSensor.detectedType();
  std::unique_ptr<LiveModeState> state(new (std::nothrow) LiveModeState());
  if (!state) {
    sensor::finishOtaBootGuard();
    sensor::SleepController::deepSleep(5, powerController);
  }
  auto& recordings = state->recordings;
  auto& acquisition = state->acquisition;
  auto& latest = state->latest;
  auto& outgoing = state->outgoing;
  auto& exchange = state->exchange;
  auto& capture = state->capture;
  auto& upload = state->upload;
  sensor::pushLowPowerWaitInhibit();
  recordings.begin(activeType);
  if (!acquisition.begin(environmentalSensor, powerController, activeType,
                         runtimeConfig.temperatureOffsetC)) {
    sensor::popLowPowerWaitInhibit();
    sensor::finishOtaBootGuard();
    sensor::SleepController::deepSleep(5, powerController);
  }
  const auto mode = activeType == lil::protocol::EnvironmentalSensorType::kLsm6dsox ?
      lil::protocol::SensorOperatingMode::kContinuousMotion : lil::protocol::SensorOperatingMode::kContinuousPrecision;
  acquisition.normal(runtimeConfig.sleepSeconds);
  uint32_t handledPresses = 0, lastTelemetry = millis() - 5000, lastStatus = millis() - 1000;
  uint32_t lastBattery = millis(), batteryStarted = 0, lastOta = millis() - 30000;
  uint32_t lastRecovery = millis();
  lil::timing::RetryBackoff radioBackoff, syncBackoff;
  uint8_t consecutiveLiveFailures = 0;
  bool batteryPending = false, radioReady = espNowTransport.begin(), haveLatest = false, haveMeasurement = false;
  uint64_t latestCapturedMs = 0;
  auto collect = [&]() {
    while (acquisition.take(capture)) {
      ++rtcState.sequence;
      latest = makeTelemetryPacket(capture.reading, battery, capture.recording ? mode :
          lil::protocol::SensorOperatingMode::kEnergySaving);
      latestCapturedMs = capture.capturedMs; haveLatest = haveMeasurement = true;
      if (capture.recording && recordings.recording() &&
          !recordings.append(capture.capturedMs, latest.payload)) acquisition.stop();
    }
  };
  auto waitForAcquisitionRelease = [&](bool collectPending) {
    const uint32_t started = millis();
    while (acquisition.active()) {
      if (collectPending) collect();
      if (uint32_t(millis() - started) >= 10000) {
        // Do not cut power while a stuck task may still own I2C. Restart so
        // startup can release the rail and recheck battery protection.
        SENSOR_LOG_PRINTLN("[LIVE] Acquisition did not release; restarting");
        esp_restart();
      }
      delay(1);
    }
  };
  for (;;) {
    const uint32_t currentPresses = acquisition.presses();
    if (handledPresses != currentPresses) {
      const uint32_t delta = currentPresses - handledPresses;
      handledPresses = currentPresses;
      if ((delta & 1U) != 0U) {
        if (recordings.recording()) {
          acquisition.stop();
          waitForAcquisitionRelease(true);
          collect(); recordings.stop();
          acquisition.normal(runtimeConfig.sleepSeconds);
        } else {
          acquisition.stop();
          waitForAcquisitionRelease(false);
          while (acquisition.take(capture)) {}
          if (recordings.start()) acquisition.start();
          else acquisition.normal(runtimeConfig.sleepSeconds);
        }
        lastStatus = millis() - 1000;
      } else if (recordings.recording()) {
        acquisition.stop();
        waitForAcquisitionRelease(true);
        collect(); recordings.stop();
        acquisition.normal(runtimeConfig.sleepSeconds);
        lastStatus = millis() - 1000;
      }
    }
    collect();
    uint32_t now = millis();
    if (radioReady && espNowTransport.sendStalled(now, 5000)) {
      espNowTransport.end();
      radioReady = espNowTransport.begin();
    }
    if (!batteryPending && now - lastBattery >=
        (SENSOR_LOW_BATTERY_PAUSE_MV > 0 ? 10000UL : 60000UL)) {
      adcReader.startBatteryMeasurement(); batteryStarted = now; batteryPending = true;
    }
    if (batteryPending && now - batteryStarted >= sensor::kHardware.adcSettleMs) {
      battery = adcReader.finishBatteryMeasurement(runtimeConfig.batteryCalibrationFactor);
      batteryPending = false; lastBattery = now;
      if constexpr (SENSOR_LOW_BATTERY_PAUSE_MV > 0) {
        if (batteryProtectionRequired(battery)) {
          // Preserve hysteresis across a watchdog restart while waiting for
          // the acquisition task to release the rail.
          rtcState.batteryPaused = true;
          acquisition.stop(); waitForAcquisitionRelease(true);
          collect(); if (recordings.recording()) recordings.stop();
          if (radioReady) { espNowTransport.end(); radioReady = false; }
          sensor::popLowPowerWaitInhibit();
          reportBatteryAndSleep(battery);
        }
      }
    }
    if (radioBackoff.ready(now) &&
        (haveLatest || now - lastTelemetry >= 5000)) {
      if (!radioReady) radioReady = espNowTransport.begin();
      outgoing = latest;
      if (!haveMeasurement) {
        sensor::EnvironmentalReading idle{}; idle.sensorType = activeType;
        idle.valid = true;
        outgoing = makeTelemetryPacket(idle, battery, recordings.recording() ? mode :
            lil::protocol::SensorOperatingMode::kEnergySaving);
      } else {
        const uint64_t nowMs64 = lil::timing::signedUsToMs(esp_timer_get_time());
        const uint64_t elapsed = nowMs64 >= latestCapturedMs ? nowMs64 - latestCapturedMs : 0ULL;
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
      outgoing.header.sequence = ++rtcState.sequence;
      haveLatest = false; lastTelemetry = now;
      exchange = {};
      const bool liveStreaming = recordings.recording() && !sensor::otaBootPending();
      WCH_RADIO_BENCHMARK_BEGIN();
      if (radioReady) exchange = espNowTransport.exchange(outgoing, runtimeConfig, sensor::otaBootPending(), liveStreaming);
      // A successful MAC delivery confirms the channel even if the station
      // is too busy to return its application response.
      if (radioReady && !exchange.configReceived && !exchange.delivered &&
          (!liveStreaming || consecutiveLiveFailures >= 2) &&
          now - lastRecovery >= 5000) {
        lastRecovery = now;
        const uint8_t recoveryChannel =
            takeNextRecoveryChannel(runtimeConfig.wifiChannel);
        exchange = espNowTransport.exchangeLpChannel(outgoing, runtimeConfig,
            recoveryChannel, true);
        if (exchange.delivered && !exchange.configReceived) {
          runtimeConfig.wifiChannel = recoveryChannel;
          configStore.saveIfChanged(runtimeConfig);
        }
      }
      WCH_RADIO_BENCHMARK_FINISH(radioReady, outgoing, exchange);
      if (exchange.configReceived) {
        rtcState.lastStationRssi = exchange.stationRssi;
        // Confirm an OTA trial boot on first station contact even if a manual
        // recording is running or the periodic OTA check is not due yet.
        sensor::confirmOtaBootAfterContact();
        if ((exchange.config.flags & lil::protocol::kFactoryReset) != 0) {
          acquisition.stop();
          waitForAcquisitionRelease(true);
          collect();
          if (recordings.recording()) recordings.stop();
          recordings.clearForReset();
          if (radioReady) { espNowTransport.end(); radioReady = false; }
          if (!applyStationConfig(exchange.config)) {
            acquisition.normal(runtimeConfig.sleepSeconds);
          }
        } else {
          const auto previousType = runtimeConfig.environmentalSensorType;
          const float previousOffset = runtimeConfig.temperatureOffsetC;
          applyStationConfig(exchange.config);
          acquisition.setNormalInterval(runtimeConfig.sleepSeconds);
          if (!runtimeConfig.provisioned ||
              runtimeConfig.environmentalSensorType != previousType ||
              runtimeConfig.temperatureOffsetC != previousOffset ||
              (runtimeConfig.environmentalSensorType != lil::protocol::EnvironmentalSensorType::kAutoDetect &&
               runtimeConfig.environmentalSensorType != activeType)) {
            acquisition.stop(); waitForAcquisitionRelease(true);
            collect(); if (recordings.recording()) recordings.stop();
            esp_restart();
          }
          if (!recordings.recording() &&
              (sensor::otaBootPending() || millis() - lastOta >= 30000)) {
            lastOta = millis();
            acquisition.pauseNormal(true);
            waitForAcquisitionRelease(true);
            if (batteryPending) {
              battery = adcReader.finishBatteryMeasurement(runtimeConfig.batteryCalibrationFactor);
              batteryPending = false;
              lastBattery = millis();
            }
            espNowTransport.configureLink(runtimeConfig.wifiChannel,
                                          runtimeConfig.txPowerQuarterDbm);
            const auto batteryBeforeOta = battery;
            sensor::checkOta(espNowTransport, runtimeConfig, adcReader, battery);
            if (battery.valid != batteryBeforeOta.valid ||
                battery.millivolts != batteryBeforeOta.millivolts) {
              lastBattery = millis();
            }
            if constexpr (SENSOR_LOW_BATTERY_PAUSE_MV > 0) {
              if (batteryProtectionRequired(battery)) {
                espNowTransport.end(); radioReady = false;
                sensor::popLowPowerWaitInhibit();
                reportBatteryAndSleep(battery);
              }
            }
            acquisition.pauseNormal(false);
          }
        }
        consecutiveLiveFailures = 0;
        radioBackoff.clear();
      } else if (liveStreaming && exchange.delivered) {
        // The station received the live telemetry frame via MAC ACK; keep the
        // radio active instead of inserting a 1-second backoff on a dropped reply.
        consecutiveLiveFailures = 0;
        radioBackoff.clear();
      } else if (liveStreaming && ++consecutiveLiveFailures < 3) {
        // Tolerate brief Wi-Fi collisions during fast live recording without
        // tearing down ESP-NOW on a single missed frame.
        radioBackoff.clear();
      } else {
        consecutiveLiveFailures = 0;
        radioBackoff.wait(millis(), 1000);
        if (radioReady) { espNowTransport.end(); radioReady = false; }
      }
      now = millis();
    }
    if (radioReady && now - lastStatus >= 1000) {
      lil::recording::StatusPacket status{};
      const uint32_t started = millis();
      status.payload = recordings.status(acquisition.dropped());
      lil::protocol::finalize(status, lil::recording::kStatusMessage, ++rtcState.sequence);
      lil::recording::Ack ack{};
      if (espNowTransport.recordingExchange(runtimeConfig, &status, sizeof(status),
          status.payload.session, status.payload.elapsedMs, 0, ack,
          recordings.recording() ? 120U : 0U))
        recordings.anchor(ack.stationEpochMs, status.payload.elapsedMs, millis() - started);
      lastStatus = millis();
      now = lastStatus;
    }
    bool synchronizedRecord = false;
    if (radioReady && !recordings.recording() && syncBackoff.ready(now)) {
      upload = {};
      if (recordings.next(upload.payload)) {
        lil::protocol::finalize(upload, lil::recording::kRecordMessage, ++rtcState.sequence);
        lil::recording::Ack ack{}; const auto& record = upload.payload.record;
        if (espNowTransport.recordingExchange(runtimeConfig, &upload, sizeof(upload),
            record.session, record.sampleMs, lil::recording::checksum(record), ack) && ack.stored &&
            recordings.acknowledge(record)) { syncBackoff.clear(); synchronizedRecord = true; }
        else syncBackoff.wait(millis(), 1000);
      }
    }
    if (!recordings.recording() && !acquisition.normal() && handledPresses == acquisition.presses())
      acquisition.normal(runtimeConfig.sleepSeconds);
    // Do not call finishOtaBootGuard() unconditionally here; the 60-second
    // boot timer handles rollback if the station remains unreachable, while
    // first contact confirms the image via confirmOtaBootAfterContact().
    const uint32_t sleepNow = millis();
    const auto recStatus = recordings.status(0);
    const bool pendingSync = recStatus.state != lil::recording::State::Fault &&
                             recStatus.pending > 0;
    const bool canSleepForBackoff = !pendingSync || !syncBackoff.ready(sleepNow);
    bool buttonIdle = true;
#ifdef ESP_PLATFORM
    buttonIdle = digitalRead(static_cast<uint8_t>(GPIO_NUM_9)) == HIGH;
#endif
    if (!recordings.recording() && !acquisition.active() && !batteryPending &&
        !haveLatest && handledPresses == acquisition.presses() &&
        canSleepForBackoff && buttonIdle) {
      if (radioReady) {
        espNowTransport.end();
        radioReady = false;
      }
      // Publish the pause before sleep so a due normal window cannot start
      // I2C between the inactive check and light-sleep entry.
      acquisition.pauseNormal(true);
      waitForAcquisitionRelease(true);
      if (!haveLatest && handledPresses == acquisition.presses()) {
        const uint32_t wakeNow = millis();
        const uint32_t batteryIntervalMs =
            SENSOR_LOW_BATTERY_PAUSE_MV > 0 ? 10000UL : 60000UL;
        const uint32_t telemetryRemaining =
            wakeNow - lastTelemetry >= 5000UL
                ? 10UL
                : 5000UL - (wakeNow - lastTelemetry);
        const uint32_t batteryRemaining =
            wakeNow - lastBattery >= batteryIntervalMs
                ? 10UL
                : batteryIntervalMs - (wakeNow - lastBattery);
        const uint32_t normalRemaining = acquisition.nextNormalDueMs(wakeNow);
        uint32_t sleepMs =
            min(telemetryRemaining, min(batteryRemaining, normalRemaining));
        if (pendingSync) {
          sleepMs = min(sleepMs, syncBackoff.remainingMs(wakeNow));
        }
        sleepMs = constrain(sleepMs, 10UL, 5000UL);
        // GPIO9 can wake from light sleep, but not from C3 deep sleep. This path
        // exists only for manual live sensors; BME deep-sleep cadence is unchanged.
#ifdef ESP_PLATFORM
        gpio_intr_disable(GPIO_NUM_9);
#endif
        gpio_wakeup_enable(GPIO_NUM_9, GPIO_INTR_LOW_LEVEL);
        esp_sleep_enable_gpio_wakeup();
        esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(sleepMs) * 1000ULL);
        (void)esp_light_sleep_start();
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        gpio_wakeup_disable(GPIO_NUM_9);
#ifdef ESP_PLATFORM
        gpio_set_intr_type(GPIO_NUM_9, GPIO_INTR_ANYEDGE);
        gpio_intr_enable(GPIO_NUM_9);
#endif
      }
      acquisition.pauseNormal(false);
    }
    // Yield between successful transfers while avoiding a fixed 5 ms per record.
    delay(synchronizedRecord ? 1 : 5);
  }
}

// CCS811 is a MOX gas sensor that requires continuous rail power so its
// micro-hotplate can complete the 20-minute burn-in and maintain its baseline.
// Wi-Fi is still shut down between scheduled reports, and PCB V4 battery
// protection overrides continuous sensing whenever voltage drops below 2800 mV.
[[gnu::noinline]] [[noreturn]] void runCcs811Mode(sensor::BatteryReading battery) {
  sensor::pushLowPowerWaitInhibit();
  uint32_t lastReport = millis();
  uint32_t lastBattery = lastReport;
  uint32_t lastOta = lastReport - 30000UL;
  uint32_t lastRecovery = lastReport;
  uint8_t consecutiveReadFailures = 0;
  bool firstReport = true;
  for (;;) {
    const uint32_t now = millis();
    const uint32_t batteryIntervalMs =
        SENSOR_LOW_BATTERY_PAUSE_MV > 0 ? 10000UL : 60000UL;
    if (now - lastBattery >= batteryIntervalMs) {
      battery = adcReader.readBattery(runtimeConfig.batteryCalibrationFactor);
      lastBattery = millis();
      if constexpr (SENSOR_LOW_BATTERY_PAUSE_MV > 0) {
        if (batteryProtectionRequired(battery)) {
          rtcState.batteryPaused = true;
          environmentalSensor.end();
          espNowTransport.end();
          sensor::popLowPowerWaitInhibit();
          reportBatteryAndSleep(battery);
        }
      }
    }
    const uint32_t intervalMs =
        static_cast<uint32_t>(max<uint32_t>(1UL, min<uint32_t>(runtimeConfig.sleepSeconds, 86400UL))) * 1000UL;
    if (firstReport || millis() - lastReport >= intervalMs) {
      firstReport = false;
      lastReport = millis();
      sensor::EnvironmentalReading reading = environmentalSensor.readContinuous();
      const bool ccsMaxResistanceOnly =
          !reading.valid &&
          reading.extended.airQuality.status != 0U &&
          reading.extended.airQuality.errorId == 0x08U;
      if (!reading.valid && !ccsMaxResistanceOnly) {
        if (++consecutiveReadFailures >= 6) {
          consecutiveReadFailures = 0;
          environmentalSensor.begin(powerController,
                                    lil::protocol::EnvironmentalSensorType::kCcs811,
                                    runtimeConfig.temperatureOffsetC);
        }
      } else {
        consecutiveReadFailures = 0;
      }
      if (millis() - lastBattery >= 5000UL) {
        battery = adcReader.readBattery(runtimeConfig.batteryCalibrationFactor);
        lastBattery = millis();
        if constexpr (SENSOR_LOW_BATTERY_PAUSE_MV > 0) {
          if (batteryProtectionRequired(battery)) {
            rtcState.batteryPaused = true;
            environmentalSensor.end();
            espNowTransport.end();
            sensor::popLowPowerWaitInhibit();
            reportBatteryAndSleep(battery);
          }
        }
      }
      ++rtcState.sequence;
      auto packet = makeTelemetryPacket(
          reading, battery, lil::protocol::SensorOperatingMode::kContinuousPrecision);
      if (espNowTransport.begin()) {
        WCH_RADIO_BENCHMARK_BEGIN();
        auto exchange = espNowTransport.exchange(packet, runtimeConfig, sensor::otaBootPending());
        if (!exchange.configReceived && !exchange.delivered &&
            millis() - lastRecovery >= 5000UL) {
          lastRecovery = millis();
          const uint8_t recoveryChannel =
              takeNextRecoveryChannel(runtimeConfig.wifiChannel);
          exchange = espNowTransport.exchangeLpChannel(
              packet, runtimeConfig, recoveryChannel, true);
          if (exchange.delivered && !exchange.configReceived) {
            runtimeConfig.wifiChannel = recoveryChannel;
            configStore.saveIfChanged(runtimeConfig);
          }
        }
        WCH_RADIO_BENCHMARK_FINISH(true, packet, exchange);
        if (exchange.configReceived) {
          rtcState.lastStationRssi = exchange.stationRssi;
          sensor::confirmOtaBootAfterContact();
          if ((exchange.config.flags & lil::protocol::kFactoryReset) != 0) {
            environmentalSensor.end();
            espNowTransport.end();
            sensor::popLowPowerWaitInhibit();
            applyStationConfig(exchange.config);
            esp_restart();
          }
          const auto previousType = runtimeConfig.environmentalSensorType;
          applyStationConfig(exchange.config);
          if (!runtimeConfig.provisioned ||
              (runtimeConfig.environmentalSensorType != previousType &&
               runtimeConfig.environmentalSensorType !=
                   lil::protocol::EnvironmentalSensorType::kAutoDetect &&
               runtimeConfig.environmentalSensorType !=
                   lil::protocol::EnvironmentalSensorType::kCcs811)) {
            environmentalSensor.end();
            espNowTransport.end();
            sensor::popLowPowerWaitInhibit();
            esp_restart();
          }
          if (sensor::otaBootPending() || millis() - lastOta >= 30000UL) {
            lastOta = millis();
            espNowTransport.configureLink(runtimeConfig.wifiChannel,
                                          runtimeConfig.txPowerQuarterDbm);
            const auto batteryBeforeOta = battery;
            sensor::checkOta(espNowTransport, runtimeConfig, adcReader, battery);
            if (battery.valid != batteryBeforeOta.valid ||
                battery.millivolts != batteryBeforeOta.millivolts) {
              lastBattery = millis();
            }
            if constexpr (SENSOR_LOW_BATTERY_PAUSE_MV > 0) {
              if (batteryProtectionRequired(battery)) {
                environmentalSensor.end();
                espNowTransport.end();
                sensor::popLowPowerWaitInhibit();
                reportBatteryAndSleep(battery);
              }
            }
          }
        }
        espNowTransport.end();
      }
    }
    delay(50);
  }
}

}  // namespace

void setup() {
#if defined(ESP_PLATFORM) && defined(CONFIG_PM_ENABLE)
  const esp_pm_config_t pmConfig{80, 40, false};
  (void)esp_pm_configure(&pmConfig);
#endif
  sensor::beginOtaBootGuard();
  sensor::beginDiagnosticLogging();
  initializeRtcState();

  if (!configStore.begin()) {
    powerController.begin();
    sensor::finishOtaBootGuard();
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
    const bool continuousLive = sensorStarted && energySavingMode &&
        lil::protocol::isLiveSensor(environmentalSensor.detectedType());
    const bool continuousGas = sensorStarted && energySavingMode &&
        environmentalSensor.detectedType() == lil::protocol::EnvironmentalSensorType::kCcs811;
    if (continuousLive) {
      // Live acquisition manages its own sampling windows and rail power.
      // Release the rail immediately so flash journal startup does not draw
      // sensor power and skip the discarded boot-time conversion.
      environment.valid = true;
      environment.sensorType = environmentalSensor.detectedType();
      environmentalSensor.end();
    } else if (continuousGas) {
      // CCS811 keeps its switched rail powered so the MOX heater can warm up
      // without restarting from a cold boot on every wake cycle.
      environment.valid = true;
      environment.sensorType = environmentalSensor.detectedType();
    } else {
      environment = reportDue
          ? environmentalSensor.readForReport(runtimeConfig.temperatureOffsetC)
          : environmentalSensor.read();
    }
    // A responding one-shot sensor (BME280, BMP280, or extended I2C sensor)
    // can still fail its conversion (bus fault, reset or stuck status). Only a
    // valid sample replenishes its retry budget. BSEC maintenance and SW2-only
    // sensors can legitimately produce no sample.
    const auto detected = environmentalSensor.detectedType();
    const bool oneShotReadFailed = sensorStarted && !environment.valid &&
        (detected == lil::protocol::EnvironmentalSensorType::kBme280 ||
         detected == lil::protocol::EnvironmentalSensorType::kBmp280 ||
         lil::protocol::hasExtendedPayload(detected));
    if (sensorStarted) {
      rtcState.lastSensorType = detected;
    }
    if (sensorStarted && !oneShotReadFailed) {
      rtcState.sensorRecoveryPending = false;
      rtcState.sensorStartAttemptsRemaining = kSensorStartAttempts;
    } else if (rtcState.sensorStartAttemptsRemaining > 0) {
      --rtcState.sensorStartAttemptsRemaining;
      rtcState.sensorRecoveryPending = rtcState.sensorStartAttemptsRemaining > 0;
      // Report failed initialization/conversion and each retry promptly,
      // including BME680 maintenance wakes that normally leave the radio off.
      reportDue = true;
    }
    if (!continuousLive && !continuousGas) environmentalSensor.end();
    // Preserve the original report timing when a long BME680 conversion
    // crosses the configured deadline. This rare boundary case deliberately
    // pays the full ADC settling time instead of delaying data by five minutes.
    if (!reportDue) {
      reportDue = lil::power::reportDue(
          rtcState.hasAttemptedReport, logicalNowMs(),
          rtcState.lastReportLogicalMs, runtimeConfig.sleepSeconds);
    }
    if (reportDue || continuousLive || continuousGas) {
      battery = preflightBattery.valid ? preflightBattery : batteryMeasurementStarted
                    ? adcReader.finishBatteryMeasurement(
                          runtimeConfig.batteryCalibrationFactor)
                    : adcReader.readBattery(
                          runtimeConfig.batteryCalibrationFactor);
    }
    if (continuousLive) runLiveMode(battery);
    if (continuousGas) runCcs811Mode(battery);
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
      lil::protocol::isBoschGasSensor(environment.sensorType);
  sensor::ExchangeResult exchange{};
  // BME680 still wakes internally every five minutes to maintain BSEC/IAQ,
  // but once provisioned it must transmit strictly at the configured report
  // interval. Calibration must not silently increase the radio cadence.
  if (reportDue && discoveryMode) {
    sensor::lowPowerSensorWaitUs((esp_random() % 121U) * 1000U);
  }
  bool measurementChanged = false;
  bool configAppliedBeforeOta = false;
  if (reportDue && espNowTransport.begin()) {
    auto packet = makeTelemetryPacket(
        environment, battery,
        discoveryMode ? lil::protocol::SensorOperatingMode::kDiscovery
                      : lil::protocol::SensorOperatingMode::kEnergySaving);
    WCH_RADIO_BENCHMARK_BEGIN();
    exchange = espNowTransport.exchange(packet, runtimeConfig, sensor::otaBootPending());
    // A successful unicast MAC acknowledgement proves that the station was on
    // the configured channel. If only the application response was lost, a
    // maximum-power channel scan cannot help and merely extends the wake.
    if (energySavingMode && !exchange.configReceived && !exchange.delivered) {
      auto recoveryPacket = packet;
      recoveryPacket.payload.operatingMode =
          lil::protocol::SensorOperatingMode::kChannelRecovery;
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
    WCH_RADIO_BENCHMARK_FINISH(true, packet, exchange);
    if (exchange.configReceived) {
      rtcState.lastStationRssi = exchange.stationRssi;
      // Confirm an OTA trial boot on proven station contact even if the station
      // deprovisions or factory-resets the sensor before checkOta() runs.
      sensor::confirmOtaBootAfterContact();
      // Persist station config / honor factory reset before starting OTA so
      // an interrupted update or reboot cannot lose updated settings.
      measurementChanged = applyStationConfig(exchange.config);
      configAppliedBeforeOta = true;
    }
    if (exchange.configReceived && exchange.config.provisioned) {
      // Use the confirmed station/channel for the pull-based OTA exchange.
      auto otaConfig = runtimeConfig;
      otaConfig.provisioned = true;
      otaConfig.stationKnown = true;
      if (macIsUsable(exchange.config.stationMac)) {
        memcpy(otaConfig.stationMac, exchange.config.stationMac, 6);
      }
      if (exchange.config.wifiChannel >= 1 && exchange.config.wifiChannel <= 13) {
        otaConfig.wifiChannel = exchange.config.wifiChannel;
      }
      espNowTransport.configureLink(otaConfig.wifiChannel,
                                    otaConfig.txPowerQuarterDbm);
      sensor::checkOta(espNowTransport, otaConfig, adcReader, battery);
      if constexpr (SENSOR_LOW_BATTERY_PAUSE_MV > 0) {
        if (batteryProtectionRequired(battery)) {
          espNowTransport.end();
          reportBatteryAndSleep(battery);
        }
      }
    }
    espNowTransport.end();
    if (exchange.delivered && !exchange.configReceived) {
      configStore.saveIfChanged(runtimeConfig);
    }
  }

  if (exchange.configReceived) {
    rtcState.lastStationRssi = exchange.stationRssi;
    if (!configAppliedBeforeOta) {
      measurementChanged = applyStationConfig(exchange.config);
    }
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
      lil::protocol::isBoschGasSensor(runtimeConfig.environmentalSensorType) ||
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
        lil::timing::usToMs(logicalNowMs() - rtcState.discoveryStartedMs),
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

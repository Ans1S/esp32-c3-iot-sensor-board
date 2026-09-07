#include <Arduino.h>

#include <esp_random.h>
#include <esp_timer.h>

#include "adc_reader.h"
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
// station yet.  A configured sensor must never keep waking every ten seconds:
// that would defeat the configured measurement interval and waste battery.
// Revision C shares the acquisition clock and tracks attempted reports.
constexpr uint32_t kRtcSignature = 0x52544343UL;  // "RTCC"
constexpr uint32_t kPairingMeasurementSeconds = 5UL * 60UL;
constexpr uint32_t kInitialSleepPhaseWindowMs = 1000UL;
constexpr uint8_t kRecoveryChannelsPerReport = 3;

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
    rtcState.acknowledgedResetRevision = response.revision;
    rtcState.initialSleepPhaseMs = static_cast<uint16_t>(
        esp_random() % (kInitialSleepPhaseWindowMs + 1UL));
    rtcState.initialSleepPhaseApplied = false;
    rtcState.nextRecoveryChannel = 1;
    rtcState.fullChannelRecoveryPending = false;
    configStore.factoryReset();
    environmentalSensor.clearIaqState();
    sensor::SleepController::deepSleep(1, powerController);
  }

  const bool iaqResetRequested =
      (response.flags & lil::protocol::kResetIaqCalibration) != 0;
  if (iaqResetRequested) {
    environmentalSensor.clearIaqState();
    runtimeConfig.bme680QuickStartComplete = true;
  }

  const bool sensorTypeValid =
      response.sensorType ==
          lil::protocol::EnvironmentalSensorType::kAutoDetect ||
      response.sensorType == lil::protocol::EnvironmentalSensorType::kBme280 ||
      response.sensorType == lil::protocol::EnvironmentalSensorType::kLsm6dsox ||
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
    const bool newProvisioned = response.provisioned != 0;
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
    runtimeConfig.revision = response.revision;
    runtimeConfig.sleepSeconds = response.sleepIntervalSeconds;
    runtimeConfig.wifiChannel = response.wifiChannel;
    runtimeConfig.txPowerQuarterDbm =
        constrain(response.txPowerQuarterDbm, 8, 84);
    runtimeConfig.environmentalSensorType = response.sensorType;
    runtimeConfig.temperatureOffsetC =
        constrain(response.temperatureOffsetC, -10.0F, 10.0F);
    runtimeConfig.batteryCalibrationFactor =
        constrain(response.batteryCalibrationFactor, 0.7F, 1.3F);
    if (newProvisioned) {
      memcpy(runtimeConfig.stationMac, response.stationMac,
             sizeof(runtimeConfig.stationMac));
      runtimeConfig.stationKnown = true;
    } else {
      memset(runtimeConfig.stationMac, 0, sizeof(runtimeConfig.stationMac));
      runtimeConfig.stationKnown = false;
    }
    runtimeConfig.provisioned = newProvisioned;
    // Compatibility field in the persisted V6 layout. ULP-only operation has
    // no separate quick-start phase.
    runtimeConfig.bme680QuickStartComplete = true;
    configStore.saveIfChanged(runtimeConfig);
    if (sensorTypeChanged || provisioningChanged) {
      environmentalSensor.clearIaqState();
    }
    if (rtcState.acknowledgedResetRevision == response.revision) {
      rtcState.acknowledgedResetRevision = 0;
    }
    return provisioningChanged;
  }
  return false;
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
                                lil::protocol::kBattery;
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
      environment.sensorType != runtimeConfig.environmentalSensorType) {
    packet.payload.flags |= lil::protocol::kSensorTypeMismatch;
  }
  lil::protocol::finalize(packet, lil::protocol::MessageType::kTelemetry,
                          rtcState.sequence);
  return packet;
}

// Motion acquisition continues while reports/OTA use the radio. The hardware
// FIFO bridges blocking exchanges; overflow is reported instead of hidden.
void runMotionMode(sensor::BatteryReading battery) {
  if (!battery.valid) battery = adcReader.readBattery(runtimeConfig.batteryCalibrationFactor);
  uint32_t lastReport = millis() - runtimeConfig.sleepSeconds * 1000UL + 25UL;
  uint32_t lastBattery = millis(), lastOta = millis() - 30000UL;
  uint32_t lastRecovery = millis();
  uint8_t readFailures = 0;
  for (;;) {
    environmentalSensor.pollMotion();
    const uint32_t now = millis();
    if (now - lastBattery >= 60000UL) {
      battery = adcReader.readBattery(runtimeConfig.batteryCalibrationFactor);
      lastBattery = now;
      if constexpr (SENSOR_LOW_BATTERY_PAUSE_MV > 0) {
        if (battery.valid && battery.millivolts < SENSOR_LOW_BATTERY_PAUSE_MV) {
          rtcState.batteryPaused = true;
          environmentalSensor.end();
          sensor::SleepController::deepSleep(3600, powerController);
        }
      }
    }
    if (now - lastReport < runtimeConfig.sleepSeconds * 1000UL) {
      delay(5);
      continue;
    }
    lastReport = now; // Start-to-start cadence; never replay an overdue burst.
    const auto reading = environmentalSensor.read();
    ++rtcState.sequence;
    auto packet = makeTelemetryPacket(reading, battery,
        lil::protocol::SensorOperatingMode::kContinuousMotion);
    if (espNowTransport.begin()) {
      auto exchange = espNowTransport.exchange(packet, runtimeConfig, sensor::otaBootPending());
      if (!exchange.delivered && !exchange.configReceived && now - lastRecovery >= 5000UL) {
        lastRecovery = now;
        exchange = espNowTransport.exchangeLpChannel(packet, runtimeConfig,
            takeNextRecoveryChannel(runtimeConfig.wifiChannel), true);
      }
      if (exchange.configReceived) {
        rtcState.lastStationRssi = exchange.stationRssi;
        applyStationConfig(exchange.config);
        if (!runtimeConfig.provisioned ||
            (runtimeConfig.environmentalSensorType != lil::protocol::EnvironmentalSensorType::kAutoDetect &&
             runtimeConfig.environmentalSensorType != lil::protocol::EnvironmentalSensorType::kLsm6dsox)) {
          environmentalSensor.end();
          esp_restart();
        }
        if (sensor::otaBootPending() || now - lastOta >= 30000UL) {
          lastOta = now;
          sensor::checkOta(espNowTransport, runtimeConfig, adcReader, battery.millivolts);
        }
      }
      espNowTransport.end();
    }
    sensor::finishOtaBootGuard();
    readFailures = reading.valid ? 0 : readFailures + 1;
    if (readFailures >= 3) {
      environmentalSensor.end();
      if (!environmentalSensor.begin(powerController, runtimeConfig.environmentalSensorType,
                                      runtimeConfig.temperatureOffsetC)) {
        sensor::SleepController::deepSleep(5, powerController);
      }
      readFailures = 0;
    }
    delay(1);
  }
}

}  // namespace

void setup() {
  sensor::beginOtaBootGuard();
  sensor::beginDiagnosticLogging();
  initializeRtcState();

  if (!configStore.begin()) {
    powerController.begin();
    sensor::SleepController::deepSleep(60, powerController);
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
    if (preflightBattery.valid) {
      const uint16_t threshold = SENSOR_LOW_BATTERY_PAUSE_MV +
          (rtcState.batteryPaused ? SENSOR_LOW_BATTERY_RESUME_MARGIN_MV : 0);
      rtcState.batteryPaused = preflightBattery.millivolts < threshold;
    }
    if (rtcState.batteryPaused) {
      sensor::SleepController::deepSleep(3600, powerController);
    }
  }
  sensor::EnvironmentalReading environment{};
  sensor::BatteryReading battery{};
  const bool energySavingMode =
      startupMode == OperatingMode::kEnergySaving;
  const bool discoveryMode = !energySavingMode;
  bool reportDue =
      sensor::otaBootPending() || discoveryMode || lil::power::reportDue(
          rtcState.hasAttemptedReport, logicalNowMs(),
          rtcState.lastReportLogicalMs, runtimeConfig.sleepSeconds);
  const bool pairingMeasurementDue =
      discoveryMode &&
      (!rtcState.hasPairingSnapshot ||
       logicalNowMs() - rtcState.pairingMeasurementMs >=
           uint64_t(kPairingMeasurementSeconds) * 1000);
  const bool measurementDue = !discoveryMode || pairingMeasurementDue;
  if (measurementDue) {
    // On PCB V4 the gated divider needs 100 ms to settle. Start it before the
    // environmental conversion so both waits overlap. BME680-only maintenance
    // wakes do not report, so they leave the divider completely off.
    bool batteryMeasurementStarted = false;
    if (reportDue && !preflightBattery.valid) {
      adcReader.startBatteryMeasurement();
      batteryMeasurementStarted = true;
    }
    const bool sensorStarted = environmentalSensor.begin(
        powerController, runtimeConfig.environmentalSensorType,
        runtimeConfig.temperatureOffsetC);
    if (sensorStarted) {
      environment = environmentalSensor.read();
    }
    const bool continuousMotion = sensorStarted && energySavingMode &&
        environment.sensorType == lil::protocol::EnvironmentalSensorType::kLsm6dsox;
    if (!continuousMotion) environmentalSensor.end();
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
    if (continuousMotion) runMotionMode(battery);
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

  bool provisioningChanged = false;
  if (exchange.configReceived) {
    rtcState.lastStationRssi = exchange.stationRssi;
    provisioningChanged = applyStationConfig(exchange.config);
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
  if (provisioningChanged) {
    // Apply both additions and removals promptly.  After removal this makes
    // the sensor visible again in the ten-second pairing window; after an
    // addition the next wake starts the selected environmental sensor.
    sleepSeconds = 1;
    rtcState.hasPairingSnapshot = false;
    rtcState.discoveryStartedMs = logicalNowMs();
    rtcState.pairingMeasurementMs = logicalNowMs();
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
  if (energySavingModeAfterExchange && !provisioningChanged) {
    sleepSeconds = lil::power::scheduledSleepSeconds(
        rtcState.hasAttemptedReport, logicalNowMs(),
        rtcState.lastReportLogicalMs + uint64_t(runtimeConfig.sleepSeconds) * 1000,
        sleepSeconds);
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

#include <Arduino.h>

#include <esp_random.h>
#include <esp_timer.h>

#include "adc_reader.h"
#include "environmental_sensor.h"
#include "espnow_transport.h"
#include "hardware_profile.h"
#include "lil_protocol.h"
#include "power_controller.h"
#include "sensor_config_store.h"
#include "sleep_controller.h"

namespace {
// Discovery is deliberately limited to sensors which have not been added to a
// station yet.  A configured sensor must never keep waking every ten seconds:
// that would defeat the configured measurement interval and waste battery.
// Revision 9 replaces the report counter, which counted only requested deep-
// sleep seconds, with a logical clock that also includes time spent awake.
// The former counter could therefore miss a configured BME680 deadline by a
// few seconds and postpone the transmission to the next five-minute wakeup.
// Revision B also retains a pending full-channel recovery across deep sleep.
// This protects the transition from the station setup AP to the channel of the
// home Wi-Fi network without imposing a permanent full scan on paired sensors.
constexpr uint32_t kRtcSignature = 0x52544342UL;  // "RTCB"
constexpr uint32_t kFastPairingWindowSeconds = 10UL * 60UL;
constexpr uint32_t kFastPairingSleepSeconds = 10UL;
constexpr uint32_t kSlowPairingSleepSeconds = 5UL * 60UL;
constexpr uint32_t kPairingMeasurementSeconds = 5UL * 60UL;
constexpr uint32_t kInitialSleepPhaseWindowMs = 1000UL;
constexpr uint8_t kRecoveryChannelsPerReport = 3;

struct RtcState {
  uint32_t signature;
  uint32_t bootCount;
  uint32_t sequence;
  uint32_t acknowledgedResetRevision;
  uint8_t consecutiveFailures;
  int8_t lastStationRssi;
  uint64_t logicalTimeMs;
  uint64_t lastReportLogicalMs;
  uint32_t unprovisionedSeconds;
  uint32_t startupDiscoverySeconds;
  uint32_t pairingMeasurementAgeSeconds;
  uint16_t initialSleepPhaseMs;
  uint8_t nextRecoveryChannel;
  bool initialSleepPhaseApplied;
  bool fullChannelRecoveryPending;
  sensor::EnvironmentalReading cachedEnvironment;
  sensor::BatteryReading cachedBattery;
  bool hasReported;
  bool hasPairingSnapshot;
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
  return rtcState.logicalTimeMs +
         static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
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
    rtcState.consecutiveFailures = 0;
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
      response.sensorType == lil::protocol::EnvironmentalSensorType::kBme680 ||
      response.sensorType ==
          lil::protocol::EnvironmentalSensorType::kDisabled;
  if (response.sleepIntervalSeconds >= 30 &&
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

}  // namespace

void setup() {
  Serial.begin(115200);
  initializeRtcState();

  if (!configStore.begin()) {
    powerController.begin();
    sensor::SleepController::deepSleep(60, powerController);
  }
  if (configStore.firmwareChanged()) {
    environmentalSensor.clearIaqState();
    Serial.println(
        "[SETUP] New firmware image: sensor assignment and IAQ startup state cleared");
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

  sensor::EnvironmentalReading environment{};
  sensor::BatteryReading battery{};
  const bool energySavingMode =
      startupMode == OperatingMode::kEnergySaving;
  const bool fastDiscoveryActive =
      !energySavingMode &&
      rtcState.unprovisionedSeconds < kFastPairingWindowSeconds;
  const bool discoveryMode = !energySavingMode;
  const bool pairingMeasurementDue =
      discoveryMode &&
      (!rtcState.hasPairingSnapshot ||
       rtcState.pairingMeasurementAgeSeconds >= kPairingMeasurementSeconds);
  const bool measurementDue = !discoveryMode || pairingMeasurementDue;
  if (measurementDue) {
    const bool sensorStarted = environmentalSensor.begin(
        powerController, runtimeConfig.environmentalSensorType,
        runtimeConfig.temperatureOffsetC);
    if (sensorStarted) {
      environment = environmentalSensor.read();
    }
    environmentalSensor.end();
    battery = adcReader.readBattery(runtimeConfig.batteryCalibrationFactor);
    if (discoveryMode) {
      rtcState.cachedEnvironment = environment;
      rtcState.cachedBattery = battery;
      rtcState.hasPairingSnapshot = true;
      rtcState.pairingMeasurementAgeSeconds = 0;
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
  const bool reportDue = discoveryMode || !rtcState.hasReported ||
                         logicalNowMs() - rtcState.lastReportLogicalMs >=
                             static_cast<uint64_t>(runtimeConfig.sleepSeconds) *
                                 1000ULL;
  if (reportDue && espNowTransport.begin()) {
    exchange = espNowTransport.exchange(packet, runtimeConfig);
    if (energySavingMode && !exchange.configReceived) {
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
      }
    }
    espNowTransport.end();
  }

  bool provisioningChanged = false;
  if (exchange.configReceived) {
    rtcState.consecutiveFailures = 0;
    rtcState.lastStationRssi = exchange.stationRssi;
    provisioningChanged = applyStationConfig(exchange.config);
    rtcState.lastReportLogicalMs = logicalNowMs();
    rtcState.hasReported = true;
  } else if (reportDue) {
    rtcState.consecutiveFailures = rtcState.consecutiveFailures < 10
                                       ? rtcState.consecutiveFailures + 1
                                       : 10;
    if (energySavingMode) {
      // A failed exchange is still a radio attempt. Start a fresh configured
      // interval instead of retrying at the BME680's internal 5-minute
      // measurement cadence.
      rtcState.lastReportLogicalMs = logicalNowMs();
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
    rtcState.unprovisionedSeconds = 0;
    rtcState.pairingMeasurementAgeSeconds = 0;
  } else if (operatingMode(runtimeConfig) == OperatingMode::kDiscovery) {
    sleepSeconds = fastDiscoveryActive ? kFastPairingSleepSeconds
                                       : kSlowPairingSleepSeconds;
  }
  const bool energySavingModeAfterExchange =
      operatingMode(runtimeConfig) == OperatingMode::kEnergySaving;
  if (!energySavingModeAfterExchange) {
    rtcState.unprovisionedSeconds =
        UINT32_MAX - rtcState.unprovisionedSeconds < sleepSeconds
            ? UINT32_MAX
            : rtcState.unprovisionedSeconds + sleepSeconds;
  } else {
    rtcState.unprovisionedSeconds = 0;
  }
  if (discoveryMode) {
    rtcState.pairingMeasurementAgeSeconds =
        UINT32_MAX - rtcState.pairingMeasurementAgeSeconds < sleepSeconds
            ? UINT32_MAX
            : rtcState.pairingMeasurementAgeSeconds + sleepSeconds;
  } else {
    rtcState.pairingMeasurementAgeSeconds = 0;
    rtcState.hasPairingSnapshot = false;
  }
  if (energySavingModeAfterExchange && !provisioningChanged) {
    environmentalSensor.prepareForDeepSleep(sleepSeconds,
                                             environment.sensorType);
  }
  const uint32_t sleepPhaseMs = rtcState.initialSleepPhaseApplied
                                    ? 0
                                    : rtcState.initialSleepPhaseMs;
  rtcState.initialSleepPhaseApplied = true;
  rtcState.logicalTimeMs =
      logicalNowMs() + static_cast<uint64_t>(sleepSeconds) * 1000ULL +
      sleepPhaseMs;
  sensor::SleepController::deepSleep(sleepSeconds, powerController,
                                     sleepPhaseMs);
}

void loop() {}

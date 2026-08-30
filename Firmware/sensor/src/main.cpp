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
constexpr uint32_t kRtcSignature = 0x52544339UL;  // "RTC9"
constexpr uint32_t kFastPairingWindowSeconds = 10UL * 60UL;
constexpr uint32_t kFastPairingSleepSeconds = 10UL;
constexpr uint32_t kSlowPairingSleepSeconds = 5UL * 60UL;
constexpr uint32_t kPairingMeasurementSeconds = 5UL * 60UL;

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

bool macIsUsable(const uint8_t mac[6]) {
  uint8_t combined = 0;
  for (size_t i = 0; i < 6; ++i) combined |= mac[i];
  return combined != 0;
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
  }
  ++rtcState.bootCount;
  ++rtcState.sequence;
}

bool applyStationConfig(const lil::protocol::ConfigResponsePayload& response) {
  if ((response.flags & lil::protocol::kFactoryReset) != 0) {
    rtcState.acknowledgedResetRevision = response.revision;
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
    memcpy(runtimeConfig.stationMac, response.stationMac,
           sizeof(runtimeConfig.stationMac));
    runtimeConfig.stationKnown = true;
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
    const sensor::BatteryReading& battery, bool discoveryMode) {
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
  packet.payload.lastStationRssi = rtcState.lastStationRssi;
  packet.payload.sensorType = environment.sensorType;
  packet.payload.iaqAccuracy = environment.iaqAccuracy;
  packet.payload.iaqCalibrationPhase = environment.iaqCalibrationPhase;
  packet.payload.iaqCalibrationElapsedMinutes =
      environment.iaqCalibrationElapsedMinutes;
  packet.payload.iaqCalibrationRemainingMinutes =
      environment.iaqCalibrationRemainingMinutes;
  if (discoveryMode) {
    packet.payload.flags |= lil::protocol::kDiscoveryBeacon;
  }
  if (environment.bme680RawFallback && environment.valid) {
    packet.payload.flags |= lil::protocol::kBme680RawFallback;
  }
  if ((environment.capabilities & lil::protocol::kIaq) != 0 &&
      environment.iaqAccuracy < 2) {
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
        "[SETUP] Neues Firmware-Image: Sensorzuordnung und IAQ-Startzustand geloescht");
  }
  runtimeConfig = configStore.load();

  powerController.begin();
  adcReader.begin(powerController);

  sensor::EnvironmentalReading environment{};
  sensor::BatteryReading battery{};
  const bool fastDiscoveryActive =
      !runtimeConfig.provisioned &&
      rtcState.unprovisionedSeconds < kFastPairingWindowSeconds;
  const bool discoveryMode = !runtimeConfig.provisioned;
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
  auto packet = makeTelemetryPacket(environment, battery, discoveryMode);

  sensor::ExchangeResult exchange{};
  // BME680 still wakes internally every five minutes to maintain BSEC/IAQ,
  // but once provisioned it must transmit strictly at the configured report
  // interval. Calibration must not silently increase the radio cadence.
  const bool reportDue = discoveryMode || !runtimeConfig.stationKnown ||
                         !rtcState.hasReported ||
                         logicalNowMs() - rtcState.lastReportLogicalMs >=
                             static_cast<uint64_t>(runtimeConfig.sleepSeconds) *
                                 1000ULL;
  if (reportDue && espNowTransport.begin()) {
    exchange = espNowTransport.exchange(packet, runtimeConfig,
                                        fastDiscoveryActive);
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
    if (runtimeConfig.provisioned) {
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
  } else if (!runtimeConfig.provisioned) {
    sleepSeconds = fastDiscoveryActive ? kFastPairingSleepSeconds
                                       : kSlowPairingSleepSeconds;
  }
  if (!runtimeConfig.provisioned) {
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
  if (runtimeConfig.provisioned && !provisioningChanged) {
    environmentalSensor.prepareForDeepSleep(sleepSeconds,
                                             environment.sensorType);
  }
  rtcState.logicalTimeMs =
      logicalNowMs() + static_cast<uint64_t>(sleepSeconds) * 1000ULL;
  sensor::SleepController::deepSleep(sleepSeconds, powerController);
}

void loop() {}

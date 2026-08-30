#pragma once

#include <Preferences.h>

#include "lil_protocol.h"

namespace sensor {

constexpr uint32_t kSensorConfigMagic = 0x534E3230UL;  // "SN20"
constexpr uint16_t kSensorConfigVersion = 6;

struct SensorRuntimeConfig {
  uint32_t magic = kSensorConfigMagic;
  uint16_t version = kSensorConfigVersion;
  uint32_t revision = 0;
  uint32_t sleepSeconds = 600;
  uint8_t stationMac[6]{};
  uint8_t wifiChannel = 1;
  int8_t txPowerQuarterDbm = 52;  // 13 dBm
  bool stationKnown = false;
  // Authoritative local setup state. It is persisted in NVS and is cleared
  // only by a station removal/factory-reset response or erased flash.
  bool provisioned = false;
  // Retained for binary compatibility with persisted V6 configurations. The
  // BME680 now operates in ULP for its complete service life.
  bool bme680QuickStartComplete = true;
  lil::protocol::EnvironmentalSensorType environmentalSensorType =
      lil::protocol::EnvironmentalSensorType::kAutoDetect;
  float temperatureOffsetC = 0.466F;
  float batteryCalibrationFactor = 1.0F;
};

class SensorConfigStore {
 public:
  bool begin();
  bool firmwareChanged() const;
  SensorRuntimeConfig load();
  bool saveIfChanged(const SensorRuntimeConfig& config);
  void factoryReset();

 private:
  Preferences preferences_;
  SensorRuntimeConfig lastStored_{};
  bool hasStoredCopy_ = false;
  bool firmwareChanged_ = false;
};

}  // namespace sensor

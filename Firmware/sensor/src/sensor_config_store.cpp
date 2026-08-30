#include "sensor_config_store.h"

#include <esp_app_desc.h>
#include <string.h>

namespace sensor {

namespace {
constexpr char kNamespace[] = "lil_sensor";
constexpr char kConfigKey[] = "config";
constexpr char kFirmwareKey[] = "fw_sha";
constexpr uint32_t kMinSleepSeconds = 30;
constexpr uint32_t kMaxSleepSeconds = 86400;

struct LegacySensorRuntimeConfigV1 {
  uint32_t magic = kSensorConfigMagic;
  uint16_t version = 1;
  uint32_t revision = 0;
  uint32_t sleepSeconds = 600;
  uint8_t stationMac[6]{};
  uint8_t wifiChannel = 1;
  int8_t txPowerQuarterDbm = 52;
  bool stationKnown = false;
};

struct LegacySensorRuntimeConfigV2 {
  uint32_t magic = kSensorConfigMagic;
  uint16_t version = 2;
  uint32_t revision = 0;
  uint32_t sleepSeconds = 600;
  uint8_t stationMac[6]{};
  uint8_t wifiChannel = 1;
  int8_t txPowerQuarterDbm = 52;
  bool stationKnown = false;
  lil::protocol::EnvironmentalSensorType environmentalSensorType =
      lil::protocol::EnvironmentalSensorType::kAutoDetect;
  float temperatureOffsetC = 0.466F;
};

struct LegacySensorRuntimeConfigV3 {
  uint32_t magic = kSensorConfigMagic;
  uint16_t version = 3;
  uint32_t revision = 0;
  uint32_t sleepSeconds = 600;
  uint8_t stationMac[6]{};
  uint8_t wifiChannel = 1;
  int8_t txPowerQuarterDbm = 52;
  bool stationKnown = false;
  lil::protocol::EnvironmentalSensorType environmentalSensorType =
      lil::protocol::EnvironmentalSensorType::kAutoDetect;
  float temperatureOffsetC = 0.466F;
  float batteryCalibrationFactor = 1.0F;
};

struct LegacySensorRuntimeConfigV4 {
  uint32_t magic = kSensorConfigMagic;
  uint16_t version = 4;
  uint32_t revision = 0;
  uint32_t sleepSeconds = 600;
  uint8_t stationMac[6]{};
  uint8_t wifiChannel = 1;
  int8_t txPowerQuarterDbm = 52;
  bool stationKnown = false;
  bool provisioned = false;
  lil::protocol::EnvironmentalSensorType environmentalSensorType =
      lil::protocol::EnvironmentalSensorType::kAutoDetect;
  float temperatureOffsetC = 0.466F;
  float batteryCalibrationFactor = 1.0F;
};

struct LegacySensorRuntimeConfigV5 {
  uint32_t magic = kSensorConfigMagic;
  uint16_t version = 5;
  uint32_t revision = 0;
  uint32_t sleepSeconds = 600;
  uint8_t stationMac[6]{};
  uint8_t wifiChannel = 1;
  int8_t txPowerQuarterDbm = 52;
  bool stationKnown = false;
  bool provisioned = false;
  bool bme680QuickStartComplete = false;
  lil::protocol::EnvironmentalSensorType environmentalSensorType =
      lil::protocol::EnvironmentalSensorType::kAutoDetect;
  float temperatureOffsetC = 0.466F;
  float batteryCalibrationFactor = 1.0F;
};

static_assert(sizeof(LegacySensorRuntimeConfigV5) ==
                  sizeof(SensorRuntimeConfig),
              "V5 migration must preserve the persisted binary layout");

template <typename Legacy>
void migrateRuntimeConfig(const Legacy& legacy, SensorRuntimeConfig& config) {
  config = SensorRuntimeConfig{};
  config.revision = legacy.revision;
  config.sleepSeconds = legacy.sleepSeconds;
  memcpy(config.stationMac, legacy.stationMac, sizeof(config.stationMac));
  config.wifiChannel = legacy.wifiChannel;
  config.txPowerQuarterDbm = legacy.txPowerQuarterDbm;
  config.stationKnown = legacy.stationKnown;
}

bool validSensorType(lil::protocol::EnvironmentalSensorType type) {
  return type == lil::protocol::EnvironmentalSensorType::kAutoDetect ||
         type == lil::protocol::EnvironmentalSensorType::kBme280 ||
         type == lil::protocol::EnvironmentalSensorType::kBme680 ||
         type == lil::protocol::EnvironmentalSensorType::kDisabled;
}
}

bool SensorConfigStore::begin() {
  if (!preferences_.begin(kNamespace, false)) {
    return false;
  }
  const esp_app_desc_t* description = esp_app_get_description();
  if (description == nullptr) {
    return false;
  }
  uint8_t storedSha[sizeof(description->app_elf_sha256)]{};
  const bool sameFirmware =
      preferences_.getBytesLength(kFirmwareKey) == sizeof(storedSha) &&
      preferences_.getBytes(kFirmwareKey, storedSha, sizeof(storedSha)) ==
          sizeof(storedSha) &&
      memcmp(storedSha, description->app_elf_sha256, sizeof(storedSha)) == 0;
  firmwareChanged_ = !sameFirmware;
  if (firmwareChanged_) {
    // Serial flashing normally leaves NVS intact. Treat a different firmware
    // image as a freshly flashed sensor while retaining state across ordinary
    // resets, deep sleep and complete power loss with the same image.
    if (!preferences_.clear() ||
        preferences_.putBytes(kFirmwareKey, description->app_elf_sha256,
                              sizeof(description->app_elf_sha256)) !=
            sizeof(description->app_elf_sha256)) {
      return false;
    }
  }
  return true;
}

bool SensorConfigStore::firmwareChanged() const {
  return firmwareChanged_;
}

SensorRuntimeConfig SensorConfigStore::load() {
  SensorRuntimeConfig config{};
  bool migrated = false;
  const size_t stored = preferences_.getBytesLength(kConfigKey);
  if (stored == sizeof(config)) {
    preferences_.getBytes(kConfigKey, &config, sizeof(config));
    if (config.version == 5 &&
        stored == sizeof(LegacySensorRuntimeConfigV5)) {
      LegacySensorRuntimeConfigV5 legacy{};
      preferences_.getBytes(kConfigKey, &legacy, sizeof(legacy));
      migrateRuntimeConfig(legacy, config);
      config.provisioned = legacy.provisioned;
      config.environmentalSensorType = legacy.environmentalSensorType;
      config.temperatureOffsetC = legacy.temperatureOffsetC;
      config.batteryCalibrationFactor = legacy.batteryCalibrationFactor;
      // The field is retained only to preserve the binary layout. ULP-only
      // operation no longer has a separate commissioning cycle.
      config.bme680QuickStartComplete = true;
      migrated = true;
    } else if (config.version == 4 &&
               stored == sizeof(LegacySensorRuntimeConfigV4)) {
      LegacySensorRuntimeConfigV4 legacy{};
      preferences_.getBytes(kConfigKey, &legacy, sizeof(legacy));
      migrateRuntimeConfig(legacy, config);
      config.provisioned = legacy.provisioned;
      config.environmentalSensorType = legacy.environmentalSensorType;
      config.temperatureOffsetC = legacy.temperatureOffsetC;
      config.batteryCalibrationFactor = legacy.batteryCalibrationFactor;
      migrated = true;
    } else if (config.version == 3 &&
               stored == sizeof(LegacySensorRuntimeConfigV3)) {
      LegacySensorRuntimeConfigV3 legacy{};
      preferences_.getBytes(kConfigKey, &legacy, sizeof(legacy));
      migrateRuntimeConfig(legacy, config);
      config.provisioned = legacy.stationKnown;
      config.environmentalSensorType = legacy.environmentalSensorType;
      config.temperatureOffsetC = legacy.temperatureOffsetC;
      config.batteryCalibrationFactor = legacy.batteryCalibrationFactor;
      migrated = true;
    }
  } else if (stored == sizeof(LegacySensorRuntimeConfigV4)) {
    LegacySensorRuntimeConfigV4 legacy{};
    preferences_.getBytes(kConfigKey, &legacy, sizeof(legacy));
    migrateRuntimeConfig(legacy, config);
    config.provisioned = legacy.provisioned;
    config.environmentalSensorType = legacy.environmentalSensorType;
    config.temperatureOffsetC = legacy.temperatureOffsetC;
    config.batteryCalibrationFactor = legacy.batteryCalibrationFactor;
    migrated = true;
  } else if (stored == sizeof(LegacySensorRuntimeConfigV3)) {
    LegacySensorRuntimeConfigV3 legacy{};
    preferences_.getBytes(kConfigKey, &legacy, sizeof(legacy));
    migrateRuntimeConfig(legacy, config);
    config.provisioned = legacy.stationKnown;
    config.environmentalSensorType = legacy.environmentalSensorType;
    config.temperatureOffsetC = legacy.temperatureOffsetC;
    config.batteryCalibrationFactor = legacy.batteryCalibrationFactor;
    migrated = true;
  } else if (stored == sizeof(LegacySensorRuntimeConfigV2)) {
    LegacySensorRuntimeConfigV2 legacy{};
    preferences_.getBytes(kConfigKey, &legacy, sizeof(legacy));
    migrateRuntimeConfig(legacy, config);
    config.provisioned = legacy.stationKnown;
    config.environmentalSensorType = legacy.environmentalSensorType;
    config.temperatureOffsetC = legacy.temperatureOffsetC;
    migrated = true;
  } else if (stored == sizeof(LegacySensorRuntimeConfigV1)) {
    LegacySensorRuntimeConfigV1 legacy{};
    preferences_.getBytes(kConfigKey, &legacy, sizeof(legacy));
    migrateRuntimeConfig(legacy, config);
    config.provisioned = legacy.stationKnown;
    migrated = true;
  }
  if (config.magic != kSensorConfigMagic ||
      config.version != kSensorConfigVersion ||
      config.sleepSeconds < kMinSleepSeconds ||
      config.sleepSeconds > kMaxSleepSeconds || config.wifiChannel < 1 ||
      config.wifiChannel > 13 ||
      !validSensorType(config.environmentalSensorType) ||
      !isfinite(config.temperatureOffsetC) ||
      config.temperatureOffsetC < -10.0F ||
      config.temperatureOffsetC > 10.0F ||
      !isfinite(config.batteryCalibrationFactor) ||
      config.batteryCalibrationFactor < 0.7F ||
      config.batteryCalibrationFactor > 1.3F) {
    config = SensorRuntimeConfig{};
  }
  lastStored_ = config;
  hasStoredCopy_ = true;
  if (migrated) {
    preferences_.putBytes(kConfigKey, &config, sizeof(config));
  }
  return config;
}

bool SensorConfigStore::saveIfChanged(const SensorRuntimeConfig& config) {
  if (hasStoredCopy_ && memcmp(&lastStored_, &config, sizeof(config)) == 0) {
    return true;
  }
  if (preferences_.putBytes(kConfigKey, &config, sizeof(config)) !=
      sizeof(config)) {
    return false;
  }
  lastStored_ = config;
  hasStoredCopy_ = true;
  return true;
}

void SensorConfigStore::factoryReset() {
  preferences_.clear();
  hasStoredCopy_ = false;
}

}  // namespace sensor

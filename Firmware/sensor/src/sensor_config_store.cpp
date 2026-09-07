#include "sensor_config_store.h"

#include <esp_app_desc.h>
#include <esp_attr.h>
#include <string.h>

namespace sensor {

namespace {
constexpr char kNamespace[] = "lil_sensor";
constexpr char kConfigKey[] = "config";
constexpr char kFirmwareKey[] = "fw_sha";
constexpr uint32_t kMinSleepSeconds = 1;
constexpr uint32_t kMaxSleepSeconds = 86400;
constexpr uint32_t kRtcConfigSignature = 0x43464737UL;  // "CFG7"

struct RtcConfigCache {
  uint32_t signature = 0;
  uint8_t firmwareSha[32]{};
  SensorRuntimeConfig config{};
};

RTC_DATA_ATTR RtcConfigCache rtcConfigCache{};

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
      type == lil::protocol::EnvironmentalSensorType::kLsm6dsox ||
         type == lil::protocol::EnvironmentalSensorType::kBme680 ||
         type == lil::protocol::EnvironmentalSensorType::kDisabled;
}

bool validRuntimeConfig(const SensorRuntimeConfig& config) {
  return config.magic == kSensorConfigMagic &&
         config.version == kSensorConfigVersion &&
         config.sleepSeconds >= kMinSleepSeconds &&
         config.sleepSeconds <= kMaxSleepSeconds && config.wifiChannel >= 1 &&
         config.wifiChannel <= 13 &&
         validSensorType(config.environmentalSensorType) &&
         isfinite(config.temperatureOffsetC) &&
         config.temperatureOffsetC >= -10.0F &&
         config.temperatureOffsetC <= 10.0F &&
         isfinite(config.batteryCalibrationFactor) &&
         config.batteryCalibrationFactor >= 0.7F &&
         config.batteryCalibrationFactor <= 1.3F;
}

void updateRtcConfigCache(const uint8_t firmwareSha[32],
                          const SensorRuntimeConfig& config) {
  rtcConfigCache.signature = 0;
  memcpy(rtcConfigCache.firmwareSha, firmwareSha,
         sizeof(rtcConfigCache.firmwareSha));
  rtcConfigCache.config = config;
  rtcConfigCache.signature = kRtcConfigSignature;
}
}

bool SensorConfigStore::ensurePreferencesOpen() {
  if (preferencesOpen_) {
    return true;
  }
  preferencesOpen_ = preferences_.begin(kNamespace, false);
  return preferencesOpen_;
}

bool SensorConfigStore::begin() {
  const esp_app_desc_t* description = esp_app_get_description();
  if (description == nullptr) {
    return false;
  }
  static_assert(sizeof(currentFirmwareSha_) ==
                    sizeof(description->app_elf_sha256),
                "Firmware SHA size changed");
  memcpy(currentFirmwareSha_, description->app_elf_sha256,
         sizeof(currentFirmwareSha_));

  if (rtcConfigCache.signature == kRtcConfigSignature &&
      memcmp(rtcConfigCache.firmwareSha, currentFirmwareSha_,
             sizeof(currentFirmwareSha_)) == 0 &&
      validRuntimeConfig(rtcConfigCache.config)) {
    lastStored_ = rtcConfigCache.config;
    hasStoredCopy_ = true;
    rtcConfigAvailable_ = true;
    firmwareChanged_ = false;
    return true;
  }

  if (!ensurePreferencesOpen()) {
    return false;
  }
  uint8_t storedSha[sizeof(currentFirmwareSha_)]{};
  const bool sameFirmware =
      preferences_.getBytesLength(kFirmwareKey) == sizeof(storedSha) &&
      preferences_.getBytes(kFirmwareKey, storedSha, sizeof(storedSha)) ==
          sizeof(storedSha) &&
      memcmp(storedSha, currentFirmwareSha_, sizeof(storedSha)) == 0;
  firmwareChanged_ = !sameFirmware;
  if (firmwareChanged_) {
    // Firmware upgrades invalidate RTC data, not the persisted pairing. Keep
    // the previous schema readable so a bootloader rollback retains settings.
    if (preferences_.putBytes(kFirmwareKey, currentFirmwareSha_,
                              sizeof(currentFirmwareSha_)) !=
            sizeof(currentFirmwareSha_)) {
      return false;
    }
  }
  rtcConfigCache.signature = 0;
  return true;
}

bool SensorConfigStore::firmwareChanged() const {
  return firmwareChanged_;
}

SensorRuntimeConfig SensorConfigStore::load() {
  if (rtcConfigAvailable_) {
    return lastStored_;
  }

  SensorRuntimeConfig config{};
  bool migrated = false;
  if (!ensurePreferencesOpen()) {
    return config;
  }
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
  if (!validRuntimeConfig(config)) {
    config = SensorRuntimeConfig{};
  }
  lastStored_ = config;
  hasStoredCopy_ = true;
  if (migrated) {
    preferences_.putBytes(kConfigKey, &config, sizeof(config));
  }
  updateRtcConfigCache(currentFirmwareSha_, config);
  rtcConfigAvailable_ = true;
  return config;
}

bool SensorConfigStore::saveIfChanged(const SensorRuntimeConfig& config) {
  if (hasStoredCopy_ && memcmp(&lastStored_, &config, sizeof(config)) == 0) {
    return true;
  }
  if (!ensurePreferencesOpen() ||
      preferences_.putBytes(kConfigKey, &config, sizeof(config)) !=
      sizeof(config)) {
    return false;
  }
  lastStored_ = config;
  hasStoredCopy_ = true;
  updateRtcConfigCache(currentFirmwareSha_, config);
  rtcConfigAvailable_ = true;
  return true;
}

void SensorConfigStore::factoryReset() {
  if (ensurePreferencesOpen()) {
    preferences_.clear();
  }
  rtcConfigCache.signature = 0;
  hasStoredCopy_ = false;
  rtcConfigAvailable_ = false;
}

}  // namespace sensor

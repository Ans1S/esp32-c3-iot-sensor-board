#include "config_store.h"

#include <LittleFS.h>
#include <WiFi.h>
#include <esp_partition.h>

namespace station {

namespace {
constexpr char kNamespace[] = "lil_station";
constexpr char kStationKey[] = "station";

bool filesystemPartitionErased() {
  const esp_partition_t* partition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "spiffs");
  if (!partition || !partition->size) return false;
  uint8_t bytes[1024];
  for (size_t offset = 0; offset < partition->size; offset += sizeof(bytes)) {
    const size_t count = min(sizeof(bytes), size_t(partition->size) - offset);
    if (esp_partition_read(partition, offset, bytes, count) != ESP_OK) return false;
    for (size_t i = 0; i < count; ++i) if (bytes[i] != 0xFF) return false;
    if (offset && offset % (64U * 1024U) == 0) delay(1);
  }
  return true;
}

template <size_t N, size_t M>
void copyStoredText(char (&destination)[N], const char (&source)[M]) {
  const size_t count = min(N - 1, M);
  memcpy(destination, source, count);
  destination[count] = '\0';
}

struct LegacyStationConfigV3 {
  uint32_t magic = kConfigMagic;
  uint16_t version = 3;
  char hostname[33] = "w-charger";
  char wifiSsid[33] = {};
  char wifiPassword[65] = {};
  char accessPointPassword[65] = {};
  char adminPassword[65] = {};
  char thingSpeakUserApiKey[41] = {};
  uint32_t defaultSleepSeconds = 600;
  uint8_t fallbackWifiChannel = 1;
  bool accessPointAlwaysOn = true;
};

struct LegacyStationConfigV4 {
  uint32_t magic = kConfigMagic;
  uint16_t version = 4;
  char hostname[33] = "w-charger";
  char wifiSsid[33] = {};
  char wifiPassword[65] = {};
  char accessPointPassword[65] = {};
  char adminPassword[65] = {};
  char thingSpeakUserApiKey[41] = {};
  uint32_t thingSpeakDefaultChannelId = 0;
  char thingSpeakReadApiKey[33] = {};
  char thingSpeakWriteApiKey[33] = {};
  uint32_t defaultSleepSeconds = 600;
  uint8_t fallbackWifiChannel = 1;
  bool accessPointAlwaysOn = true;
};

struct LegacySensorConfig {
  bool occupied = false;
  uint8_t mac[6] = {};
  char name[25] = {};
  uint32_t revision = 1;
  uint32_t sleepSeconds = 600;
  uint32_t thingSpeakChannelId = 0;
  char thingSpeakWriteKey[33] = {};
  uint16_t pendingFlags = 0;
  bool cloudUploadEnabled = false;
};

struct LegacyThingSpeakFieldMapping {
  uint8_t temperature = 1;
  uint8_t humidity = 2;
  uint8_t pressure = 3;
  uint8_t iaq = 0;
  uint8_t battery = 5;
};

struct LegacySensorConfigV2 {
  bool occupied = false;
  uint8_t mac[6] = {};
  char name[25] = {};
  uint32_t revision = 1;
  uint32_t sleepSeconds = 600;
  uint32_t thingSpeakChannelId = 0;
  char thingSpeakWriteKey[33] = {};
  uint16_t pendingFlags = 0;
  bool cloudUploadEnabled = false;
  LegacyThingSpeakFieldMapping thingSpeakFields{};
};

struct LegacySensorConfigV3 {
  bool occupied = false;
  uint8_t mac[6] = {};
  char name[25] = {};
  uint32_t revision = 1;
  uint32_t sleepSeconds = 600;
  uint32_t thingSpeakChannelId = 0;
  char thingSpeakWriteKey[33] = {};
  uint16_t pendingFlags = 0;
  bool cloudUploadEnabled = false;
  LegacyThingSpeakFieldMapping thingSpeakFields{};
  bool provisioned = false;
  uint8_t thingSpeakProfileSlot = 0xFF;
  uint32_t storageVersion = 3;
};

struct LegacySensorConfigV4 {
  bool occupied = false;
  uint8_t mac[6] = {};
  char name[25] = {};
  uint32_t revision = 1;
  uint32_t sleepSeconds = 600;
  uint32_t thingSpeakChannelId = 0;
  char thingSpeakWriteKey[33] = {};
  uint16_t pendingFlags = 0;
  bool cloudUploadEnabled = false;
  ThingSpeakFieldMapping thingSpeakFields{};
  bool provisioned = false;
  uint8_t thingSpeakProfileSlot = 0xFF;
  lil::protocol::EnvironmentalSensorType environmentalSensorType =
      lil::protocol::EnvironmentalSensorType::kAutoDetect;
  float temperatureOffsetC = 0.466F;
  uint32_t storageVersion = 4;
};

struct LegacySensorConfigV5 {
  bool occupied = false;
  uint8_t mac[6] = {};
  char name[25] = {};
  uint32_t revision = 1;
  uint32_t sleepSeconds = 600;
  uint32_t thingSpeakChannelId = 0;
  char thingSpeakWriteKey[33] = {};
  uint16_t pendingFlags = 0;
  bool cloudUploadEnabled = false;
  ThingSpeakFieldMapping thingSpeakFields{};
  bool provisioned = false;
  uint8_t thingSpeakProfileSlot = 0xFF;
  lil::protocol::EnvironmentalSensorType environmentalSensorType =
      lil::protocol::EnvironmentalSensorType::kAutoDetect;
  float temperatureOffsetC = 0.466F;
  float batteryCalibrationFactor = 1.0F;
  uint32_t storageVersion = 5;
};

void copyLegacyFields(const LegacyThingSpeakFieldMapping& source,
                      ThingSpeakFieldMapping& destination) {
  destination.temperature = source.temperature;
  destination.humidity = source.humidity;
  destination.pressure = source.pressure;
  destination.iaq = source.iaq;
  destination.battery = source.battery;
  destination.gasResistance = 0;
}

template <typename Legacy>
void migrateSensorBase(const Legacy& legacy, SensorConfig& config) {
  config = SensorConfig{};
  config.occupied = legacy.occupied;
  memcpy(config.mac, legacy.mac, sizeof(config.mac));
  copyStoredText(config.name, legacy.name);
  config.revision = legacy.revision;
  config.sleepSeconds = legacy.sleepSeconds;
  config.thingSpeakChannelId = legacy.thingSpeakChannelId;
  copyStoredText(config.thingSpeakWriteKey, legacy.thingSpeakWriteKey);
  config.pendingFlags = legacy.pendingFlags;
}

template <typename Legacy>
void migrateStationBase(const Legacy& legacy, StationConfig& config) {
  config = StationConfig{};
  config.magic = legacy.magic;
  config.version = kConfigVersion;
  strlcpy(config.hostname, "w-charger", sizeof(config.hostname));
  copyStoredText(config.wifiSsid, legacy.wifiSsid);
  copyStoredText(config.wifiPassword, legacy.wifiPassword);
  copyStoredText(config.accessPointPassword, legacy.accessPointPassword);
  config.adminPassword[0] = '\0';
  copyStoredText(config.thingSpeakUserApiKey, legacy.thingSpeakUserApiKey);
  config.defaultSleepSeconds = legacy.defaultSleepSeconds;
  config.fallbackWifiChannel = legacy.fallbackWifiChannel;
  config.setupPortalRequired = true;
}

String sensorKey(size_t index) {
  return "node" + String(index);
}

String historyKey(size_t index) {
  return "/history-" + String(index) + ".bin";
}

String latestTelemetryKey(size_t index) {
  return "/latest-" + String(index) + ".bin";
}

bool validSensorType(lil::protocol::EnvironmentalSensorType type) {
  return type == lil::protocol::EnvironmentalSensorType::kAutoDetect ||
         type == lil::protocol::EnvironmentalSensorType::kBme280 ||
      type == lil::protocol::EnvironmentalSensorType::kLsm6dsox ||
      type == lil::protocol::EnvironmentalSensorType::kTmp117 ||
      type == lil::protocol::EnvironmentalSensorType::kMax30102 ||
         type == lil::protocol::EnvironmentalSensorType::kBme680 ||
         type == lil::protocol::EnvironmentalSensorType::kDisabled;
}

bool validAdminPasswordHash(const char* hash) {
  const size_t length = strnlen(hash, 65);
  if (length == 0) {
    return true;
  }
  if (length != 64) {
    return false;
  }
  for (size_t i = 0; i < length; ++i) {
    const char value = hash[i];
    if (!((value >= '0' && value <= '9') ||
          (value >= 'a' && value <= 'f') ||
          (value >= 'A' && value <= 'F'))) {
      return false;
    }
  }
  return true;
}
}  // namespace

bool ConfigStore::begin() {
  if (!preferences_.begin(kNamespace, false)) return false;
  storageAvailable_ = LittleFS.begin(false);
  // A mount error must never erase existing recordings. Initialize storage
  // automatically only when every byte belongs to an erased new partition.
  if (!storageAvailable_ && filesystemPartitionErased()) {
    storageAvailable_ = LittleFS.format() && LittleFS.begin(false);
  }
  if (!storageAvailable_) {
    Serial.printf("[Storage] Filesystem unavailable; existing data preserved. History and recording synchronization are disabled until recovery.\n");
  }
  // NVS settings, radio acknowledgements and the recovery UI remain usable.
  return true;
}

void sanitizeSensorConfig(SensorConfig& config) {
  config.name[sizeof(config.name) - 1] = '\0';
  config.thingSpeakWriteKey[sizeof(config.thingSpeakWriteKey) - 1] = '\0';
  if (!config.revision) config.revision = 1;
  if (config.sleepSeconds < kMinSleepSeconds || config.sleepSeconds > kMaxSleepSeconds)
    config.sleepSeconds = 600;
  if (config.thingSpeakProfileSlot >= kMaxThingSpeakChannels) config.thingSpeakProfileSlot = 0xFF;
  if (!validSensorType(config.environmentalSensorType))
    config.environmentalSensorType = lil::protocol::EnvironmentalSensorType::kAutoDetect;
  if (!isfinite(config.temperatureOffsetC) || config.temperatureOffsetC < -10.0F || config.temperatureOffsetC > 10.0F)
    config.temperatureOffsetC = 0.466F;
  if (!isfinite(config.batteryCalibrationFactor) || config.batteryCalibrationFactor < 0.7F || config.batteryCalibrationFactor > 1.3F)
    config.batteryCalibrationFactor = 1.0F;
  uint16_t assigned = 0;
  uint8_t* fields[] = {&config.thingSpeakFields.temperature, &config.thingSpeakFields.humidity,
      &config.thingSpeakFields.pressure, &config.thingSpeakFields.iaq,
      &config.thingSpeakFields.battery, &config.thingSpeakFields.gasResistance};
  for (auto* field : fields) {
    if (*field > 8 || (*field && (assigned & (1U << *field)))) *field = 0;
    if (*field) assigned |= 1U << *field;
  }
  config.pendingFlags &= kSensorCommandFlags | kAwaitingProvisioningAck;
  if (lil::protocol::isLiveSensor(config.environmentalSensorType) ||
      !config.thingSpeakChannelId || !config.thingSpeakWriteKey[0]) config.cloudUploadEnabled = false;
}

void terminateStationStrings(StationConfig& config) {
  config.hostname[sizeof(config.hostname) - 1] = '\0';
  config.wifiSsid[sizeof(config.wifiSsid) - 1] = '\0';
  config.wifiPassword[sizeof(config.wifiPassword) - 1] = '\0';
  config.accessPointPassword[sizeof(config.accessPointPassword) - 1] = '\0';
  config.adminPassword[sizeof(config.adminPassword) - 1] = '\0';
  config.thingSpeakUserApiKey[sizeof(config.thingSpeakUserApiKey) - 1] = '\0';
  config.thingSpeakReadApiKey[sizeof(config.thingSpeakReadApiKey) - 1] = '\0';
  config.thingSpeakWriteApiKey[sizeof(config.thingSpeakWriteApiKey) - 1] = '\0';
  for (auto& channel : config.thingSpeakChannels) {
    channel.name[sizeof(channel.name) - 1] = '\0';
    channel.readApiKey[sizeof(channel.readApiKey) - 1] = '\0';
    channel.writeApiKey[sizeof(channel.writeApiKey) - 1] = '\0';
  }
}

void ConfigStore::applyDefaults(StationConfig& config) {
  config = StationConfig{};
  copyText(config.hostname, "w-charger");
  copyText(config.accessPointPassword, kDefaultSetupPassword);
  config.adminPassword[0] = '\0';
}

StationConfig ConfigStore::loadStationConfig() {
  StationConfig config{};
  const size_t stored = preferences_.getBytesLength(kStationKey);
  if (stored == sizeof(config)) {
    if (preferences_.getBytes(kStationKey, &config, sizeof(config)) != sizeof(config)) {
      applyDefaults(config);
      return config;
    }
  } else if (stored == sizeof(LegacyStationConfigV4)) {
    LegacyStationConfigV4 legacy{};
    if (preferences_.getBytes(kStationKey, &legacy, sizeof(legacy)) != sizeof(legacy)) {
      applyDefaults(config); return config;
    }
    migrateStationBase(legacy, config);
    config.thingSpeakDefaultChannelId = legacy.thingSpeakDefaultChannelId;
    copyStoredText(config.thingSpeakReadApiKey, legacy.thingSpeakReadApiKey);
    copyStoredText(config.thingSpeakWriteApiKey, legacy.thingSpeakWriteApiKey);
    if (legacy.thingSpeakDefaultChannelId != 0) {
      auto& profile = config.thingSpeakChannels[0];
      profile.occupied = true;
      strlcpy(profile.name, "Home-Sensor", sizeof(profile.name));
      profile.channelId = legacy.thingSpeakDefaultChannelId;
      copyStoredText(profile.readApiKey, legacy.thingSpeakReadApiKey);
      copyStoredText(profile.writeApiKey, legacy.thingSpeakWriteApiKey);
    }
    saveStationConfig(config);
  } else if (stored == sizeof(LegacyStationConfigV3)) {
    LegacyStationConfigV3 legacy{};
    if (preferences_.getBytes(kStationKey, &legacy, sizeof(legacy)) != sizeof(legacy)) {
      applyDefaults(config); return config;
    }
    migrateStationBase(legacy, config);
    saveStationConfig(config);
  } else {
    applyDefaults(config);
    saveStationConfig(config);
    return config;
  }

  terminateStationStrings(config);
  if (config.magic != kConfigMagic || config.version != kConfigVersion ||
      config.defaultSleepSeconds < kMinSleepSeconds ||
      config.defaultSleepSeconds > kMaxSleepSeconds ||
      config.fallbackWifiChannel < 1 || config.fallbackWifiChannel > 13) {
    applyDefaults(config);
    saveStationConfig(config);
  } else if (!validAdminPasswordHash(config.adminPassword)) {
    // Older development builds never stored a usable website password. Avoid
    // locking the owner out if that previously reserved field contains data.
    config.adminPassword[0] = '\0';
    saveStationConfig(config);
  }
  return config;
}

bool ConfigStore::saveStationConfig(const StationConfig& config) {
  return preferences_.putBytes(kStationKey, &config, sizeof(config)) == sizeof(config);
}

bool ConfigStore::loadSensorConfigs(SensorConfig* configs, size_t count) {
  if (configs == nullptr) {
    return false;
  }

  for (size_t i = 0; i < count; ++i) {
    const String key = sensorKey(i);
    // Preferences reports each absent node as an error.  Empty registry slots
    // are expected on every normal boot, so check the key before asking NVS
    // for its size.
    const size_t stored = preferences_.isKey(key.c_str())
                              ? preferences_.getBytesLength(key.c_str())
                              : 0;
    if (stored == sizeof(SensorConfig)) {
      if (preferences_.getBytes(key.c_str(), &configs[i], sizeof(SensorConfig)) != sizeof(SensorConfig)) {
        configs[i] = SensorConfig{}; continue;
      }
      if (configs[i].storageVersion != kSensorStorageVersion) {
        configs[i] = SensorConfig{};
      } else if (configs[i].thingSpeakProfileSlot >= kMaxThingSpeakChannels) {
        configs[i].thingSpeakProfileSlot = 0xFF;
      }
      if (!validSensorType(configs[i].environmentalSensorType) ||
          !isfinite(configs[i].temperatureOffsetC) ||
          configs[i].temperatureOffsetC < -10.0F ||
          configs[i].temperatureOffsetC > 10.0F ||
          !isfinite(configs[i].batteryCalibrationFactor) ||
          configs[i].batteryCalibrationFactor < 0.7F ||
          configs[i].batteryCalibrationFactor > 1.3F) {
        configs[i].environmentalSensorType =
            lil::protocol::EnvironmentalSensorType::kAutoDetect;
        configs[i].temperatureOffsetC = 0.466F;
        configs[i].batteryCalibrationFactor = 1.0F;
        saveSensorConfig(i, configs[i]);
      }
    } else if (stored == sizeof(LegacySensorConfigV5)) {
      LegacySensorConfigV5 legacy{};
      if (preferences_.getBytes(key.c_str(), &legacy, sizeof(legacy)) != sizeof(legacy)) {
        configs[i] = SensorConfig{}; continue;
      }
      migrateSensorBase(legacy, configs[i]);
      configs[i].cloudUploadEnabled = legacy.cloudUploadEnabled;
      configs[i].thingSpeakFields = legacy.thingSpeakFields;
      configs[i].provisioned = legacy.provisioned;
      configs[i].thingSpeakProfileSlot =
          legacy.thingSpeakProfileSlot < kMaxThingSpeakChannels
              ? legacy.thingSpeakProfileSlot
              : 0xFF;
      configs[i].environmentalSensorType = legacy.environmentalSensorType;
      configs[i].temperatureOffsetC = legacy.temperatureOffsetC;
      configs[i].batteryCalibrationFactor = legacy.batteryCalibrationFactor;
      saveSensorConfig(i, configs[i]);
    } else if (stored == sizeof(LegacySensorConfigV4)) {
      LegacySensorConfigV4 legacy{};
      if (preferences_.getBytes(key.c_str(), &legacy, sizeof(legacy)) != sizeof(legacy)) {
        configs[i] = SensorConfig{}; continue;
      }
      migrateSensorBase(legacy, configs[i]);
      configs[i].cloudUploadEnabled = legacy.cloudUploadEnabled;
      configs[i].thingSpeakFields = legacy.thingSpeakFields;
      configs[i].provisioned = legacy.provisioned;
      configs[i].thingSpeakProfileSlot =
          legacy.thingSpeakProfileSlot < kMaxThingSpeakChannels
              ? legacy.thingSpeakProfileSlot
              : 0xFF;
      configs[i].environmentalSensorType = legacy.environmentalSensorType;
      configs[i].temperatureOffsetC = legacy.temperatureOffsetC;
      saveSensorConfig(i, configs[i]);
    } else if (stored == sizeof(LegacySensorConfigV3)) {
      LegacySensorConfigV3 legacy{};
      if (preferences_.getBytes(key.c_str(), &legacy, sizeof(legacy)) != sizeof(legacy)) {
        configs[i] = SensorConfig{}; continue;
      }
      migrateSensorBase(legacy, configs[i]);
      configs[i].cloudUploadEnabled = legacy.cloudUploadEnabled;
      copyLegacyFields(legacy.thingSpeakFields,
                       configs[i].thingSpeakFields);
      configs[i].provisioned = legacy.provisioned;
      configs[i].thingSpeakProfileSlot =
          legacy.thingSpeakProfileSlot < kMaxThingSpeakChannels
              ? legacy.thingSpeakProfileSlot
              : 0xFF;
      saveSensorConfig(i, configs[i]);
    } else if (stored == sizeof(LegacySensorConfigV2)) {
      LegacySensorConfigV2 legacy{};
      if (preferences_.getBytes(key.c_str(), &legacy, sizeof(legacy)) != sizeof(legacy)) {
        configs[i] = SensorConfig{}; continue;
      }
      migrateSensorBase(legacy, configs[i]);
      configs[i].cloudUploadEnabled = false;
      copyLegacyFields(legacy.thingSpeakFields,
                       configs[i].thingSpeakFields);
      configs[i].provisioned = false;
      configs[i].thingSpeakProfileSlot = 0xFF;
      saveSensorConfig(i, configs[i]);
    } else if (stored == sizeof(LegacySensorConfig)) {
      LegacySensorConfig legacy{};
      if (preferences_.getBytes(key.c_str(), &legacy, sizeof(legacy)) != sizeof(legacy)) {
        configs[i] = SensorConfig{}; continue;
      }
      migrateSensorBase(legacy, configs[i]);
      configs[i].cloudUploadEnabled = false;
      configs[i].provisioned = false;
      configs[i].thingSpeakProfileSlot = 0xFF;
      saveSensorConfig(i, configs[i]);
    } else {
      configs[i] = SensorConfig{};
    }
    // Apply the same bounds after every migration path, before these values
    // control radio timing, string operations or cloud field selection.
    const SensorConfig previous = configs[i];
    sanitizeSensorConfig(configs[i]);
    if (memcmp(&previous, &configs[i], sizeof(previous))) saveSensorConfig(i, configs[i]);
  }
  return true;
}

bool ConfigStore::saveSensorConfig(size_t index, const SensorConfig& config) {
  if (index >= kMaxSensors) {
    return false;
  }
  const String key = sensorKey(index);
  return preferences_.putBytes(key.c_str(), &config, sizeof(config)) == sizeof(config);
}

bool ConfigStore::deleteSensorConfig(size_t index) {
  if (index >= kMaxSensors) {
    return false;
  }
  const String key = sensorKey(index);
  return !preferences_.isKey(key.c_str()) || preferences_.remove(key.c_str());
}

bool ConfigStore::loadSensorIdentities(SensorIdentity* identities, size_t count) {
  if (!identities || count != kMaxSensorIdentities) return false;
  const size_t bytes = count * sizeof(SensorIdentity);
  if (!preferences_.isKey("identities1")) return true;
  if (preferences_.getBytesLength("identities1") != bytes ||
      preferences_.getBytes("identities1", identities, bytes) != bytes) return false;
  for (size_t i = 0; i < count; ++i) {
    identities[i].name[24] = '\0';
    if (identities[i].motion.enabled && !validMotionReference(identities[i].motion)) identities[i].motion = {};
  }
  return true;
}

bool ConfigStore::saveSensorIdentities(const SensorIdentity* identities, size_t count) {
  if (!identities || count != kMaxSensorIdentities) return false;
  const size_t bytes = count * sizeof(SensorIdentity);
  return preferences_.putBytes("identities1", identities, bytes) == bytes;
}

size_t ConfigStore::historyBytesLength(size_t index) {
  if (index >= kMaxSensors) {
    return 0;
  }
  const String key = historyKey(index);
  File file = LittleFS.open(key, FILE_READ);
  return file ? file.size() : 0;
}

bool ConfigStore::loadHistory(size_t index, void* output, size_t length) {
  if (index >= kMaxSensors || output == nullptr || length == 0 ||
      historyBytesLength(index) != length) {
    return false;
  }
  return loadHistoryRange(index, 0, output, length);
}

bool ConfigStore::loadHistoryRange(size_t index, size_t offset, void* output,
                                   size_t length) {
  if (index >= kMaxSensors || output == nullptr || length == 0) {
    return false;
  }
  const String key = historyKey(index);
  File file = LittleFS.open(key, FILE_READ);
  if (!file || offset > file.size() || length > file.size() - offset ||
      !file.seek(offset)) {
    return false;
  }
  return file.read(static_cast<uint8_t*>(output), length) == length;
}

bool ConfigStore::saveHistory(size_t index, const void* data, size_t length) {
  if (index >= kMaxSensors || data == nullptr || length == 0) {
    return false;
  }
  const String key = historyKey(index);
  const String temporary = key + ".tmp";
  File file = LittleFS.open(temporary, FILE_WRITE);
  if (!file || file.write(static_cast<const uint8_t*>(data), length) != length) {
    if (file) {
      file.close();
    }
    LittleFS.remove(temporary);
    return false;
  }
  file.close();
  return LittleFS.rename(temporary, key);
}

bool ConfigStore::saveHistoryParts(size_t index, const void* header,
                                   size_t headerLength, const void* samples,
                                   size_t samplesLength) {
  if (index >= kMaxSensors || header == nullptr || headerLength == 0 ||
      (samplesLength > 0 && samples == nullptr)) {
    return false;
  }
  const String key = historyKey(index);
  const String temporary = key + ".tmp";
  File file = LittleFS.open(temporary, FILE_WRITE);
  const bool headerWritten =
      file && file.write(static_cast<const uint8_t*>(header), headerLength) ==
                  headerLength;
  const bool samplesWritten =
      headerWritten &&
      (samplesLength == 0 ||
       file.write(static_cast<const uint8_t*>(samples), samplesLength) ==
           samplesLength);
  if (!samplesWritten) {
    if (file) {
      file.close();
    }
    LittleFS.remove(temporary);
    return false;
  }
  file.close();
  return LittleFS.rename(temporary, key);
}

bool ConfigStore::updateHistory(size_t index, size_t expectedLength,
                                const void* header, size_t headerLength,
                                size_t sampleOffset, const void* sample,
                                size_t sampleLength) {
  if (index >= kMaxSensors || header == nullptr || headerLength == 0 ||
      sample == nullptr || sampleLength == 0 || sampleOffset < headerLength ||
      sampleOffset > expectedLength ||
      sampleLength > expectedLength - sampleOffset) {
    return false;
  }
  const String key = historyKey(index);
  File file = LittleFS.open(key, "r+");
  if (!file || file.size() != expectedLength || !file.seek(sampleOffset) ||
      file.write(static_cast<const uint8_t*>(sample), sampleLength) !=
          sampleLength) {
    return false;
  }
  file.flush();
  if (!file.seek(0) ||
      file.write(static_cast<const uint8_t*>(header), headerLength) !=
          headerLength) {
    return false;
  }
  file.flush();
  return true;
}

bool ConfigStore::deleteHistory(size_t index) {
  if (index >= kMaxSensors || !storageAvailable_) {
    return false;
  }
  const String key = historyKey(index);
  return !LittleFS.exists(key) || LittleFS.remove(key);
}

bool ConfigStore::loadLatestTelemetry(size_t index, void* output,
                                      size_t length) {
  if (index >= kMaxSensors || output == nullptr || length == 0) {
    return false;
  }
  const String key = latestTelemetryKey(index);
  File file = LittleFS.open(key, FILE_READ);
  return file && file.size() == length &&
         file.read(static_cast<uint8_t*>(output), length) == length;
}

bool ConfigStore::saveLatestTelemetry(size_t index, const void* data,
                                      size_t length) {
  if (index >= kMaxSensors || data == nullptr || length == 0) {
    return false;
  }
  const String key = latestTelemetryKey(index);
  const String temporary = key + ".tmp";
  File file = LittleFS.open(temporary, FILE_WRITE);
  if (!file || file.write(static_cast<const uint8_t*>(data), length) != length) {
    if (file) {
      file.close();
    }
    LittleFS.remove(temporary);
    return false;
  }
  file.close();
  return LittleFS.rename(temporary, key);
}

bool ConfigStore::deleteLatestTelemetry(size_t index) {
  if (index >= kMaxSensors || !storageAvailable_) {
    return false;
  }
  const String key = latestTelemetryKey(index);
  return !LittleFS.exists(key) || LittleFS.remove(key);
}

bool ConfigStore::factoryReset() {
  const bool preferencesCleared = preferences_.clear();
  const bool historyCleared = LittleFS.format();
  storageAvailable_ = historyCleared && LittleFS.begin(false);
  return preferencesCleared && historyCleared;
}

}  // namespace station

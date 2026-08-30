#include "sensor_registry.h"

#include <string.h>
#include <time.h>

namespace station {

namespace {
constexpr uint32_t kHistoryMagic = 0x48495354UL;  // "HIST"
constexpr uint16_t kHistoryVersion = 2;
constexpr uint32_t kLatestTelemetryMagic = 0x4C415354UL;  // "LAST"
constexpr time_t kMinimumValidTime = 1577836800;  // 2020-01-01 UTC

class LockGuard {
 public:
  explicit LockGuard(SemaphoreHandle_t mutex) : mutex_(mutex) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
  }
  ~LockGuard() { xSemaphoreGive(mutex_); }

 private:
  SemaphoreHandle_t mutex_;
};
}  // namespace

bool SensorRegistry::begin(ConfigStore& store, uint32_t defaultSleepSeconds) {
  store_ = &store;
  defaultSleepSeconds_ = constrain(defaultSleepSeconds, kMinSleepSeconds,
                                   kMaxSleepSeconds);
  mutex_ = xSemaphoreCreateMutex();
  if (mutex_ == nullptr || !store_->loadSensorConfigs(configs_, kMaxSensors)) {
    return false;
  }
  for (size_t i = 0; i < kMaxSensors; ++i) {
    if (!store_->loadHistory(i, &history_[i], sizeof(history_[i])) ||
        history_[i].magic != kHistoryMagic ||
        history_[i].version != kHistoryVersion) {
      history_[i] = StoredHistory{};
      history_[i].magic = kHistoryMagic;
      history_[i].version = kHistoryVersion;
    }
    LatestTelemetry latest{};
    if (configs_[i].occupied &&
        store_->loadLatestTelemetry(i, &latest, sizeof(latest)) &&
        latest.magic == kLatestTelemetryMagic &&
        latest.protocolVersion == lil::protocol::kVersion) {
      runtime_[i].telemetry = latest.telemetry;
      runtime_[i].stationRssi = latest.stationRssi;
      runtime_[i].hasPersistedTelemetry = true;
    }
  }
  return true;
}

int SensorRegistry::findIndexLocked(const uint8_t mac[6]) const {
  for (size_t i = 0; i < kMaxSensors; ++i) {
    if (configs_[i].occupied && memcmp(configs_[i].mac, mac, 6) == 0) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int SensorRegistry::allocateIndexLocked(const uint8_t mac[6]) {
  for (size_t i = 0; i < kMaxSensors; ++i) {
    if (!configs_[i].occupied) {
      configs_[i] = SensorConfig{};
      configs_[i].occupied = true;
      memcpy(configs_[i].mac, mac, 6);
      configs_[i].sleepSeconds = defaultSleepSeconds_;
      configs_[i].revision = 1;
      copyText(configs_[i].name, "Sensor " + formatMac(mac).substring(9));
      if (!store_->saveSensorConfig(i, configs_[i])) {
        configs_[i] = SensorConfig{};
        return -1;
      }
      history_[i] = StoredHistory{};
      history_[i].magic = kHistoryMagic;
      history_[i].version = kHistoryVersion;
      store_->deleteHistory(i);
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool SensorRegistry::registerTelemetry(
    const uint8_t mac[6], uint32_t sequence,
    const lil::protocol::TelemetryPayload& telemetry, int8_t rssi,
    SensorConfig& responseConfig, bool& duplicate) {
  LockGuard lock(mutex_);
  int index = findIndexLocked(mac);
  if (index < 0) {
    index = allocateIndexLocked(mac);
  }
  if (index < 0) {
    return false;
  }

  SensorConfig& config = configs_[index];
  auto& current = runtime_[index];
  const bool discoveryMode =
      (telemetry.flags & lil::protocol::kDiscoveryBeacon) != 0;
  // ESP-NOW receive callbacks and the sensor's configuration response cross
  // asynchronously. A discovery packet assembled immediately before the
  // provisioning response can therefore arrive after the first configured
  // measurement. Never let such an older packet replace real telemetry with
  // the discovery snapshot (PCB V0/zero values) or undo provisioning. The
  // boot counter keeps a real later reboot valid even when its sequence starts
  // over after a complete power loss.
  const bool staleWithinSameBoot =
      current.hasSequence &&
      telemetry.bootCount == current.telemetry.bootCount &&
      static_cast<int32_t>(sequence - current.lastSequence) <= 0;
  if (staleWithinSameBoot) {
    duplicate = true;
    responseConfig = config;
    return true;
  }
  if (telemetry.appliedConfigRevision == config.revision) {
    uint16_t acknowledgedFlags = config.pendingFlags & kSensorCommandFlags;
    // Provisioning is confirmed only by a packet which was created after the
    // sensor persisted provisioned=true. A stale discovery packet may already
    // carry an older/current revision but must not complete the handshake.
    if (!discoveryMode) {
      acknowledgedFlags |= config.pendingFlags & kAwaitingProvisioningAck;
    }
    if (acknowledgedFlags != 0) {
      config.pendingFlags &= ~acknowledgedFlags;
      if (!store_->saveSensorConfig(index, config)) {
        return false;
      }
    }
  }
  // Once provisioning has previously been confirmed, a new discovery packet
  // is an explicit statement from the node that its local NVS flag was erased.
  // During the initial handshake it is merely an in-flight packet from before
  // the sensor received the user's setup response and must be ignored.
  if (config.provisioned && discoveryMode &&
      (config.pendingFlags & kAwaitingProvisioningAck) == 0) {
    config.provisioned = false;
    config.pendingFlags &= ~kAwaitingProvisioningAck;
    ++config.revision;
    if (config.revision == 0) {
      config.revision = 1;
    }
    if (!store_->saveSensorConfig(index, config)) {
      return false;
    }
  }

  lil::protocol::TelemetryPayload displayTelemetry = telemetry;
  const bool rawFallback =
      (telemetry.flags & lil::protocol::kBme680RawFallback) != 0;
  const bool previousIaqAvailable =
      current.hasTelemetry &&
      (current.telemetry.capabilities & lil::protocol::kIaq) != 0 &&
      current.telemetry.iaqAccuracy > 0;
  if (rawFallback && previousIaqAvailable) {
    // A direct Bosch reading contains fresh T/H/P/gas values but no new BSEC
    // IAQ.  Keep the last real IAQ for the dashboard instead of replacing it
    // with the payload's default zero.  The raw-fallback flag remains set, so
    // callers can label the value stale and must not upload it as a new point.
    displayTelemetry.capabilities |= lil::protocol::kIaq;
    displayTelemetry.iaq = current.telemetry.iaq;
    displayTelemetry.iaqAccuracy = current.telemetry.iaqAccuracy;
  }

  duplicate = false;
  current.hasTelemetry = true;
  current.telemetry = displayTelemetry;
  current.stationRssi = rssi;
  current.lastSeenMs = millis();
  current.receivedPackets++;
  // Keep the setup measurement across the station restart. Discovery packets
  // contain a real, periodically refreshed sensor snapshot even though they
  // are intentionally excluded from history charts and cloud uploads.
  if (!discoveryMode || !current.hasPersistedTelemetry) {
    current.hasPersistedTelemetry = saveLatestTelemetryLocked(
        static_cast<size_t>(index), displayTelemetry, rssi);
  }
  if (!duplicate) {
    current.lastSequence = sequence;
    current.hasSequence = true;
  }
  if (!discoveryMode) {
    recordHistoryLocked(static_cast<size_t>(index), telemetry);
  }
  if (!config.bme680QuickStartComplete) {
    // Compatibility with configurations created before ULP-only operation.
    // There is no longer a separate quick-start acknowledgement.
    config.bme680QuickStartComplete = true;
    if (!store_->saveSensorConfig(index, config)) {
      return false;
    }
  }
  responseConfig = config;
  return true;
}

size_t SensorRegistry::views(SensorView* output, size_t capacity) const {
  if (output == nullptr || capacity == 0) {
    return 0;
  }
  LockGuard lock(mutex_);
  size_t written = 0;
  for (size_t i = 0; i < kMaxSensors && written < capacity; ++i) {
    if (configs_[i].occupied) {
      output[written].config = configs_[i];
      output[written].runtime = runtime_[i];
      ++written;
    }
  }
  return written;
}

bool SensorRegistry::updateConfig(const uint8_t mac[6], const String& name,
                                  uint32_t sleepSeconds, uint32_t channelId,
                                  const String& writeKey,
                                  bool uploadEnabled,
                                  const ThingSpeakFieldMapping& fields,
                                  uint8_t thingSpeakProfileSlot,
                                  lil::protocol::EnvironmentalSensorType sensorType,
                                  float temperatureOffsetC,
                                  float batteryCalibrationFactor) {
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return false;
  }
  SensorConfig& config = configs_[index];
  const bool wasProvisioned = config.provisioned;
  const bool sensorTypeChanged =
      config.environmentalSensorType != sensorType;
  copyText(config.name, name);
  config.sleepSeconds = constrain(sleepSeconds, kMinSleepSeconds,
                                  kMaxSleepSeconds);
  const bool channelChanged = config.thingSpeakChannelId != channelId;
  config.thingSpeakChannelId = channelId;
  if (writeKey.length() > 0) {
    copyText(config.thingSpeakWriteKey, writeKey);
  } else if (channelChanged || channelId == 0) {
    config.thingSpeakWriteKey[0] = '\0';
  }
  config.cloudUploadEnabled = uploadEnabled;
  config.thingSpeakFields = fields;
  config.thingSpeakProfileSlot = thingSpeakProfileSlot;
  config.environmentalSensorType = sensorType;
  config.temperatureOffsetC = constrain(temperatureOffsetC, -10.0F, 10.0F);
  config.batteryCalibrationFactor =
      constrain(batteryCalibrationFactor, 0.7F, 1.3F);
  config.provisioned = true;
  if (!wasProvisioned || sensorTypeChanged) {
    config.bme680QuickStartComplete = true;
  }
  if (!wasProvisioned) {
    config.pendingFlags |= kAwaitingProvisioningAck;
  }
  ++config.revision;
  if (config.revision == 0) {
    config.revision = 1;
  }
  return store_->saveSensorConfig(index, config);
}

bool SensorRegistry::setThingSpeakChannel(const uint8_t mac[6],
                                           uint32_t channelId,
                                           const String& writeKey,
                                           uint8_t thingSpeakProfileSlot) {
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return false;
  }
  SensorConfig& config = configs_[index];
  const bool wasProvisioned = config.provisioned;
  config.thingSpeakChannelId = channelId;
  copyText(config.thingSpeakWriteKey, writeKey);
  config.cloudUploadEnabled = channelId != 0 && writeKey.length() > 0;
  config.thingSpeakProfileSlot = thingSpeakProfileSlot;
  config.provisioned = true;
  if (!wasProvisioned) {
    config.pendingFlags |= kAwaitingProvisioningAck;
    ++config.revision;
    if (config.revision == 0) {
      config.revision = 1;
    }
  }
  return store_->saveSensorConfig(index, config);
}

bool SensorRegistry::syncThingSpeakProfile(
    uint8_t slot, const ThingSpeakChannelProfile& profile) {
  if (slot >= kMaxThingSpeakChannels || !profile.occupied) {
    return false;
  }
  LockGuard lock(mutex_);
  bool success = true;
  for (size_t i = 0; i < kMaxSensors; ++i) {
    if (!configs_[i].occupied ||
        configs_[i].thingSpeakProfileSlot != slot) {
      continue;
    }
    configs_[i].thingSpeakChannelId = profile.channelId;
    copyText(configs_[i].thingSpeakWriteKey, profile.writeApiKey);
    if (profile.channelId == 0 || profile.writeApiKey[0] == '\0') {
      configs_[i].cloudUploadEnabled = false;
    }
    success = store_->saveSensorConfig(i, configs_[i]) && success;
  }
  return success;
}

bool SensorRegistry::removeThingSpeakProfile(uint8_t slot) {
  if (slot >= kMaxThingSpeakChannels) {
    return false;
  }
  LockGuard lock(mutex_);
  bool success = true;
  for (size_t i = 0; i < kMaxSensors; ++i) {
    if (!configs_[i].occupied ||
        configs_[i].thingSpeakProfileSlot != slot) {
      continue;
    }
    configs_[i].thingSpeakProfileSlot = 0xFF;
    configs_[i].thingSpeakChannelId = 0;
    configs_[i].thingSpeakWriteKey[0] = '\0';
    configs_[i].cloudUploadEnabled = false;
    success = store_->saveSensorConfig(i, configs_[i]) && success;
  }
  return success;
}

bool SensorRegistry::requestFactoryReset(const uint8_t mac[6]) {
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return false;
  }
  SensorConfig& config = configs_[index];
  config.provisioned = false;
  config.pendingFlags &= ~kAwaitingProvisioningAck;
  config.pendingFlags |= lil::protocol::kFactoryReset;
  ++config.revision;
  if (config.revision == 0) {
    config.revision = 1;
  }
  return store_->saveSensorConfig(index, config);
}

bool SensorRegistry::requestIaqCalibrationReset(const uint8_t mac[6]) {
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return false;
  }
  configs_[index].pendingFlags |= lil::protocol::kResetIaqCalibration;
  configs_[index].bme680QuickStartComplete = true;
  ++configs_[index].revision;
  if (configs_[index].revision == 0) {
    configs_[index].revision = 1;
  }
  return store_->saveSensorConfig(index, configs_[index]);
}

bool SensorRegistry::deleteSensor(const uint8_t mac[6]) {
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0 || !store_->deleteSensorConfig(index) ||
      !store_->deleteHistory(index) ||
      !store_->deleteLatestTelemetry(index)) {
    return false;
  }
  configs_[index] = SensorConfig{};
  runtime_[index] = SensorRuntime{};
  history_[index] = StoredHistory{};
  return true;
}

bool SensorRegistry::saveLatestTelemetryLocked(
    size_t index, const lil::protocol::TelemetryPayload& telemetry,
    int8_t stationRssi) {
  LatestTelemetry latest{};
  latest.magic = kLatestTelemetryMagic;
  latest.protocolVersion = lil::protocol::kVersion;
  latest.telemetry = telemetry;
  latest.stationRssi = stationRssi;
  return store_->saveLatestTelemetry(index, &latest, sizeof(latest));
}

bool SensorRegistry::recordHistoryLocked(
    size_t index, const lil::protocol::TelemetryPayload& telemetry) {
  const time_t now = time(nullptr);
  if (index >= kMaxSensors || now < kMinimumValidTime) {
    return false;
  }
  const uint32_t bucket =
      static_cast<uint32_t>(now) / kHistoryBucketSeconds *
      kHistoryBucketSeconds;
  const size_t slot = (bucket / kHistoryBucketSeconds) % kHistoryBucketCount;
  HistorySample& sample = history_[index].samples[slot];
  // A bucket contains the newest received measurement in that half-hour.
  // Updating it also persists the dashboard's current value, while the fixed
  // ring still exposes at most 48 chart points per sensor.
  sample = HistorySample{};
  sample.timestamp = bucket;
  sample.capabilities = telemetry.capabilities;
  sample.temperature = telemetry.temperatureC;
  sample.humidity = telemetry.humidityPercent;
  sample.pressure = telemetry.pressureHpa;
  sample.iaq = telemetry.iaq;
  sample.gasKohms = telemetry.gasResistanceOhms / 1000.0F;
  sample.batteryVolts = telemetry.batteryMillivolts / 1000.0F;
  sample.iaqAccuracy = telemetry.iaqAccuracy;
  sample.sensorType = static_cast<uint8_t>(telemetry.sensorType);
  sample.pcbVersion = telemetry.pcbVersion;
  history_[index].magic = kHistoryMagic;
  history_[index].version = kHistoryVersion;
  ++history_[index].revision;
  if (history_[index].revision == 0) {
    history_[index].revision = 1;
  }
  return store_->saveHistory(index, &history_[index], sizeof(history_[index]));
}

size_t SensorRegistry::history(const uint8_t mac[6], HistorySample* output,
                               size_t capacity) const {
  if (output == nullptr || capacity == 0) {
    return 0;
  }
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return 0;
  }
  const time_t now = time(nullptr);
  const uint32_t cutoff =
      now >= kMinimumValidTime
          ? static_cast<uint32_t>(now) - 24UL * 60UL * 60UL
          : 0;
  size_t count = 0;
  for (const HistorySample& sample : history_[index].samples) {
    if (sample.timestamp != 0 && sample.timestamp >= cutoff &&
        count < capacity) {
      output[count++] = sample;
    }
  }
  for (size_t i = 1; i < count; ++i) {
    const HistorySample value = output[i];
    size_t j = i;
    while (j > 0 && output[j - 1].timestamp > value.timestamp) {
      output[j] = output[j - 1];
      --j;
    }
    output[j] = value;
  }
  return count;
}

uint32_t SensorRegistry::historyRevision(const uint8_t mac[6]) const {
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return 0;
  }
  return history_[index].revision;
}

bool SensorRegistry::findConfig(const uint8_t mac[6], SensorConfig& output) const {
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return false;
  }
  output = configs_[index];
  return true;
}

bool SensorRegistry::findView(const uint8_t mac[6], SensorView& output) const {
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return false;
  }
  output.config = configs_[index];
  output.runtime = runtime_[index];
  return true;
}

String SensorRegistry::formatMac(const uint8_t mac[6]) {
  char text[18];
  snprintf(text, sizeof(text), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0],
           mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(text);
}

bool SensorRegistry::parseMac(const String& text, uint8_t output[6]) {
  unsigned int values[6];
  if (sscanf(text.c_str(), "%x:%x:%x:%x:%x:%x", &values[0], &values[1],
             &values[2], &values[3], &values[4], &values[5]) != 6) {
    return false;
  }
  for (size_t i = 0; i < 6; ++i) {
    if (values[i] > 0xFF) {
      return false;
    }
    output[i] = static_cast<uint8_t>(values[i]);
  }
  return true;
}

}  // namespace station

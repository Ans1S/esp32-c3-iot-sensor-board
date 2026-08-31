#include "sensor_registry.h"

#include <algorithm>
#include <esp_heap_caps.h>
#include <limits.h>
#include <string.h>
#include <time.h>

namespace station {

namespace {
constexpr uint32_t kHistoryMagic = 0x48495354UL;  // "HIST"
constexpr uint16_t kHistoryVersion = 3;
constexpr size_t kLegacyHistoryBucketCount = 48;
constexpr uint32_t kLatestTelemetryMagic = 0x4C415354UL;  // "LAST"
constexpr time_t kMinimumValidTime = 1577836800;  // 2020-01-01 UTC

#pragma pack(push, 1)
struct PersistedHistoryHeader {
  uint32_t magic = kHistoryMagic;
  uint16_t version = kHistoryVersion;
  uint16_t sampleSize = sizeof(HistorySample);
  uint32_t revision = 0;
  uint32_t bucketSeconds = 0;
  uint16_t capacity = 0;
  uint16_t reserved = 0;
};

struct LegacyHistorySampleV2 {
  uint32_t timestamp = 0;
  uint16_t capabilities = 0;
  float temperature = 0.0F;
  float humidity = 0.0F;
  float pressure = 0.0F;
  float iaq = 0.0F;
  float gasKohms = 0.0F;
  float batteryVolts = 0.0F;
  uint8_t iaqAccuracy = 0;
  uint8_t sensorType = 0;
  uint8_t pcbVersion = 0;
};
#pragma pack(pop)

struct LegacyStoredHistoryV2 {
  uint32_t magic = 0;
  uint16_t version = 0;
  uint32_t revision = 0;
  LegacyHistorySampleV2 samples[kLegacyHistoryBucketCount]{};
};

static_assert(sizeof(HistorySample) == 23,
              "History samples must remain compact");
static_assert(sizeof(LegacyStoredHistoryV2) == 1596,
              "Legacy history layout changed unexpectedly");

uint32_t normalizedHistoryBucketSeconds(uint32_t bucketSeconds) {
  return constrain(bucketSeconds, kMinSleepSeconds, kMaxSleepSeconds);
}

size_t historyBucketCount(uint32_t bucketSeconds) {
  const uint32_t normalized = normalizedHistoryBucketSeconds(bucketSeconds);
  return (kHistoryWindowSeconds + normalized - 1) / normalized;
}

HistorySample* allocateHistorySamples(size_t count) {
  if (count == 0 || count > UINT16_MAX) {
    return nullptr;
  }
  auto* samples = static_cast<HistorySample*>(heap_caps_calloc(
      count, sizeof(HistorySample), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (samples == nullptr) {
    samples = static_cast<HistorySample*>(
        heap_caps_calloc(count, sizeof(HistorySample), MALLOC_CAP_8BIT));
  }
  return samples;
}

int16_t encodeSignedHundredths(float value) {
  if (!isfinite(value)) {
    return 0;
  }
  const long scaled = lroundf(value * 100.0F);
  return static_cast<int16_t>(
      max<long>(INT16_MIN, min<long>(INT16_MAX, scaled)));
}

uint16_t encodeUnsigned(float value, float scale) {
  if (!isfinite(value) || value <= 0.0F) {
    return 0;
  }
  const long scaled = lroundf(value * scale);
  return static_cast<uint16_t>(
      min<unsigned long>(UINT16_MAX, static_cast<unsigned long>(scaled)));
}

HistorySample compactLegacySample(const LegacyHistorySampleV2& source) {
  HistorySample sample{};
  sample.timestamp = source.timestamp;
  sample.capabilities = source.capabilities;
  sample.temperatureCentiC = encodeSignedHundredths(source.temperature);
  sample.humidityCentiPercent = encodeUnsigned(source.humidity, 100.0F);
  sample.pressureDeciHpa = encodeUnsigned(source.pressure, 10.0F);
  sample.iaqDeci = encodeUnsigned(source.iaq, 10.0F);
  sample.gasResistanceOhms =
      source.gasKohms <= 0.0F || !isfinite(source.gasKohms)
          ? 0
          : static_cast<uint32_t>(
                min<double>(UINT32_MAX, round(source.gasKohms * 1000.0)));
  sample.batteryMillivolts =
      encodeUnsigned(source.batteryVolts, 1000.0F);
  sample.iaqAccuracy = source.iaqAccuracy;
  sample.sensorType = source.sensorType;
  sample.pcbVersion = source.pcbVersion;
  return sample;
}

PersistedHistoryHeader persistedHeader(uint32_t revision,
                                       uint32_t bucketSeconds,
                                       size_t capacity) {
  PersistedHistoryHeader header{};
  header.revision = revision;
  header.bucketSeconds = bucketSeconds;
  header.capacity = static_cast<uint16_t>(capacity);
  return header;
}

size_t persistedHistoryLength(size_t capacity) {
  return sizeof(PersistedHistoryHeader) + capacity * sizeof(HistorySample);
}

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
    if (configs_[i].occupied && !loadHistoryLocked(i) &&
        !configureHistoryLocked(i, configs_[i].sleepSeconds, false)) {
      Serial.printf("[History] Could not allocate history for sensor %u\n",
                    static_cast<unsigned>(i));
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

bool SensorRegistry::loadHistoryLocked(size_t index) {
  if (index >= kMaxSensors || !configs_[index].occupied) {
    return false;
  }
  const size_t storedLength = store_->historyBytesLength(index);
  const uint32_t desiredBucketSeconds =
      normalizedHistoryBucketSeconds(configs_[index].sleepSeconds);

  if (storedLength == sizeof(LegacyStoredHistoryV2)) {
    LegacyStoredHistoryV2 legacy{};
    if (!store_->loadHistory(index, &legacy, sizeof(legacy)) ||
        legacy.magic != kHistoryMagic || legacy.version != 2 ||
        !configureHistoryLocked(index, desiredBucketSeconds, false, false)) {
      return false;
    }
    history_[index].revision = legacy.revision;
    for (const LegacyHistorySampleV2& source : legacy.samples) {
      if (source.timestamp == 0) {
        continue;
      }
      HistorySample sample = compactLegacySample(source);
      const uint32_t bucket = sample.timestamp / desiredBucketSeconds *
                              desiredBucketSeconds;
      const size_t slot =
          (bucket / desiredBucketSeconds) % history_[index].capacity;
      HistorySample& destination = history_[index].samples[slot];
      if (destination.timestamp == 0 ||
          sample.timestamp >= destination.timestamp) {
        destination = sample;
      }
    }
    for (size_t i = 0; i < history_[index].capacity; ++i) {
      if (history_[index].samples[i].timestamp != 0) {
        history_[index].samples[i].timestamp =
            history_[index].samples[i].timestamp / desiredBucketSeconds *
            desiredBucketSeconds;
      }
    }
    if (!persistHistoryLocked(index)) {
      Serial.printf("[History] Could not migrate history for sensor %u\n",
                    static_cast<unsigned>(index));
    }
    return true;
  }

  PersistedHistoryHeader header{};
  if (storedLength < sizeof(header) ||
      !store_->loadHistoryRange(index, 0, &header, sizeof(header)) ||
      header.magic != kHistoryMagic || header.version != kHistoryVersion ||
      header.sampleSize != sizeof(HistorySample) || header.capacity == 0 ||
      header.bucketSeconds < kMinSleepSeconds ||
      header.bucketSeconds > kMaxSleepSeconds ||
      header.capacity != historyBucketCount(header.bucketSeconds) ||
      storedLength != persistedHistoryLength(header.capacity)) {
    return false;
  }

  HistorySample* samples = allocateHistorySamples(header.capacity);
  if (samples == nullptr ||
      !store_->loadHistoryRange(index, sizeof(header), samples,
                                header.capacity * sizeof(HistorySample))) {
    if (samples != nullptr) {
      heap_caps_free(samples);
    }
    return false;
  }
  history_[index].revision = header.revision;
  history_[index].bucketSeconds = header.bucketSeconds;
  history_[index].capacity = header.capacity;
  history_[index].samples = samples;
  if (header.bucketSeconds != desiredBucketSeconds &&
      !configureHistoryLocked(index, desiredBucketSeconds, true)) {
    Serial.printf("[History] Could not resize history for sensor %u\n",
                  static_cast<unsigned>(index));
  }
  return true;
}

bool SensorRegistry::configureHistoryLocked(size_t index,
                                            uint32_t bucketSeconds,
                                            bool preserveSamples,
                                            bool persistImmediately) {
  if (index >= kMaxSensors) {
    return false;
  }
  const uint32_t normalized = normalizedHistoryBucketSeconds(bucketSeconds);
  const size_t capacity = historyBucketCount(normalized);
  HistorySample* samples = allocateHistorySamples(capacity);
  if (samples == nullptr) {
    return false;
  }

  StoredHistory previous = history_[index];
  if (preserveSamples && previous.samples != nullptr) {
    const time_t now = time(nullptr);
    const uint32_t cutoff =
        now >= kMinimumValidTime
            ? static_cast<uint32_t>(now) - kHistoryWindowSeconds
            : 0;
    for (size_t i = 0; i < previous.capacity; ++i) {
      const HistorySample& source = previous.samples[i];
      if (source.timestamp == 0 || source.timestamp < cutoff) {
        continue;
      }
      const uint32_t bucket = source.timestamp / normalized * normalized;
      const size_t slot = (bucket / normalized) % capacity;
      if (samples[slot].timestamp == 0 ||
          source.timestamp >= samples[slot].timestamp) {
        samples[slot] = source;
      }
    }
    for (size_t i = 0; i < capacity; ++i) {
      if (samples[i].timestamp != 0) {
        samples[i].timestamp =
            samples[i].timestamp / normalized * normalized;
      }
    }
  }

  history_[index].revision = previous.revision;
  history_[index].bucketSeconds = normalized;
  history_[index].capacity = capacity;
  history_[index].samples = samples;
  if (previous.samples != nullptr) {
    ++history_[index].revision;
    if (history_[index].revision == 0) {
      history_[index].revision = 1;
    }
    heap_caps_free(previous.samples);
  }
  // Legacy migration defers replacement until the complete converted ring is
  // in memory, so an interrupted conversion can be retried at the next boot.
  if (persistImmediately && !persistHistoryLocked(index)) {
    Serial.printf("[History] Could not persist history layout for sensor %u\n",
                  static_cast<unsigned>(index));
  }
  return true;
}

bool SensorRegistry::persistHistoryLocked(size_t index) {
  if (index >= kMaxSensors || history_[index].samples == nullptr ||
      history_[index].capacity == 0) {
    return false;
  }
  const PersistedHistoryHeader header = persistedHeader(
      history_[index].revision, history_[index].bucketSeconds,
      history_[index].capacity);
  return store_->saveHistoryParts(
      index, &header, sizeof(header), history_[index].samples,
      history_[index].capacity * sizeof(HistorySample));
}

bool SensorRegistry::persistHistorySampleLocked(size_t index, size_t slot) {
  if (index >= kMaxSensors || history_[index].samples == nullptr ||
      slot >= history_[index].capacity) {
    return false;
  }
  const PersistedHistoryHeader header = persistedHeader(
      history_[index].revision, history_[index].bucketSeconds,
      history_[index].capacity);
  const size_t expectedLength =
      persistedHistoryLength(history_[index].capacity);
  const size_t sampleOffset =
      sizeof(header) + slot * sizeof(HistorySample);
  return store_->updateHistory(index, expectedLength, &header, sizeof(header),
                               sampleOffset, &history_[index].samples[slot],
                               sizeof(HistorySample)) ||
         persistHistoryLocked(index);
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
      store_->deleteHistory(i);
      if (!configureHistoryLocked(i, configs_[i].sleepSeconds, false)) {
        Serial.printf("[History] Could not allocate history for sensor %u\n",
                      static_cast<unsigned>(i));
      }
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
  const uint32_t newSleepSeconds =
      constrain(sleepSeconds, kMinSleepSeconds, kMaxSleepSeconds);
  if (history_[index].bucketSeconds != newSleepSeconds &&
      !configureHistoryLocked(static_cast<size_t>(index), newSleepSeconds,
                              true)) {
    Serial.printf("[History] Could not apply the new interval for sensor %u\n",
                  static_cast<unsigned>(index));
  }
  config.sleepSeconds = newSleepSeconds;
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
  if (history_[index].samples != nullptr) {
    heap_caps_free(history_[index].samples);
  }
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
  const uint32_t desiredBucketSeconds =
      normalizedHistoryBucketSeconds(configs_[index].sleepSeconds);
  if ((history_[index].samples == nullptr ||
       history_[index].bucketSeconds != desiredBucketSeconds) &&
      !configureHistoryLocked(index, desiredBucketSeconds, true)) {
    return false;
  }
  const uint32_t bucket = static_cast<uint32_t>(now) /
                              history_[index].bucketSeconds *
                          history_[index].bucketSeconds;
  const size_t slot =
      (bucket / history_[index].bucketSeconds) % history_[index].capacity;
  HistorySample& sample = history_[index].samples[slot];
  // A bucket contains the newest received measurement in one configured
  // sensor interval. The ring size changes with that interval while its time
  // span remains 24 hours.
  sample = HistorySample{};
  sample.timestamp = bucket;
  sample.capabilities = telemetry.capabilities;
  sample.temperatureCentiC = encodeSignedHundredths(telemetry.temperatureC);
  sample.humidityCentiPercent =
      encodeUnsigned(telemetry.humidityPercent, 100.0F);
  sample.pressureDeciHpa = encodeUnsigned(telemetry.pressureHpa, 10.0F);
  sample.iaqDeci = encodeUnsigned(telemetry.iaq, 10.0F);
  sample.gasResistanceOhms = telemetry.gasResistanceOhms;
  sample.batteryMillivolts = telemetry.batteryMillivolts;
  sample.iaqAccuracy = telemetry.iaqAccuracy;
  sample.sensorType = static_cast<uint8_t>(telemetry.sensorType);
  sample.pcbVersion = telemetry.pcbVersion;
  ++history_[index].revision;
  if (history_[index].revision == 0) {
    history_[index].revision = 1;
  }
  return persistHistorySampleLocked(index, slot);
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
          ? static_cast<uint32_t>(now) - kHistoryWindowSeconds
          : 0;
  size_t count = 0;
  for (size_t i = 0; i < history_[index].capacity; ++i) {
    const HistorySample& sample = history_[index].samples[i];
    if (sample.timestamp != 0 && sample.timestamp >= cutoff &&
        count < capacity) {
      output[count++] = sample;
    }
  }
  std::sort(output, output + count,
            [](const HistorySample& first, const HistorySample& second) {
              return first.timestamp < second.timestamp;
            });
  return count;
}

size_t SensorRegistry::historyCapacity(const uint8_t mac[6]) const {
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  return index < 0 ? 0 : history_[index].capacity;
}

uint32_t SensorRegistry::historyBucketSeconds(const uint8_t mac[6]) const {
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return 0;
  }
  return history_[index].bucketSeconds != 0
             ? history_[index].bucketSeconds
             : normalizedHistoryBucketSeconds(configs_[index].sleepSeconds);
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
  int consumed = 0;
  if (sscanf(text.c_str(), "%x:%x:%x:%x:%x:%x%n", &values[0], &values[1],
             &values[2], &values[3], &values[4], &values[5], &consumed) != 6 ||
      consumed != static_cast<int>(text.length())) {
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

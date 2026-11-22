#include "sensor_registry.h"
#include "sensor_timing.h"
#include "normal_history_policy.h"
#include "telemetry_validation.h"
#include "precision_cadence.h"

#include <algorithm>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <iterator>
#include <limits.h>
#include <memory>
#include <new>
#include <string.h>
#include <time.h>

namespace station {

namespace {
constexpr uint32_t kHistoryMagic = 0x48495354UL;  // "HIST"
constexpr uint16_t kHistoryVersion = 4;
constexpr size_t kLegacyHistoryBucketCount = 48;
constexpr uint32_t kLatestTelemetryMagic = 0x4C415354UL;  // "LAST"
constexpr time_t kMinimumValidTime = 1577836800;  // 2020-01-01 UTC
constexpr int8_t kTxPowerLevelsQuarterDbm[] = {34, 44, 52, 60};
constexpr int8_t kDefaultTxPowerQuarterDbm = 52;  // 13 dBm.
constexpr int8_t kPcbV4TxPowerQuarterDbm = 44;    // 11 dBm.
constexpr int8_t kStrongSignalRssi = -55;
constexpr int8_t kWeakSignalRssi = -72;
constexpr int8_t kEmergencySignalRssi = -82;
constexpr uint8_t kStrongSamplesBeforeReduction = 8;
constexpr uint8_t kWeakSamplesBeforeIncrease = 2;

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

static_assert(sizeof(HistorySample) == 40,
              "History samples must remain compact");
static_assert(sizeof(LegacyStoredHistoryV2) == 1596,
              "Legacy history layout changed unexpectedly");

uint32_t normalizedHistoryBucketSeconds(uint32_t bucketSeconds) {
  return constrain(bucketSeconds, kMinSleepSeconds, kMaxSleepSeconds);
}

size_t historyBucketCount(uint32_t bucketSeconds) {
  const uint32_t normalized = normalizedHistoryBucketSeconds(bucketSeconds);
  return normalized < 60 ? 900 : (kHistoryWindowSeconds + normalized - 1) / normalized;
}

HistorySample* allocateHistorySamples(size_t count) {
  if (count == 0 || count > UINT16_MAX) {
    return nullptr;
  }
  auto* samples = static_cast<HistorySample*>(heap_caps_calloc(
      count, sizeof(HistorySample), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (samples == nullptr &&
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) >
          count * sizeof(HistorySample) + 96U * 1024U) {
    samples = static_cast<HistorySample*>(
        heap_caps_calloc(count, sizeof(HistorySample), MALLOC_CAP_8BIT));
  }
  return samples;
}

int16_t encodeSignedHundredths(float value) {
  if (!isfinite(value)) {
    return 0;
  }
  // Bound before scaling: a finite input can overflow the multiplication or
  // lroundf's integer range long before the final compact-value clamp.
  if (value >= INT16_MAX / 100.0F) return INT16_MAX;
  if (value <= INT16_MIN / 100.0F) return INT16_MIN;
  const long scaled = lroundf(value * 100.0F);
  return static_cast<int16_t>(
      max<long>(INT16_MIN, min<long>(INT16_MAX, scaled)));
}

uint16_t encodeUnsigned(float value, float scale) {
  if (!isfinite(value) || value <= 0.0F) {
    return 0;
  }
  if (value >= UINT16_MAX / scale) return UINT16_MAX;
  const long scaled = lroundf(value * scale);
  return static_cast<uint16_t>(
      min<unsigned long>(UINT16_MAX, static_cast<unsigned long>(scaled)));
}

uint32_t encodeGasResistance(float value) {
  // Legacy history also receives unsupported/failed-reading placeholders.
  // Keep ordinary truncation, but never cast NaN or an out-of-range float.
  if (!isfinite(value) || value <= 0.0F) return 0;
  if (value >= 4294967040.0F) return UINT32_MAX;
  return static_cast<uint32_t>(value);
}

bool sensorConfigEquals(const SensorConfig& a, const SensorConfig& b) {
  return a.occupied == b.occupied &&
         memcmp(a.mac, b.mac, sizeof(a.mac)) == 0 &&
         strncmp(a.name, b.name, sizeof(a.name)) == 0 &&
         a.revision == b.revision &&
         a.sleepSeconds == b.sleepSeconds &&
         a.thingSpeakChannelId == b.thingSpeakChannelId &&
         strncmp(a.thingSpeakWriteKey, b.thingSpeakWriteKey,
                 sizeof(a.thingSpeakWriteKey)) == 0 &&
         a.pendingFlags == b.pendingFlags &&
         a.cloudUploadEnabled == b.cloudUploadEnabled &&
         a.thingSpeakFields.temperature == b.thingSpeakFields.temperature &&
         a.thingSpeakFields.humidity == b.thingSpeakFields.humidity &&
         a.thingSpeakFields.pressure == b.thingSpeakFields.pressure &&
         a.thingSpeakFields.iaq == b.thingSpeakFields.iaq &&
         a.thingSpeakFields.battery == b.thingSpeakFields.battery &&
         a.thingSpeakFields.gasResistance == b.thingSpeakFields.gasResistance &&
         a.provisioned == b.provisioned &&
         a.thingSpeakProfileSlot == b.thingSpeakProfileSlot &&
         a.environmentalSensorType == b.environmentalSensorType &&
         a.temperatureOffsetC == b.temperatureOffsetC &&
         a.batteryCalibrationFactor == b.batteryCalibrationFactor &&
         a.bme680QuickStartComplete == b.bme680QuickStartComplete &&
         a.storageVersion == b.storageVersion;
}

HistorySample compactLegacySample(const LegacyHistorySampleV2& source) {
  HistorySample sample{};
  sample.timestamp = source.timestamp;
  sample.capabilities = source.capabilities;
  sample.temperatureCentiC = encodeSignedHundredths(source.temperature);
  sample.humidityCentiPercent = encodeUnsigned(source.humidity, 100.0F);
  sample.pressureDeciHpa = encodeUnsigned(source.pressure, 10.0F);
  sample.iaqDeci = encodeUnsigned(source.iaq, 10.0F);
  if (source.gasKohms <= 0.0F || !isfinite(source.gasKohms)) {
    sample.gasResistanceOhms = 0;
  } else {
    const float ohms = source.gasKohms * 1000.0F;
    sample.gasResistanceOhms =
        ohms >= 4294967040.0F ? UINT32_MAX : static_cast<uint32_t>(roundf(ohms));
  }
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
  ~LockGuard() { unlock(); }
  void unlock() { if (held_) { xSemaphoreGive(mutex_); held_ = false; } }
  void relock() { if (!held_) { xSemaphoreTake(mutex_, portMAX_DELAY); held_ = true; } }

 private:
  SemaphoreHandle_t mutex_;
  bool held_ = true;
};
}  // namespace

void SensorRegistry::updateTxPowerLocked(size_t index, int8_t rssi,
                                         uint8_t pcbVersion) {
  SensorRuntime& current = runtime_[index];
  const bool validRssi = rssi < 0;
  if (!current.txPowerInitialized) {
    // V4 starts one documented hardware power step below V3 only when the
    // first received packet still has comfortable link margin.
    current.sensorTxPowerQuarterDbm =
        pcbVersion >= 4 && validRssi && rssi >= -68
            ? kPcbV4TxPowerQuarterDbm
            : kDefaultTxPowerQuarterDbm;
    current.txPowerInitialized = true;
  }
  if (!validRssi) {
    return;
  }

  if (rssi <= kEmergencySignalRssi) {
    current.weakSignalSamples = kWeakSamplesBeforeIncrease;
    current.strongSignalSamples = 0;
  } else if (rssi <= kWeakSignalRssi) {
    current.strongSignalSamples = 0;
    if (current.weakSignalSamples < kWeakSamplesBeforeIncrease) {
      ++current.weakSignalSamples;
    }
  } else if (rssi >= kStrongSignalRssi) {
    current.weakSignalSamples = 0;
    if (current.strongSignalSamples < kStrongSamplesBeforeReduction) {
      ++current.strongSignalSamples;
    }
  } else {
    current.strongSignalSamples = 0;
    current.weakSignalSamples = 0;
  }

  size_t level = 0;
  while (level + 1 < std::size(kTxPowerLevelsQuarterDbm) &&
         kTxPowerLevelsQuarterDbm[level] <
             current.sensorTxPowerQuarterDbm) {
    ++level;
  }
  const size_t minimumLevel = pcbVersion >= 4 ? 0 : 1;
  if (current.weakSignalSamples >= kWeakSamplesBeforeIncrease &&
      level + 1 < std::size(kTxPowerLevelsQuarterDbm)) {
    current.sensorTxPowerQuarterDbm = kTxPowerLevelsQuarterDbm[level + 1];
    current.weakSignalSamples = 0;
  } else if (current.strongSignalSamples >=
                 kStrongSamplesBeforeReduction &&
             level > minimumLevel) {
    current.sensorTxPowerQuarterDbm = kTxPowerLevelsQuarterDbm[level - 1];
    current.strongSignalSamples = 0;
  }
}

SensorRegistry::~SensorRegistry() {
  if (identitiesScratch_) {
#ifdef ESP_PLATFORM
    heap_caps_free(identitiesScratch_);
#else
    delete[] identitiesScratch_;
#endif
    identitiesScratch_ = nullptr;
  }
  for (auto& h : history_) {
    if (h.samples) { heap_caps_free(h.samples); h.samples = nullptr; }
  }
  for (auto& l : live_) {
    if (l.samples) { heap_caps_free(l.samples); l.samples = nullptr; }
  }
}

SensorIdentity* SensorRegistry::ensureIdentitiesScratchLocked() {
  if (!identitiesScratch_) {
#ifdef ESP_PLATFORM
    identitiesScratch_ = static_cast<SensorIdentity*>(heap_caps_malloc(
        sizeof(identities_), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!identitiesScratch_) {
      identitiesScratch_ = static_cast<SensorIdentity*>(
          heap_caps_malloc(sizeof(identities_), MALLOC_CAP_8BIT));
    }
#else
    identitiesScratch_ = new (std::nothrow) SensorIdentity[kMaxSensorIdentities];
#endif
  }
  return identitiesScratch_;
}

void SensorRegistry::resetForFactory() {
  if (!mutex_ || !storageMutex_) return;
  LockGuard storage(storageMutex_);
  LockGuard lock(mutex_);
  for (size_t i = 0; i < kMaxSensors; ++i) {
    ++generations_[i];
    configs_[i] = SensorConfig{};
    runtime_[i] = SensorRuntime{};
    if (live_[i].samples) { heap_caps_free(live_[i].samples); live_[i] = LiveHistory{}; }
    if (history_[i].samples) { heap_caps_free(history_[i].samples); history_[i] = StoredHistory{}; }
  }
  for (auto& id : identities_) id = SensorIdentity{};
  pagedConfigDirty_ = true;
  if (paged_) paged_->resetForFactory();
}

bool SensorRegistry::begin(ConfigStore& store, uint32_t defaultSleepSeconds, PagedHistory* paged) {
  store_ = &store;
  defaultSleepSeconds_ = constrain(defaultSleepSeconds, kMinSleepSeconds,
                                   kMaxSleepSeconds);
  mutex_ = xSemaphoreCreateMutex();
  storageMutex_ = xSemaphoreCreateMutex();
  if (mutex_ == nullptr || storageMutex_ == nullptr || !store_->loadSensorConfigs(configs_, kMaxSensors)) {
    return false;
  }
  if (!store_->loadSensorIdentities(identities_, kMaxSensorIdentities)) {
    for (auto& identity : identities_) identity = SensorIdentity{};
    Serial.printf("[Identity] Could not load remembered sensor identities\n");
  }
  bool identitiesChanged = false;
  for (const auto& config : configs_) if (config.occupied) {
    const int previous = identityIndexLocked(config.mac);
    if (previous < 0 || strcmp(identities_[previous].name, config.name)) {
      identitiesChanged |= rememberIdentityLocked(config.mac, config.name) >= 0;
    }
  }
  if (identitiesChanged && !store_->saveSensorIdentities(identities_, kMaxSensorIdentities)) return false;
  for (size_t i = 0; i < kMaxSensors; ++i) {
    if (configs_[i].occupied && !loadHistoryLocked(i) &&
        !paged && !configureHistoryLocked(i, configs_[i].sleepSeconds, false)) {
      Serial.printf("[History] Could not allocate history for sensor %u\n",
                    static_cast<unsigned>(i));
    }
    LatestTelemetry latest{};
    bool loaded = false;
    if (configs_[i].occupied) {
      LatestTelemetryV2 latestV2{};
      if (store_->loadLatestTelemetry(i, &latestV2, sizeof(latestV2)) &&
          latestV2.crc32 == lil::protocol::crc32(
                                reinterpret_cast<const uint8_t*>(&latestV2.base),
                                sizeof(latestV2.base))) {
        latest = latestV2.base;
        loaded = true;
      } else if (store_->loadLatestTelemetry(i, &latest, sizeof(latest))) {
        loaded = true;
      }
    }
    if (configs_[i].occupied && !loaded) {
      struct PreviousMotionFeedback {
        uint32_t magic;
        uint8_t version;
        uint8_t payload[lil::protocol::kMotionFeedbackTelemetryPayloadSize];
        int8_t rssi;
      };
      struct PreviousMotionFeedbackV2 {
        PreviousMotionFeedback base;
        uint32_t crc32;
      } previousV2{};
      PreviousMotionFeedback previous{};
      if (store_->loadLatestTelemetry(i, &previousV2, sizeof(previousV2)) &&
          previousV2.crc32 == lil::protocol::crc32(
                                  reinterpret_cast<const uint8_t*>(&previousV2.base),
                                  sizeof(previousV2.base))) {
        latest.magic = previousV2.base.magic;
        latest.protocolVersion = previousV2.base.version;
        memcpy(&latest.telemetry, previousV2.base.payload, sizeof(previousV2.base.payload));
        latest.stationRssi = previousV2.base.rssi;
        loaded = true;
      } else if (store_->loadLatestTelemetry(i, &previous, sizeof(previous))) {
        latest.magic = previous.magic;
        latest.protocolVersion = previous.version;
        memcpy(&latest.telemetry, previous.payload, sizeof(previous.payload));
        latest.stationRssi = previous.rssi;
        loaded = true;
      }
    }
    if (configs_[i].occupied && !loaded) {
      struct PreviousTimed {
        uint32_t magic;
        uint8_t version;
        uint8_t payload[lil::protocol::kTimedTelemetryPayloadSize];
        int8_t rssi;
      };
      struct PreviousTimedV2 {
        PreviousTimed base;
        uint32_t crc32;
      } previousV2{};
      PreviousTimed previous{};
      if (store_->loadLatestTelemetry(i, &previousV2, sizeof(previousV2)) &&
          previousV2.crc32 == lil::protocol::crc32(
                                  reinterpret_cast<const uint8_t*>(&previousV2.base),
                                  sizeof(previousV2.base))) {
        latest.magic = previousV2.base.magic;
        latest.protocolVersion = previousV2.base.version;
        memcpy(&latest.telemetry, previousV2.base.payload, sizeof(previousV2.base.payload));
        latest.stationRssi = previousV2.base.rssi;
        loaded = true;
      } else if (store_->loadLatestTelemetry(i, &previous, sizeof(previous))) {
        latest.magic = previous.magic;
        latest.protocolVersion = previous.version;
        memcpy(&latest.telemetry, previous.payload, sizeof(previous.payload));
        latest.stationRssi = previous.rssi;
        loaded = true;
      }
    }
    if (configs_[i].occupied && !loaded) {
      struct PreviousPrecision { uint32_t magic; uint8_t version;
        uint8_t payload[lil::protocol::kPrecisionTelemetryPayloadSize]; int8_t rssi; } previous{};
      if (store_->loadLatestTelemetry(i, &previous, sizeof(previous))) {
        latest.magic = previous.magic; latest.protocolVersion = previous.version;
        memcpy(&latest.telemetry, previous.payload, sizeof(previous.payload));
        latest.stationRssi = previous.rssi; loaded = true;
      }
    }
    if (configs_[i].occupied && !loaded) {
      struct PreviousLatest { uint32_t magic; uint8_t version;
        uint8_t payload[lil::protocol::kMotionTelemetryPayloadSize]; int8_t rssi; } previous{};
      if (store_->loadLatestTelemetry(i, &previous, sizeof(previous))) {
        latest.magic = previous.magic; latest.protocolVersion = previous.version;
        memcpy(&latest.telemetry, previous.payload, sizeof(previous.payload));
        latest.stationRssi = previous.rssi; loaded = true;
      }
    }
    if (loaded &&
        latest.magic == kLatestTelemetryMagic &&
        latest.protocolVersion == lil::protocol::kVersion &&
        lil::protocol::validTelemetryValues(latest.telemetry)) {
      runtime_[i].telemetry = latest.telemetry;
      runtime_[i].stationRssi = latest.stationRssi;
      runtime_[i].hasPersistedTelemetry = true;
    }
  }
  if (paged) {
    paged->configure(configs_, kMaxSensors);
    size_t legacyBytes = 0;
    for (size_t i=0;i<kMaxSensors;++i) if(configs_[i].occupied) legacyBytes += store_->historyBytesLength(i);
    paged->setLegacyBytes(legacyBytes);
    for (size_t i=0;i<kMaxSensors;++i) {
      auto& h=history_[i];
      if(h.samples){
        std::sort(h.samples,h.samples+h.capacity,[](const HistorySample& a,const HistorySample& b){return a.timestamp<b.timestamp;});
        bool ok=true;size_t imported=0;
        for(size_t row=0;row<h.capacity&&ok;++row)if(h.samples[row].timestamp){
          ok=paged->appendLegacy(configs_[i].mac,h.samples[row],h.bucketSeconds);++imported;
          if (row % 32 == 0) delay(1);
        }
        if(ok&&(!imported||paged->flush(configs_[i].mac))){
          const size_t bytes=store_->historyBytesLength(i);
          if(store_->deleteHistory(i)){legacyBytes=legacyBytes>bytes?legacyBytes-bytes:0;paged->setLegacyBytes(legacyBytes);}
        }
        heap_caps_free(h.samples);h={};
      }
    }
    paged_=paged;
  }
  return true;
}

#if defined(__GNUC__) && !defined(__clang__)
[[gnu::noinline]]
#endif
bool SensorRegistry::loadLegacyHistoryV2Locked(size_t index,
                                               uint32_t desiredBucketSeconds) {
  std::unique_ptr<LegacyStoredHistoryV2> legacy(new (std::nothrow) LegacyStoredHistoryV2());
  if (!legacy ||
      !store_->loadHistory(index, legacy.get(), sizeof(LegacyStoredHistoryV2)) ||
      legacy->magic != kHistoryMagic || legacy->version != 2 ||
      !configureHistoryLocked(index, desiredBucketSeconds, false, false)) {
    return false;
  }
  history_[index].revision = legacy->revision;
  for (const LegacyHistorySampleV2& source : legacy->samples) {
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
  if (!persistHistoryLocked(index)) {
    Serial.printf("[History] Could not migrate history for sensor %u\n",
                  static_cast<unsigned>(index));
  }
  return true;
}

#if defined(__GNUC__) && !defined(__clang__)
[[gnu::noinline]]
#endif
bool SensorRegistry::loadLegacyHistoryV3Locked(size_t index,
                                               uint32_t desiredBucketSeconds,
                                               size_t storedLength) {
  PersistedHistoryHeader header{};
  if (storedLength < sizeof(header) ||
      !store_->loadHistoryRange(index, 0, &header, sizeof(header)) ||
      header.magic != kHistoryMagic || header.version != 3 ||
      header.sampleSize != 23 || header.bucketSeconds < 30 ||
      header.bucketSeconds > kMaxSleepSeconds ||
      header.capacity != (kHistoryWindowSeconds + header.bucketSeconds - 1) / header.bucketSeconds ||
      storedLength != sizeof(header) + header.capacity * 23) {
    return false;
  }
  if (!configureHistoryLocked(index, desiredBucketSeconds, false, false)) return false;
  constexpr size_t kV3SampleBytes = 23;
  constexpr size_t kV3BatchSlots = 64;
  std::unique_ptr<uint8_t[]> v3Heap(new (std::nothrow) uint8_t[kV3BatchSlots * kV3SampleBytes]);
  uint8_t v3Stack[8 * kV3SampleBytes];
  uint8_t* const v3Batch = v3Heap ? v3Heap.get() : v3Stack;
  const size_t batchLimit = v3Heap ? kV3BatchSlots : 8;
  for (size_t slot = 0; slot < header.capacity;) {
    const size_t batch = min<size_t>(batchLimit, header.capacity - slot);
    if (!store_->loadHistoryRange(index, sizeof(header) + slot * kV3SampleBytes,
                                  v3Batch, batch * kV3SampleBytes)) {
      return false;
    }
    for (size_t j = 0; j < batch; ++j) {
      HistorySample sample{};
      memcpy(&sample, v3Batch + j * kV3SampleBytes, kV3SampleBytes);
      if (!sample.timestamp) continue;
      const size_t destination = (sample.timestamp / desiredBucketSeconds) % history_[index].capacity;
      if (sample.timestamp >= history_[index].samples[destination].timestamp)
        history_[index].samples[destination] = sample;
    }
    slot += batch;
  }
  history_[index].revision = header.revision;
  return persistHistoryLocked(index);
}

bool SensorRegistry::loadHistoryLocked(size_t index) {
  if (index >= kMaxSensors || !configs_[index].occupied) {
    return false;
  }
  if (lil::protocol::isLiveSensor(configs_[index].environmentalSensorType)) return true;
  if (configs_[index].sleepSeconds < 60)
    return configureHistoryLocked(index, configs_[index].sleepSeconds, false, false);
  const size_t storedLength = store_->historyBytesLength(index);
  const uint32_t desiredBucketSeconds =
      normalizedHistoryBucketSeconds(configs_[index].sleepSeconds);

  if (storedLength == sizeof(LegacyStoredHistoryV2)) {
    return loadLegacyHistoryV2Locked(index, desiredBucketSeconds);
  }
  if (loadLegacyHistoryV3Locked(index, desiredBucketSeconds, storedLength)) {
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
  if (index < kMaxSensors && history_[index].bucketSeconds < 60) return true;
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

int SensorRegistry::identityIndexLocked(const uint8_t mac[6]) const {
  for (size_t i = 0; i < kMaxSensorIdentities; ++i)
    if (identities_[i].name[0] && !memcmp(identities_[i].mac, mac, 6)) return static_cast<int>(i);
  return -1;
}

int SensorRegistry::rememberIdentityLocked(const uint8_t mac[6], const char* name,
                                          SensorIdentity* previous) {
  int index = identityIndexLocked(mac);
  bool replaceIdentity = false;
  if (index < 0) for (size_t i = 0; i < kMaxSensorIdentities; ++i) {
    if (!identities_[i].name[0]) { index = static_cast<int>(i); break; }
  }
  if (index < 0) for (size_t i = 0; i < kMaxSensorIdentities; ++i) {
    // Remembered names are a bounded convenience cache. Historical entries
    // must never prevent provisioning another sensor; active references stay.
    if (findIndexLocked(identities_[i].mac) < 0) {
      index = static_cast<int>(i);
      replaceIdentity = true;
      break;
    }
  }
  if (index < 0) return -1;
  if (previous) *previous = identities_[index];
  if (replaceIdentity) identities_[index] = SensorIdentity{};
  memcpy(identities_[index].mac, mac, 6);
  copyText(identities_[index].name, String(name));
  return index;
}

void SensorRegistry::commitConfigLocked(size_t index,
    const SensorConfig& previous, const SensorConfig& saved) {
  SensorConfig committed = saved;
  // Keep serving the previous settings until NVS succeeds. Preserve radio
  // acknowledgements of old commands received during that non-blocking save.
  committed.pendingFlags &= ~(previous.pendingFlags & ~configs_[index].pendingFlags);
  configs_[index] = committed;
  // Only re-persist if an in-flight radio ACK modified pendingFlags after NVS write.
  runtime_[index].configPersistencePending = committed.pendingFlags != saved.pendingFlags;
  pagedConfigDirty_ = true;
}

int SensorRegistry::allocateIndexLocked(const uint8_t mac[6]) {
  for (size_t i = 0; i < kMaxSensors; ++i) {
    if (!configs_[i].occupied) {
      configs_[i] = SensorConfig{};
      runtime_[i] = SensorRuntime{};
      configs_[i].occupied = true;
      memcpy(configs_[i].mac, mac, 6);
      configs_[i].sleepSeconds = defaultSleepSeconds_;
      configs_[i].revision = 1;
      snprintf(configs_[i].name, sizeof(configs_[i].name),
               "Sensor %02X:%02X:%02X", mac[3], mac[4], mac[5]);
      const int identity = identityIndexLocked(mac);
      if (identity >= 0) copyText(configs_[i].name, identities_[identity].name);
      ++generations_[i];
      runtime_[i].configPersistencePending = true;
      runtime_[i].storageInitializationPending = true;
      pagedConfigDirty_ = true;
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool SensorRegistry::registerTelemetry(
    const uint8_t mac[6], uint32_t sequence,
    const lil::protocol::TelemetryPayload& telemetry, int8_t rssi,
    SensorConfig& responseConfig, int8_t& txPowerQuarterDbm, bool& duplicate,
    uint32_t& generation) {
  if (!lil::protocol::validTelemetryValues(telemetry)) return false;
  LockGuard lock(mutex_);
  int index = findIndexLocked(mac);
  if (index < 0) {
    index = allocateIndexLocked(mac);
  }
  if (index < 0) {
    return false;
  }

  generation = generations_[index];
  SensorConfig& config = configs_[index];
  auto& current = runtime_[index];
  const bool discoveryMode =
      (telemetry.flags & lil::protocol::kDiscoveryBeacon) != 0;
  // ESP-NOW receive callbacks and the sensor's configuration response cross
  // asynchronously. A discovery packet assembled immediately before the
  // provisioning response can therefore arrive after the first configured
  // measurement. Reject a bounded trailing window of sequences within the same
  // bootCount so stale/duplicate frames are dropped without rejecting half of
  // all random sequence numbers after a cold boot (bootCount == 1).
  const uint32_t sequenceLag = current.lastSequence - sequence;
  const bool staleWithinSameBoot =
      current.hasSequence &&
      telemetry.bootCount == current.telemetry.bootCount &&
      sequenceLag < 1024U;
  if (staleWithinSameBoot) {
    duplicate = true;
    responseConfig = config;
    txPowerQuarterDbm = current.txPowerInitialized
                            ? current.sensorTxPowerQuarterDbm
                            : kDefaultTxPowerQuarterDbm;
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
      current.configPersistencePending = true;
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
    current.configPersistencePending = true;
    pagedConfigDirty_ = true;
  }

  const bool rawFallback =
      (telemetry.flags & lil::protocol::kBme680RawFallback) != 0;
  const bool previousIaqAvailable =
      current.hasTelemetry &&
      (current.telemetry.capabilities & lil::protocol::kIaq) != 0 &&
      current.telemetry.iaqAccuracy > 0;
  const float previousIaq = current.telemetry.iaq;
  const uint8_t previousIaqAccuracy = current.telemetry.iaqAccuracy;
  // Preserve the two cached IAQ scalars before replacing the runtime payload;
  // avoid a second full packed-payload copy through a stack temporary.
  current.telemetry = telemetry;
  if (rawFallback && previousIaqAvailable) {
    // A direct Bosch reading contains fresh T/H/P/gas values but no new BSEC
    // IAQ.  Keep the last real IAQ for the dashboard instead of replacing it
    // with the payload's default zero.  The raw-fallback flag remains set, so
    // callers can label the value stale and must not upload it as a new point.
    current.telemetry.capabilities |= lil::protocol::kIaq;
    current.telemetry.iaq = previousIaq;
    current.telemetry.iaqAccuracy = previousIaqAccuracy;
  }

  duplicate = false;
  current.hasTelemetry = true;
  current.stationRssi = rssi;
  const uint64_t monotonicMs = lil::timing::signedUsToMs(esp_timer_get_time());
  const uint64_t elapsedMonotonicMs =
      monotonicMs >= current.lastSeenMonotonicMs
          ? monotonicMs - current.lastSeenMonotonicMs
          : 0;
  current.receivedIntervalMs = current.receivedPackets ?
      static_cast<uint32_t>(min<uint64_t>(UINT32_MAX, elapsedMonotonicMs)) : 0;
  current.lastSeenMs = millis();
  current.lastSeenMonotonicMs = monotonicMs;
  current.receivedPackets++;
  const bool ordinaryPrecision = paged_ && usesNormalHistory(telemetry);
  if (lil::protocol::isLiveSensor(telemetry.sensorType) && !ordinaryPrecision && !discoveryMode &&
      !(telemetry.flags & lil::protocol::kBatteryProtectionActive)) {
    const bool timed = (telemetry.live.flags & lil::protocol::kLiveTimingKnown) != 0;
    const bool fresh = (telemetry.live.flags & lil::protocol::kLiveSampleFresh) != 0;
    const bool gap = telemetry.motion.fifoOverrun ||
        (telemetry.live.flags & lil::protocol::kLiveGap);
    const bool measurement = !(telemetry.flags & lil::protocol::kSensorReadFailed) &&
        (telemetry.capabilities &
        (lil::protocol::kTemperature | lil::protocol::kMotion | lil::protocol::kOptical)) != 0;
    // Idle contact packets retain the last card values, but do not fabricate
    // additional readings between configured normal measurement windows.
    const bool storeSample = gap || (measurement && (!timed || fresh));
    auto& live = live_[index];
    const size_t capacity = 60000 / lil::protocol::liveHistoryIntervalMs(telemetry.sensorType) + 1;
    if (live.samples && live.capacity != capacity) { heap_caps_free(live.samples); live = LiveHistory{}; }
    // A 32-bit sample ID can wrap after a long outage. Expire the old ring
    // before either a cached contact or a fresh sample can make it look recent.
    if (live.count && monotonicMs >= live.lastSampleMonotonicMs &&
        monotonicMs - live.lastSampleMonotonicMs > 60000) {
      live.head = live.count = 0;
    }
    live.capacity = capacity;
    // Only slots 0..count-1 are ever read; malloc avoids zeroing up to ~180 KiB
    // while holding the registry mutex.
    if (storeSample && !live.samples) live.samples = static_cast<LiveSample*>(heap_caps_malloc(
        capacity * sizeof(LiveSample), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (storeSample && !live.samples && heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) > capacity * sizeof(LiveSample) + 96U * 1024U)
      live.samples = static_cast<LiveSample*>(heap_caps_malloc(capacity * sizeof(LiveSample), MALLOC_CAP_8BIT));
    if (storeSample && live.samples) {
      live.samples[live.head] = {current.lastSeenMs, telemetry};
      live.lastSampleMonotonicMs = monotonicMs;
      if (++live.head == live.capacity) live.head = 0;
      if (live.count < live.capacity) ++live.count;
    }
  } else if (live_[index].samples) {
    heap_caps_free(live_[index].samples); live_[index] = LiveHistory{};
  }
  updateTxPowerLocked(static_cast<size_t>(index), rssi, telemetry.pcbVersion);
  current.lastSequence = sequence;
  current.hasSequence = true;
  if (!config.bme680QuickStartComplete) {
    // Compatibility with configurations created before ULP-only operation.
    // There is no longer a separate quick-start acknowledgement.
    config.bme680QuickStartComplete = true;
    current.configPersistencePending = true;
  }
  responseConfig = config;
  txPowerQuarterDbm = current.sensorTxPowerQuarterDbm;
  return true;
}

bool SensorRegistry::persistTelemetry(
    const uint8_t mac[6], uint32_t sequence,
    const lil::protocol::TelemetryPayload& telemetry, int8_t rssi,
    uint32_t generation, uint32_t receivedAt) {
  // Lock order is storage -> registry. Radio registration takes registry only.
  LockGuard storage(storageMutex_);
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0 || generations_[index] != generation) return false;
  const SensorRuntime snapshot = runtime_[index];
  const SensorConfig config = configs_[index];
  const bool needPagedConfigure = paged_ != nullptr && pagedConfigDirty_;
  if (needPagedConfigure) {
    memcpy(storageConfigScratch_, configs_, sizeof(storageConfigScratch_));
    pagedConfigDirty_ = false;
  }
  lock.unlock();
  bool success = true;
  bool initialized = !snapshot.storageInitializationPending;
  if (!initialized) initialized = store_->deleteHistory(index);
  bool configSaved = false;
  if (snapshot.configPersistencePending) {
    configSaved = store_->saveSensorConfig(index, config);
    success = configSaved;
  }
  const bool discovery = (telemetry.flags & lil::protocol::kDiscoveryBeacon) != 0;
  const bool batteryProtection = (telemetry.flags & lil::protocol::kBatteryProtectionActive) != 0;
  const bool precisionNormal = lil::timing::supportsNormalPrecisionMeasurements(telemetry.sensorType) &&
      telemetry.operatingMode == lil::protocol::SensorOperatingMode::kEnergySaving;
  const bool freshNormal = !(telemetry.live.flags & lil::protocol::kLiveTimingKnown) ||
      (telemetry.live.flags & (lil::protocol::kLiveSampleFresh|lil::protocol::kLiveGap));
  const bool live = lil::protocol::isLiveSensor(telemetry.sensorType) && !batteryProtection && (!paged_ || !usesNormalHistory(telemetry));
  bool latestSaved = false;
  if (snapshot.hasSequence && snapshot.lastSequence == sequence &&
      (!discovery || !snapshot.hasPersistedTelemetry) && (batteryProtection || config.sleepSeconds >= 60) &&
      !live && (!precisionNormal || freshNormal)) {
    latestSaved = saveLatestTelemetryLocked(index, snapshot.telemetry, rssi);
    success = latestSaved && success;
  }
  if (needPagedConfigure) {
    paged_->configure(storageConfigScratch_, kMaxSensors);
  }
  // Pre-NTP packets carry receivedAt == 0 and cannot be indexed in UTC history;
  // skip history append cleanly without flagging a dropped packet.
  if (!discovery && initialized && !live && receivedAt >= static_cast<uint32_t>(kMinimumValidTime)) {
    if (paged_) {
      const bool supportedByPaged =
          config.provisioned &&
          config.environmentalSensorType != lil::protocol::EnvironmentalSensorType::kMax30102 &&
          telemetry.sensorType != lil::protocol::EnvironmentalSensorType::kMax30102;
      if (supportedByPaged && (!precisionNormal || freshNormal)) {
        success = paged_->append(mac, receivedAt, telemetry,
            batteryProtection ? 86400 : config.sleepSeconds) && success;
      }
    } else {
      success = recordHistoryLocked(index, telemetry,
                    batteryProtection && config.sleepSeconds < 60 ? 86400 : config.sleepSeconds,
                    receivedAt) && success;
    }
  }
  lock.relock();
  // A new radio packet can change pending commands while the flash is busy.
  if (configSaved && sensorConfigEquals(config, configs_[index])) {
    runtime_[index].configPersistencePending = false;
  }
  runtime_[index].storageInitializationPending = !initialized;
  runtime_[index].hasPersistedTelemetry |= latestSaved;
  return success && initialized;
}

size_t SensorRegistry::liveHistory(const uint8_t mac[6], LiveSample* output,
                                   size_t capacity, const uint32_t* afterMs) const {
  if (!output) return 0;
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0 || !live_[index].samples) return 0;
  const auto& live = live_[index];
  const uint64_t monotonicMs = lil::timing::signedUsToMs(esp_timer_get_time());
  if (monotonicMs >= live.lastSampleMonotonicMs &&
      monotonicMs - live.lastSampleMonotonicMs > 60000) return 0;
  size_t count = 0;
  const uint32_t now = millis();
  const auto selected = [now, afterMs](uint32_t receivedMs) {
    const uint32_t sampleAge = now - receivedMs;
    if (sampleAge > 60000U) return false;
    if (!afterMs) return true;
    const uint32_t cursorAge = now - *afterMs;
    return sampleAge < cursorAge;
  };
  size_t position = live.head >= live.count ? live.head - live.count :
      live.capacity - (live.count - live.head);
  for (size_t scanned = 0; scanned < live.count && count < capacity;) {
    if (!selected(live.samples[position].receivedMs)) {
      ++scanned;
      if (++position == live.capacity) position = 0;
      continue;
    }
    // Check every timestamp, then copy each physically contiguous fresh run.
    // A full fresh ring needs at most two libc copies and no per-row division.
    const size_t first = position;
    size_t run = 0;
    do { ++run; ++position; ++scanned; }
    while (position < live.capacity && scanned < live.count && count + run < capacity &&
           selected(live.samples[position].receivedMs));
    memcpy(output + count, live.samples + first, run * sizeof(LiveSample));
    count += run;
    if (position == live.capacity) position = 0;
  }
  return count;
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
      const int identity = identityIndexLocked(configs_[i].mac);
      output[written].motionReference = identity >= 0 ? identities_[identity].motion : MotionReference{};
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
  const bool validType = lil::protocol::isValidSensorType(sensorType);
  if (!validType || !name.length() || !isfinite(temperatureOffsetC) ||
      !isfinite(batteryCalibrationFactor)) return false;
  uint16_t assigned = 0;
  for (const uint8_t field : {fields.temperature, fields.humidity, fields.pressure,
                            fields.iaq, fields.battery, fields.gasResistance}) {
    if (field > 8 || (field && (assigned & (1U << field)))) return false;
    if (field) assigned |= 1U << field;
  }
  LockGuard storage(storageMutex_);
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return false;
  }
  SensorConfig config = configs_[index];
  const SensorConfig previousConfig = config;
  const bool wasProvisioned = config.provisioned;
  const bool sensorTypeChanged =
      config.environmentalSensorType != sensorType;
  SensorIdentity previousIdentity{};
  const int identity = rememberIdentityLocked(mac, name.c_str(), &previousIdentity);
  if (identity < 0) return false;
  copyText(config.name, name);
  const uint32_t newSleepSeconds =
      constrain(sleepSeconds, kMinSleepSeconds, kMaxSleepSeconds);
  config.sleepSeconds = newSleepSeconds;
  const bool channelChanged = config.thingSpeakChannelId != channelId;
  config.thingSpeakChannelId = channelId;
  if (writeKey.length() > 0) {
    copyText(config.thingSpeakWriteKey, writeKey);
  } else if (channelChanged || channelId == 0) {
    config.thingSpeakWriteKey[0] = '\0';
  }
  config.cloudUploadEnabled = uploadEnabled && !lil::protocol::isLiveSensor(sensorType);
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
  SensorIdentity* identities = ensureIdentitiesScratchLocked();
  if (!identities) {
    identities_[identity] = previousIdentity;
    return false;
  }
  memcpy(identities, identities_, sizeof(identities_));
  identities_[identity] = previousIdentity;
  const SensorConfig snapshot = config;
  lock.unlock();
  const bool configSaved = store_->saveSensorConfig(index, snapshot);
  const bool identitySaved = configSaved && store_->saveSensorIdentities(identities, kMaxSensorIdentities);
  if (!configSaved || !identitySaved) {
    lock.relock();
    const SensorConfig restored = configs_[index];
    runtime_[index].configPersistencePending = true;
    lock.unlock();
    // An identity-write error may follow a successful config write. Restore
    // the durable config as well; pending state retries if storage stays full.
    store_->saveSensorConfig(index, restored);
    return false;
  }
  lock.relock();
  commitConfigLocked(index, previousConfig, snapshot);
  identities_[identity] = identities[identity];
  if (sensorTypeChanged) {
    if (live_[index].samples) heap_caps_free(live_[index].samples);
    live_[index] = LiveHistory{};
    runtime_[index].hasRecordingStatus = false;
  }
  if (paged_) {
    memcpy(storageConfigScratch_, configs_, sizeof(storageConfigScratch_));
    pagedConfigDirty_ = false;
  }
  lock.unlock();
  if (paged_) {
    // Typed pages keep earlier measurements available after a sensor change.
    // A settings acknowledgement depends only on the durable settings writes.
    paged_->configure(storageConfigScratch_, kMaxSensors);
    return true;
  }
  if (lil::protocol::isLiveSensor(snapshot.environmentalSensorType)) {
    if (history_[index].samples) heap_caps_free(history_[index].samples);
    history_[index] = StoredHistory{};
  } else if (history_[index].bucketSeconds != snapshot.sleepSeconds) {
    configureHistoryLocked(index, snapshot.sleepSeconds, true);
  }
  return true;
}

bool SensorRegistry::setThingSpeakChannel(const uint8_t mac[6],
                                           uint32_t channelId,
                                           const String& writeKey,
                                           uint8_t thingSpeakProfileSlot) {
  LockGuard storage(storageMutex_);
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return false;
  }
  SensorConfig config = configs_[index];
  const SensorConfig previousConfig = config;
  const bool wasProvisioned = config.provisioned;
  config.thingSpeakChannelId = channelId;
  copyText(config.thingSpeakWriteKey, writeKey);
  config.cloudUploadEnabled = channelId != 0 && writeKey.length() > 0 &&
      !lil::protocol::isLiveSensor(config.environmentalSensorType);
  config.thingSpeakProfileSlot = thingSpeakProfileSlot;
  config.provisioned = true;
  if (!wasProvisioned) {
    config.pendingFlags |= kAwaitingProvisioningAck;
    ++config.revision;
    if (config.revision == 0) {
      config.revision = 1;
    }
  }
  const SensorConfig snapshot = config;
  lock.unlock();
  const bool saved = store_->saveSensorConfig(index, snapshot);
  if (saved) {
    lock.relock();
    commitConfigLocked(index, previousConfig, snapshot);
    if (paged_) {
      memcpy(storageConfigScratch_, configs_, sizeof(storageConfigScratch_));
      pagedConfigDirty_ = false;
    }
    lock.unlock();
    if (paged_) paged_->configure(storageConfigScratch_, kMaxSensors);
  }
  return saved;
}

bool SensorRegistry::syncThingSpeakProfile(
    uint8_t slot, const ThingSpeakChannelProfile& profile) {
  if (slot >= kMaxThingSpeakChannels || !profile.occupied) {
    return false;
  }
  LockGuard storage(storageMutex_);
  LockGuard lock(mutex_);
  bool success = true;
  // Reuse storageConfigScratch_ under storageMutex_ instead of placing a
  // 2 KiB SensorConfig[19] array on the 8 KiB loopTask stack.
  memcpy(storageConfigScratch_, configs_, sizeof(storageConfigScratch_));
  bool changed[kMaxSensors]{};
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
    runtime_[i].configPersistencePending = true;
    changed[i] = true;
    const SensorConfig snapshot = configs_[i];
    lock.unlock();
    const bool saved = store_->saveSensorConfig(i, snapshot);
    success = saved && success;
    lock.relock();
    if (saved && sensorConfigEquals(snapshot, configs_[i])) {
      runtime_[i].configPersistencePending = false;
    }
  }
  if (!success) {
    for (size_t i = 0; i < kMaxSensors; ++i) if (changed[i]) {
      configs_[i].thingSpeakChannelId = storageConfigScratch_[i].thingSpeakChannelId;
      copyText(configs_[i].thingSpeakWriteKey, storageConfigScratch_[i].thingSpeakWriteKey);
      configs_[i].cloudUploadEnabled = storageConfigScratch_[i].cloudUploadEnabled;
      const SensorConfig restored = configs_[i];
      lock.unlock();
      // Preserve command acknowledgements which arrived while flash was busy.
      const bool restoredSaved = store_->saveSensorConfig(i, restored);
      lock.relock();
      if (restoredSaved && sensorConfigEquals(restored, configs_[i])) {
        runtime_[i].configPersistencePending = false;
      }
    }
  }
  return success;
}

bool SensorRegistry::removeThingSpeakProfile(uint8_t slot) {
  if (slot >= kMaxThingSpeakChannels) {
    return false;
  }
  LockGuard storage(storageMutex_);
  LockGuard lock(mutex_);
  bool success = true;
  memcpy(storageConfigScratch_, configs_, sizeof(storageConfigScratch_));
  bool changed[kMaxSensors]{};
  for (size_t i = 0; i < kMaxSensors; ++i) {
    if (!configs_[i].occupied ||
        configs_[i].thingSpeakProfileSlot != slot) {
      continue;
    }
    configs_[i].thingSpeakProfileSlot = 0xFF;
    configs_[i].thingSpeakChannelId = 0;
    configs_[i].thingSpeakWriteKey[0] = '\0';
    configs_[i].cloudUploadEnabled = false;
    runtime_[i].configPersistencePending = true;
    changed[i] = true;
    const SensorConfig snapshot = configs_[i];
    lock.unlock();
    const bool saved = store_->saveSensorConfig(i, snapshot);
    success = saved && success;
    lock.relock();
    if (saved && sensorConfigEquals(snapshot, configs_[i])) {
      runtime_[i].configPersistencePending = false;
    }
  }
  if (!success) {
    for (size_t i = 0; i < kMaxSensors; ++i) if (changed[i]) {
      configs_[i].thingSpeakProfileSlot = storageConfigScratch_[i].thingSpeakProfileSlot;
      configs_[i].thingSpeakChannelId = storageConfigScratch_[i].thingSpeakChannelId;
      copyText(configs_[i].thingSpeakWriteKey, storageConfigScratch_[i].thingSpeakWriteKey);
      configs_[i].cloudUploadEnabled = storageConfigScratch_[i].cloudUploadEnabled;
      const SensorConfig restored = configs_[i];
      lock.unlock();
      const bool restoredSaved = store_->saveSensorConfig(i, restored);
      lock.relock();
      if (restoredSaved && sensorConfigEquals(restored, configs_[i])) {
        runtime_[i].configPersistencePending = false;
      }
    }
  }
  return success;
}

bool SensorRegistry::requestFactoryReset(const uint8_t mac[6]) {
  LockGuard storage(storageMutex_);
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return false;
  }
  SensorConfig config = configs_[index];
  const SensorConfig previousConfig = config;
  config.provisioned = false;
  config.pendingFlags &= ~kAwaitingProvisioningAck;
  config.pendingFlags |= lil::protocol::kFactoryReset;
  ++config.revision;
  if (config.revision == 0) {
    config.revision = 1;
  }
  const SensorConfig snapshot = config;
  lock.unlock();
  const bool saved = store_->saveSensorConfig(index, snapshot);
  if (saved) {
    lock.relock();
    commitConfigLocked(index, previousConfig, snapshot);
    if (paged_) {
      memcpy(storageConfigScratch_, configs_, sizeof(storageConfigScratch_));
      pagedConfigDirty_ = false;
    }
    lock.unlock();
    if (paged_) paged_->configure(storageConfigScratch_, kMaxSensors);
  }
  return saved;
}

bool SensorRegistry::requestIaqCalibrationReset(const uint8_t mac[6]) {
  LockGuard storage(storageMutex_);
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return false;
  }
  const SensorConfig previousConfig = configs_[index];
  SensorConfig config = previousConfig;
  config.pendingFlags |= lil::protocol::kResetIaqCalibration;
  config.bme680QuickStartComplete = true;
  ++config.revision;
  if (config.revision == 0) {
    config.revision = 1;
  }
  const SensorConfig snapshot = config;
  lock.unlock();
  const bool saved = store_->saveSensorConfig(index, snapshot);
  if (saved) { lock.relock(); commitConfigLocked(index, previousConfig, snapshot); }
  return saved;
}

bool SensorRegistry::deleteSensor(const uint8_t mac[6]) {
  LockGuard storage(storageMutex_);
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) return false;
  lock.unlock();
  if(paged_&&!paged_->clear(mac))return false;
  // Keep the durable identity until its old slot files are removed. A failed
  // filesystem operation must not let a reboot reuse those files for a node.
  if (!store_->deleteHistory(index) ||
      !store_->deleteLatestTelemetry(index) ||
      !store_->deleteSensorConfig(index)) {
    return false;
  }
  lock.relock();
  ++generations_[index];
  if (live_[index].samples) heap_caps_free(live_[index].samples);
  live_[index] = LiveHistory{};
  configs_[index] = SensorConfig{};
  runtime_[index] = SensorRuntime{};
  lock.unlock();
  if (history_[index].samples != nullptr) {
    heap_caps_free(history_[index].samples);
  }
  history_[index] = StoredHistory{};
  if(paged_){
    lock.relock();
    memcpy(storageConfigScratch_, configs_, sizeof(storageConfigScratch_));
    pagedConfigDirty_ = false;
    lock.unlock();
    paged_->configure(storageConfigScratch_, kMaxSensors);
  }
  return true;
}

bool SensorRegistry::setMotionReference(const uint8_t mac[6], bool clear) {
  LockGuard storage(storageMutex_);
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) return false;
  MotionReference reference{};
  if (!clear) {
    if (configs_[index].environmentalSensorType != lil::protocol::EnvironmentalSensorType::kAutoDetect &&
        configs_[index].environmentalSensorType != lil::protocol::EnvironmentalSensorType::kLsm6dsox) return false;
    const auto& current = runtime_[index];
    const auto& t = current.telemetry;
    const uint64_t nowMonotonic = lil::timing::signedUsToMs(esp_timer_get_time());
    const uint64_t age = nowMonotonic >= current.lastSeenMonotonicMs
                             ? nowMonotonic - current.lastSeenMonotonicMs
                             : 0;
    const uint32_t limit = configs_[index].sleepSeconds * 1000UL + 2000UL;
    if (!current.hasTelemetry || t.sensorType != lil::protocol::EnvironmentalSensorType::kLsm6dsox ||
        !(t.capabilities & lil::protocol::kMotion) || (t.flags & (lil::protocol::kSensorReadFailed | lil::protocol::kBatteryProtectionActive)) ||
        t.motion.fifoOverrun || (t.live.flags & lil::protocol::kLiveGap) || age > limit ||
        ((t.live.flags & lil::protocol::kLiveTimingKnown) &&
         (t.live.acquisitionAgeMs == UINT16_MAX || uint64_t(t.live.acquisitionAgeMs) + age > limit))) return false;
    memcpy(reference.accelerationG, t.motion.accelerationG, sizeof(reference.accelerationG));
    memcpy(reference.angularRateDps, t.motion.angularRateDps, sizeof(reference.angularRateDps));
    if (!validMotionReference(reference)) return false;
    reference.enabled = true;
  }
  SensorIdentity previousIdentity{};
  const int identity = rememberIdentityLocked(mac, configs_[index].name, &previousIdentity);
  if (identity < 0) return false;
  SensorIdentity* snapshot = ensureIdentitiesScratchLocked();
  if (!snapshot) {
    identities_[identity] = previousIdentity;
    return false;
  }
  memcpy(snapshot, identities_, sizeof(identities_));
  snapshot[identity].motion = reference;
  identities_[identity] = previousIdentity;
  lock.unlock();
  const bool saved = store_->saveSensorIdentities(snapshot, kMaxSensorIdentities);
  if (saved) { lock.relock(); identities_[identity] = snapshot[identity]; }
  return saved;
}

bool SensorRegistry::saveLatestTelemetryLocked(
    size_t index, const lil::protocol::TelemetryPayload& telemetry,
    int8_t stationRssi) {
  LatestTelemetryV2 latest{};
  latest.base.magic = kLatestTelemetryMagic;
  latest.base.protocolVersion = lil::protocol::kVersion;
  latest.base.telemetry = telemetry;
  latest.base.stationRssi = stationRssi;
  latest.crc32 = lil::protocol::crc32(
      reinterpret_cast<const uint8_t*>(&latest.base), sizeof(latest.base));
  return store_->saveLatestTelemetry(index, &latest, sizeof(latest));
}

bool SensorRegistry::recordHistoryLocked(
    size_t index, const lil::protocol::TelemetryPayload& telemetry,
    uint32_t bucketSeconds, uint32_t receivedAt) {
  const time_t now = receivedAt;
  if (index >= kMaxSensors || now < kMinimumValidTime) {
    return false;
  }
  const uint32_t desiredBucketSeconds =
      normalizedHistoryBucketSeconds(bucketSeconds);
  if ((history_[index].samples == nullptr ||
       history_[index].bucketSeconds != desiredBucketSeconds) &&
      !configureHistoryLocked(index, desiredBucketSeconds, true)) {
    return false;
  }
  const uint32_t bucketIndex =
      static_cast<uint32_t>(now) / history_[index].bucketSeconds;
  const size_t slot = bucketIndex % history_[index].capacity;
  HistorySample& sample = history_[index].samples[slot];
  // A bucket contains the newest received measurement in one configured
  // sensor interval. The ring size changes with that interval while its time
  // span remains 24 hours.
  if (sample.timestamp > static_cast<uint32_t>(now)) return true;
  sample = HistorySample{};
  // Buckets choose storage slots; the point retains its real reception time.
  // Rounding the timestamp backwards can hide a recent reading in short views.
  sample.timestamp = static_cast<uint32_t>(now);
  sample.capabilities = telemetry.capabilities;
  if ((telemetry.flags & lil::protocol::kBme680RawFallback) != 0) {
    sample.capabilities &= ~lil::protocol::kIaq;
  }
  if ((telemetry.flags & lil::protocol::kSensorReadFailed) != 0)
    sample.capabilities &= lil::protocol::kBattery;
  if (telemetry.flags & lil::protocol::kBatteryReadFailed)
    sample.capabilities &= ~lil::protocol::kBattery;
  if (sample.capabilities & lil::protocol::kMotion) {
    for (size_t axis = 0; axis < 3; ++axis) {
      sample.accelerationMilliG[axis] = static_cast<int16_t>(lroundf(constrain(telemetry.motion.accelerationG[axis], -32.0F, 32.0F) * 1000));
      sample.angularRateDeciDps[axis] = static_cast<int16_t>(lroundf(constrain(telemetry.motion.angularRateDps[axis], -3000.0F, 3000.0F) * 10));
    }
  }
  sample.peakAccelerationMilliG = encodeUnsigned(telemetry.motion.peakAccelerationG, 1000.0F);
  sample.peakAngularRateDeciDps = encodeUnsigned(telemetry.motion.peakAngularRateDps, 10.0F);
  sample.motionOverrun = telemetry.motion.fifoOverrun;
  sample.temperatureCentiC = encodeSignedHundredths(telemetry.temperatureC);
  sample.humidityCentiPercent =
      encodeUnsigned(telemetry.humidityPercent, 100.0F);
  sample.pressureDeciHpa = encodeUnsigned(telemetry.pressureHpa, 10.0F);
  sample.iaqDeci = encodeUnsigned(telemetry.iaq, 10.0F);
  sample.gasResistanceOhms = encodeGasResistance(telemetry.gasResistanceOhms);
  sample.batteryMillivolts = telemetry.batteryMillivolts;
  sample.iaqAccuracy = telemetry.iaqAccuracy;
  sample.sensorType = static_cast<uint8_t>(telemetry.sensorType);
  sample.pcbVersion = telemetry.pcbVersion;
  ++history_[index].revision;
  if (history_[index].revision == 0) {
    history_[index].revision = 1;
  }
  return desiredBucketSeconds < 60 || persistHistorySampleLocked(index, slot);
}

size_t SensorRegistry::history(const uint8_t mac[6], HistorySample* output,
                               size_t capacity) const {
  if (output == nullptr || capacity == 0) {
    return 0;
  }
  LockGuard storage(storageMutex_);
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return 0;
  }
  lock.unlock();
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
  storage.unlock();
  std::sort(output, output + count,
            [](const HistorySample& first, const HistorySample& second) {
              return first.timestamp < second.timestamp;
            });
  return count;
}

size_t SensorRegistry::historyCapacity(const uint8_t mac[6]) const {
  if(paged_)return paged_->info(mac).capacityPoints;
  LockGuard storage(storageMutex_);
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  return index < 0 ? 0 : history_[index].capacity;
}

uint32_t SensorRegistry::historyBucketSeconds(const uint8_t mac[6]) const {
  if(paged_)return paged_->info(mac).intervalSeconds;
  LockGuard storage(storageMutex_);
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
  if(paged_)return paged_->info(mac).revision;
  LockGuard storage(storageMutex_);
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return 0;
  }
  return history_[index].revision;
}

bool SensorRegistry::channelShared(uint32_t channelId) const {
  LockGuard lock(mutex_);
  unsigned count = 0;
  for (const auto& config : configs_) {
    if (config.occupied && config.provisioned && config.cloudUploadEnabled &&
        config.thingSpeakChannelId == channelId && ++count > 1) return true;
  }
  return false;
}

bool SensorRegistry::generationMatches(const uint8_t mac[6], uint32_t generation) const {
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  return index >= 0 && generations_[index] == generation;
}

bool SensorRegistry::findConfig(const uint8_t mac[6], SensorConfig& output,
                               uint32_t* generation) const {
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return false;
  }
  output = configs_[index];
  if (generation) *generation = generations_[index];
  return true;
}

bool SensorRegistry::cloudUploadMatches(
    const uint8_t mac[6], uint32_t generation, uint32_t channelId,
    const char* writeKey, const ThingSpeakFieldMapping& fields) const {
  if (!writeKey) return false;
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0 || generations_[index] != generation) return false;
  const auto& config = configs_[index];
  return config.provisioned && config.cloudUploadEnabled &&
      !lil::protocol::isLiveSensor(config.environmentalSensorType) &&
      config.thingSpeakChannelId == channelId &&
      !strcmp(config.thingSpeakWriteKey, writeKey) &&
      !memcmp(&config.thingSpeakFields, &fields, sizeof(fields));
}

bool SensorRegistry::findView(const uint8_t mac[6], SensorView& output) const {
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) {
    return false;
  }
  output.config = configs_[index];
  output.runtime = runtime_[index];
  const int identity = identityIndexLocked(mac);
  output.motionReference = identity >= 0 ? identities_[identity].motion : MotionReference{};
  return true;
}

void SensorRegistry::updateRecording(const uint8_t mac[6], const lil::recording::Status& status) {
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  if (index < 0) return;
  if (status.session && (!runtime_[index].hasRecordingStatus ||
      runtime_[index].recording.session != status.session)) {
    live_[index].head = live_[index].count = 0;
  }
  runtime_[index].recording = status;
  runtime_[index].hasRecordingStatus = true;
}
bool SensorRegistry::needsPersistence(const uint8_t mac[6]) const {
  LockGuard lock(mutex_);
  const int index = findIndexLocked(mac);
  return index >= 0 && (runtime_[index].configPersistencePending || runtime_[index].storageInitializationPending);
}

String SensorRegistry::formatMac(const uint8_t mac[6]) {
  char text[18];
  snprintf(text, sizeof(text), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0],
           mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(text);
}

bool SensorRegistry::parseMac(const String& text, uint8_t output[6]) {
  if (!output) return false;
  uint8_t parsed[6]{};
  const char* cursor = text.c_str();
  for (size_t i = 0; i < 6; ++i) {
    unsigned value = 0, digits = 0;
    while (digits < 2) {
      const char c = *cursor;
      const int digit = c >= '0' && c <= '9' ? c - '0' :
                        c >= 'a' && c <= 'f' ? c - 'a' + 10 :
                        c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
      if (digit < 0) break;
      value = value * 16 + static_cast<unsigned>(digit);
      ++digits; ++cursor;
    }
    if (!digits || (i < 5 ? *cursor != ':' : *cursor != '\0')) return false;
    parsed[i] = static_cast<uint8_t>(value);
    if (i < 5) ++cursor;
  }
  if (static_cast<size_t>(cursor - text.c_str()) != text.length()) return false;
  memcpy(output, parsed, sizeof(parsed));
  return true;
}

}  // namespace station

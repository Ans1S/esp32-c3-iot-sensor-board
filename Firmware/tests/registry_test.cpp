#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif
#include <assert.h>
#include <ctime>
#include <functional>
#include <vector>
#include "sensor_registry.h"
#include "esp_heap_caps.h"

namespace {
std::function<void()> storageHook;
unsigned writes = 0;
std::vector<uint8_t> histories[station::kMaxSensors];
station::SensorConfig saved[station::kMaxSensors];
void onStorage() { ++writes; auto hook = storageHook; if (hook) hook(); }
}

namespace station {
bool ConfigStore::loadSensorConfigs(SensorConfig* out, size_t count) {
  for (size_t i = 0; i < count; ++i) out[i] = saved[i];
  return true;
}
bool ConfigStore::saveSensorConfig(size_t index, const SensorConfig& config) {
  onStorage(); saved[index] = config; return true;
}
bool ConfigStore::deleteSensorConfig(size_t index) {
  onStorage(); saved[index] = SensorConfig{}; return true;
}
size_t ConfigStore::historyBytesLength(size_t index) { return histories[index].size(); }
bool ConfigStore::loadHistory(size_t index, void* out, size_t length) {
  return loadHistoryRange(index, 0, out, length);
}
bool ConfigStore::loadHistoryRange(size_t index, size_t offset, void* out, size_t length) {
  if (offset + length > histories[index].size()) return false;
  memcpy(out, histories[index].data() + offset, length); return true;
}
bool ConfigStore::saveHistoryParts(size_t index, const void* header, size_t hsize,
                                  const void* samples, size_t ssize) {
  onStorage(); histories[index].resize(hsize + ssize);
  memcpy(histories[index].data(), header, hsize);
  memcpy(histories[index].data() + hsize, samples, ssize); return true;
}
bool ConfigStore::updateHistory(size_t index, size_t expected, const void* header,
                               size_t hsize, size_t offset, const void* sample,
                               size_t ssize) {
  onStorage();
  if (histories[index].size() != expected) return false;
  memcpy(histories[index].data(), header, hsize);
  memcpy(histories[index].data() + offset, sample, ssize); return true;
}
bool ConfigStore::deleteHistory(size_t index) {
  onStorage(); histories[index].clear(); return true;
}
bool ConfigStore::loadLatestTelemetry(size_t, void*, size_t) { return false; }
bool ConfigStore::saveLatestTelemetry(size_t, const void*, size_t) { onStorage(); return true; }
bool ConfigStore::deleteLatestTelemetry(size_t) { onStorage(); return true; }
}

int main() {
  station::ConfigStore store;
  station::SensorRegistry registry;
  assert(registry.begin(store, 600));
  uint8_t mac[6] = {2, 3, 4, 5, 6, 7};
  lil::protocol::TelemetryPayload telemetry{};
  telemetry.bootCount = 1;
  telemetry.capabilities = lil::protocol::kTemperature | lil::protocol::kIaq;
  telemetry.temperatureC = 21.5F;
  telemetry.iaq = 42;
  telemetry.iaqAccuracy = 2;
  station::SensorConfig response{};
  int8_t txPower;
  bool duplicate;
  uint32_t generation;
  assert(registry.registerTelemetry(mac, 1, telemetry, -60, response,
                                     txPower, duplicate, generation));
  assert(!duplicate && writes == 0 && testAllocations == 0);
  const uint32_t initialGeneration = generation;

  // Every flash operation injects a radio registration. If the registry mutex
  // is still held, the non-blocking test mutex detects the original deadlock.
  unsigned injected = 0;
  storageHook = [&] {
    uint32_t observed;
    assert(registry.registerTelemetry(mac, 1, telemetry, -60, response,
                                       txPower, duplicate, observed));
    assert(duplicate);
    ++injected;
  };
  const uint32_t received = static_cast<uint32_t>(time(nullptr)) - 1200;
  assert(registry.persistTelemetry(mac, 1, telemetry, -60, generation, received));
  assert(injected > 0 && testAllocations == 1);
  station::HistorySample samples[200]{};
  assert(registry.history(mac, samples, 200) == 1);
  assert(samples[0].timestamp == received / 600 * 600);
  assert(samples[0].temperatureCentiC == 2150);

  assert(registry.updateConfig(mac, "Test", 30, 0, "", false, {}, 0xff,
             lil::protocol::EnvironmentalSensorType::kBme680, 0.4F, 1.0F));
  assert(registry.historyCapacity(mac) == 900);
  assert(registry.requestIaqCalibrationReset(mac));
  assert(registry.requestFactoryReset(mac));
  assert(registry.setThingSpeakChannel(mac, 123, "key", 0));
  station::ThingSpeakChannelProfile profile{};
  profile.occupied = true; profile.channelId = 456;
  assert(registry.syncThingSpeakProfile(0, profile));
  assert(registry.removeThingSpeakProfile(0));

  // A command acknowledged during a write must stay pending for another save.
  storageHook = [&] {
    storageHook = nullptr;
    station::SensorConfig current;
    assert(registry.findConfig(mac, current));
    telemetry.appliedConfigRevision = current.revision;
    assert(registry.registerTelemetry(mac, 2, telemetry, -60, response,
                                       txPower, duplicate, generation));
  };
  assert(registry.persistTelemetry(mac, 1, telemetry, -60, generation, received));
  station::SensorView view;
  assert(registry.findView(mac, view));
  assert(view.runtime.configPersistencePending);
  assert(registry.persistTelemetry(mac, 2, telemetry, -60, generation, received));
  assert(registry.findView(mac, view) && !view.runtime.configPersistencePending);
  assert(saved[0].pendingFlags == view.config.pendingFlags);

  telemetry.flags = lil::protocol::kBme680RawFallback;
  assert(registry.registerTelemetry(mac, 3, telemetry, -60, response,
                                     txPower, duplicate, generation));
  assert(registry.persistTelemetry(mac, 3, telemetry, -60, generation, received + 60));
  size_t count = registry.history(mac, samples, 200);
  assert(count > 0 && (samples[count - 1].capabilities & lil::protocol::kIaq) == 0);
  storageHook = nullptr;
  assert(registry.updateConfig(mac, "Motion", 1, 0, "", false, {}, 0xff,
             lil::protocol::EnvironmentalSensorType::kLsm6dsox, 0, 1));
  telemetry.flags = 0;
  telemetry.sensorType = lil::protocol::EnvironmentalSensorType::kLsm6dsox;
  telemetry.capabilities = lil::protocol::kMotion;
  telemetry.motion.accelerationG[0] = -1.234F;
  telemetry.motion.angularRateDps[1] = -123.4F;
  telemetry.motion.peakAccelerationG = 2.5F;
  telemetry.motion.fifoOverrun = 1;
  assert(registry.registerTelemetry(mac, 4, telemetry, -60, response, txPower, duplicate, generation));
  assert(registry.persistTelemetry(mac, 4, telemetry, -60, generation, received));
  const unsigned fastWrites = writes;
  for (uint32_t i = 1; i <= 901; ++i) {
    assert(registry.registerTelemetry(mac, 4 + i, telemetry, -60, response, txPower, duplicate, generation));
    assert(registry.persistTelemetry(mac, 4 + i, telemetry, -60, generation, received + i));
  }
  assert(writes == fastWrites); // No per-report flash writes in fast mode.
  station::HistorySample motionSamples[900]{};
  assert(registry.history(mac, motionSamples, 900) == 900);
  assert(motionSamples[0].timestamp == received + 2);
  assert(motionSamples[899].timestamp == received + 901);
  assert(motionSamples[899].accelerationMilliG[0] == -1234);
  assert(motionSamples[899].angularRateDeciDps[1] == -1234);
  assert(motionSamples[899].peakAccelerationMilliG == 2500 && motionSamples[899].motionOverrun);
  assert(registry.deleteSensor(mac));
  assert(registry.registerTelemetry(mac, 1, telemetry, -60, response,
                                     txPower, duplicate, generation));
  assert(generation != initialGeneration);
  const unsigned before = writes;
  assert(!registry.persistTelemetry(mac, 3, telemetry, -60, initialGeneration, received));
  assert(writes == before);
  // Migration preserves the compact environmental V3 prefix.
  saved[0] = station::SensorConfig{};
  saved[0].occupied = true; saved[0].provisioned = true;
  saved[0].sleepSeconds = 600; memcpy(saved[0].mac, mac, 6);
  #pragma pack(push, 1)
  struct V3Header { uint32_t magic; uint16_t version, sampleSize;
                    uint32_t revision, bucketSeconds; uint16_t capacity, reserved; };
  #pragma pack(pop)
  const V3Header header{0x48495354, 3, 23, 12, 600, 144, 0};
  histories[0].assign(sizeof(header) + 144 * 23, 0);
  memcpy(histories[0].data(), &header, sizeof(header));
  station::HistorySample old{};
  old.timestamp = received / 600 * 600; old.temperatureCentiC = 2345;
  old.capabilities = lil::protocol::kTemperature;
  memcpy(histories[0].data() + sizeof(header) + (old.timestamp / 600 % 144) * 23, &old, 23);
  station::SensorRegistry migrated;
  assert(migrated.begin(store, 600));
  assert(migrated.history(mac, samples, 200) == 1);
  assert(samples[0].temperatureCentiC == 2345 && samples[0].timestamp == old.timestamp);
  assert(histories[0].size() == sizeof(header) + 144 * sizeof(station::HistorySample));
  puts("Registry interleaving, bounded RAM history, no fast flash writes and V3 migration passed");
}

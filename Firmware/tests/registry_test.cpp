#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif
#include <assert.h>
#include <ctime>
#include <functional>
#include <limits>
#include <vector>
#include <algorithm>
#include <chrono>
#include <deque>
#include "sensor_registry.h"
#include "normal_history_policy.h"
#include "esp_heap_caps.h"
int64_t testTimeUs = 100000;
void testRadioTick() { ++testMillis; }

namespace {
std::function<void()> storageHook;
unsigned writes = 0;
unsigned failConfigSaves = 0;
bool failHistoryDelete = false;
std::vector<uint8_t> histories[station::kMaxSensors];
station::SensorConfig saved[station::kMaxSensors];
station::SensorIdentity identities[station::kMaxSensorIdentities];
void onStorage() { ++writes; auto hook = storageHook; if (hook) hook(); }
}

namespace station {
bool ConfigStore::loadSensorIdentities(SensorIdentity* out, size_t count) {
  memcpy(out, identities, count * sizeof(SensorIdentity)); return true;
}
bool ConfigStore::saveSensorIdentities(const SensorIdentity* in, size_t count) {
  onStorage(); memcpy(identities, in, count * sizeof(SensorIdentity)); return true;
}
bool ConfigStore::loadSensorConfigs(SensorConfig* out, size_t count) {
  for (size_t i = 0; i < count; ++i) out[i] = saved[i];
  return true;
}
bool ConfigStore::saveSensorConfig(size_t index, const SensorConfig& config) {
  onStorage();
  if (failConfigSaves) { --failConfigSaves; return false; }
  saved[index] = config; return true;
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
  onStorage(); if (failHistoryDelete) return false;
  histories[index].clear(); return true;
}
bool ConfigStore::loadLatestTelemetry(size_t, void*, size_t) { return false; }
bool ConfigStore::saveLatestTelemetry(size_t, const void*, size_t) { onStorage(); return true; }
bool ConfigStore::deleteLatestTelemetry(size_t) { onStorage(); return true; }
}

void liveRingRegression(bool benchmark) {
  using namespace lil::protocol;
  for (const auto type : {EnvironmentalSensorType::kLsm6dsox,
                         EnvironmentalSensorType::kTmp117, EnvironmentalSensorType::kMax30102}) {
    for (auto& config : saved) config = {};
    for (auto& history : histories) history.clear();
    const uint8_t mac[6] = {2, 4, 6, 8, 10, 12};
    saved[0].occupied = saved[0].provisioned = true;
    memcpy(saved[0].mac, mac, 6); saved[0].sleepSeconds = 1;
    saved[0].environmentalSensorType = type;
    station::ConfigStore store;
    station::SensorRegistry registry;
    assert(registry.begin(store, 1));
    const size_t interval = liveHistoryIntervalMs(type), ringCapacity = 60000 / interval + 1;
    std::vector<station::LiveSample> output(ringCapacity + 1);
    std::deque<station::LiveSample> expected;
    TelemetryPayload value{};
    value.sensorType = type; value.bootCount = 1;
    value.capabilities = type == EnvironmentalSensorType::kLsm6dsox ? kMotion :
        type == EnvironmentalSensorType::kTmp117 ? kTemperature : kOptical;
    value.live.flags = kLiveTimingKnown | kLiveSampleFresh;
    assert(!registry.liveHistory(mac, output.data(), output.size()));
    assert(!registry.liveHistory(mac, nullptr, output.size()));
    for (size_t i = 0; i < 2 * ringCapacity + 19; ++i) {
      const uint64_t monotonic = uint64_t(UINT32_MAX) - 10000 + i * interval;
      testTimeUs = int64_t(monotonic * 1000); testMillis = uint32_t(monotonic);
      value.temperatureC = 20 + float(i) / 1000;
      value.batteryMillivolts = 3800 + i % 100;
      value.live.processingMs = uint16_t(i);
      station::SensorConfig response;
      int8_t txPower; bool duplicate; uint32_t generation;
      assert(registry.registerTelemetry(mac, i + 1, value, -60, response, txPower, duplicate, generation));
      expected.push_back({testMillis, value});
      while (expected.size() > ringCapacity || testMillis - expected.front().receivedMs > 60000) expected.pop_front();
      if (i % 97 == 0 || i + 1 == ringCapacity || i + 1 == 2 * ringCapacity) {
        for (const size_t limit : {size_t(0), size_t(1), ringCapacity - 1, ringCapacity, ringCapacity + 1}) {
          const size_t count = registry.liveHistory(mac, output.data(), limit);
          assert(count == std::min(limit, expected.size()));
          for (size_t j = 0; j < count; ++j) {
            assert(output[j].receivedMs == expected[j].receivedMs);
            assert(!memcmp(&output[j].telemetry, &expected[j].telemetry, sizeof(value)));
          }
        }
        const size_t cursorPosition = expected.size() / 2;
        const uint32_t cursor = expected[cursorPosition].receivedMs;
        const size_t count = registry.liveHistory(mac, output.data(), output.size(), &cursor);
        assert(count == expected.size() - cursorPosition - 1);
        for (size_t j = 0; j < count; ++j) {
          const auto& row = expected[cursorPosition + j + 1];
          assert(output[j].receivedMs == row.receivedMs);
          assert(!memcmp(&output[j].telemetry, &row.telemetry, sizeof(value)));
        }
      }
    }
    if (benchmark) {
      constexpr size_t runs = 9, iterations = 1000;
      double times[runs]; size_t checksum = 0;
      for (size_t run = 0; run < runs; ++run) {
        const auto start = std::chrono::steady_clock::now();
        for (size_t i = 0; i < iterations; ++i)
          checksum += registry.liveHistory(mac, output.data(), output.size());
        times[run] = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count() / iterations;
      }
      std::sort(times, times + runs);
      printf("Live ring host type=%u rows=%zu runs=%zu iterations=%zu median=%.6f us/read range=%.6f..%.6f checksum=%zu\n",
             unsigned(type), expected.size(), runs, iterations, times[runs / 2], times[0], times[runs - 1], checksum);
    }
    testMillis += 20000; testTimeUs += 20000000;
    while (!expected.empty() && testMillis - expected.front().receivedMs > 60000) expected.pop_front();
    assert(registry.liveHistory(mac, output.data(), output.size()) == expected.size());
    for (size_t j = 0; j < expected.size(); ++j) assert(output[j].receivedMs == expected[j].receivedMs);
  }
  puts("Live ring: all three capacities, empty/partial/full/wrapped/truncated/expired reads and millis rollover preserve exact sample order/bytes");
}

void registrationBenchmark() {
  using namespace lil::protocol;
  for (const auto type : {EnvironmentalSensorType::kBme280, EnvironmentalSensorType::kBme680}) {
    for (auto& config : saved) config = {};
    for (auto& history : histories) history.clear();
    const uint8_t mac[6] = {2, 5, 8, 11, 14, 17};
    saved[0].occupied = saved[0].provisioned = true;
    memcpy(saved[0].mac, mac, 6); saved[0].environmentalSensorType = type;
    station::ConfigStore store; station::SensorRegistry registry;
    assert(registry.begin(store, 600));
    TelemetryPayload value{};
    value.sensorType = type; value.bootCount = 1; value.temperatureC = 21.5F;
    value.capabilities = kTemperature | kIaq; value.iaqAccuracy = 2;
    station::SensorConfig response; int8_t power; bool duplicate; uint32_t generation;
    constexpr size_t runs = 9, iterations = 20000;
    double times[runs]; uint32_t sequence = 0;
    testMillis = 1000; testTimeUs = 1000000;
    for (size_t run = 0; run < runs; ++run) {
      const auto start = std::chrono::steady_clock::now();
      for (size_t i = 0; i < iterations; ++i) {
        const bool fallback = type == EnvironmentalSensorType::kBme680 && (i & 1);
        value.flags = fallback ? kBme680RawFallback : 0; value.iaq = fallback ? 0 : 42;
        assert(registry.registerTelemetry(mac, ++sequence, value, -60, response, power, duplicate, generation));
        assert(!duplicate);
      }
      times[run] = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count() / iterations;
    }
    station::SensorView view; assert(registry.findView(mac, view));
    assert(view.runtime.receivedPackets == runs * iterations && view.runtime.telemetry.iaq == 42 && view.runtime.telemetry.temperatureC == 21.5F);
    std::sort(times, times + runs);
    printf("Registration host type=%u runs=%zu iterations=%zu median=%.6f us/packet range=%.6f..%.6f packets=%lu\n",
           unsigned(type), runs, iterations, times[runs / 2], times[0], times[runs - 1], static_cast<unsigned long>(sequence));
  }
}

int main(int argc, char** argv) {
  {
    uint8_t parsed[6] = {9,9,9,9,9,9};
    assert(station::SensorRegistry::parseMac("a:BC:03:04:05:ff", parsed));
    assert(parsed[0] == 10 && parsed[1] == 188 && parsed[5] == 255);
    const uint8_t unchanged[6] = {10,188,3,4,5,255};
    for (const char* invalid : {"FFFFFFFFFFFFFFFFFFFFFFFF:02:03:04:05:06",
         "100:02:03:04:05:06", "01:02:03:04:05:06x", "01:02:03:04:05:",
         "-1:02:03:04:05:06", " 1:02:03:04:05:06"}) {
      assert(!station::SensorRegistry::parseMac(invalid, parsed));
      assert(!memcmp(parsed, unchanged, sizeof(parsed)));
    }
    assert(!station::SensorRegistry::parseMac(String(std::string("01:02:03:04:05:06\0x",19)), parsed));
    assert(!memcmp(parsed, unchanged, sizeof(parsed)));
    assert(!station::SensorRegistry::parseMac("01:02:03:04:05:06", nullptr));
  }
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
  auto malformed = telemetry;
  malformed.temperatureC = std::numeric_limits<float>::quiet_NaN();
  assert(!registry.registerTelemetry(mac, 1, malformed, -60, response, txPower, duplicate, generation));
  malformed = telemetry; malformed.live.count = lil::protocol::kOpticalBatchSize + 1;
  assert(!registry.registerTelemetry(mac, 1, malformed, -60, response, txPower, duplicate, generation));
  malformed = telemetry; malformed.sensorType = static_cast<lil::protocol::EnvironmentalSensorType>(42);
  assert(!registry.registerTelemetry(mac, 1, malformed, -60, response, txPower, duplicate, generation));
  malformed = telemetry; malformed.operatingMode = static_cast<lil::protocol::SensorOperatingMode>(42);
  assert(!registry.registerTelemetry(mac, 1, malformed, -60, response, txPower, duplicate, generation));
  malformed = telemetry; malformed.capabilities = lil::protocol::kMotion;
  malformed.motion.accelerationG[0] = std::numeric_limits<float>::infinity();
  assert(!registry.registerTelemetry(mac, 1, malformed, -60, response, txPower, duplicate, generation));
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
  assert(samples[0].timestamp == received);
  assert(samples[0].temperatureCentiC == 2150);

  // Failed NVS actions remain absent from radio replies throughout the write.
  station::SensorConfig beforeFailedWrite{};
  assert(registry.findConfig(mac, beforeFailedWrite));
  storageHook = [&] {
    station::SensorConfig during{};
    assert(registry.findConfig(mac, during));
    assert(!memcmp(&during, &beforeFailedWrite, sizeof(during)));
  };
  failConfigSaves = 1;
  assert(!registry.updateConfig(mac, "Unsaved name", 30, 123, "key", true, {}, 0xff,
      lil::protocol::EnvironmentalSensorType::kBme680, .5F, 1));
  failConfigSaves = 1; assert(!registry.setThingSpeakChannel(mac, 999, "unsaved-key", 0));
  failConfigSaves = 1; assert(!registry.requestFactoryReset(mac));
  failConfigSaves = 1; assert(!registry.requestIaqCalibrationReset(mac));
  station::SensorConfig afterFailedWrite{};
  assert(registry.findConfig(mac, afterFailedWrite));
  assert(!memcmp(&beforeFailedWrite, &afterFailedWrite, sizeof(afterFailedWrite)));
  assert(!strcmp(saved[0].name, beforeFailedWrite.name));
  storageHook = nullptr;

  assert(registry.updateConfig(mac, "Test", 30, 0, "", false, {}, 0xff,
             lil::protocol::EnvironmentalSensorType::kBme680, 0.4F, 1.0F));
  assert(registry.historyCapacity(mac) == 900);
  assert(registry.requestIaqCalibrationReset(mac));
  assert(registry.requestFactoryReset(mac));
  assert(registry.setThingSpeakChannel(mac, 123, "key", 0));
  station::SensorConfig cloudSnapshot{};
  assert(registry.findConfig(mac, cloudSnapshot, &generation));
  assert(registry.cloudUploadMatches(mac, generation, 123, "key", cloudSnapshot.thingSpeakFields));
  assert(!registry.cloudUploadMatches(mac, generation + 1, 123, "key", cloudSnapshot.thingSpeakFields));
  assert(!registry.cloudUploadMatches(mac, generation, 123, "old-key", cloudSnapshot.thingSpeakFields));
  auto changedFields = cloudSnapshot.thingSpeakFields; changedFields.temperature = 8;
  assert(!registry.cloudUploadMatches(mac, generation, 123, "key", changedFields));
  station::ThingSpeakChannelProfile profile{};
  profile.occupied = true; profile.channelId = 456;
  strcpy(profile.writeApiKey, "new-key");
  failConfigSaves = 1;
  assert(!registry.syncThingSpeakProfile(0, profile));
  assert(registry.cloudUploadMatches(mac, generation, 123, "key", cloudSnapshot.thingSpeakFields));
  assert(saved[0].thingSpeakChannelId == 123 && !strcmp(saved[0].thingSpeakWriteKey, "key"));
  failConfigSaves = 1;
  assert(!registry.removeThingSpeakProfile(0));
  assert(registry.cloudUploadMatches(mac, generation, 123, "key", cloudSnapshot.thingSpeakFields));
  assert(saved[0].thingSpeakProfileSlot == 0);
  assert(registry.syncThingSpeakProfile(0, profile));
  assert(!registry.cloudUploadMatches(mac, generation, 123, "key", cloudSnapshot.thingSpeakFields));
  assert(registry.removeThingSpeakProfile(0));
  assert(!registry.cloudUploadMatches(mac, generation, 456, "new-key", cloudSnapshot.thingSpeakFields));

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
  // A short history view must use reception time, not the start of its bucket.
  assert(registry.updateConfig(mac, "Environmental", 900, 0, "", false, {}, 0xff,
             lil::protocol::EnvironmentalSensorType::kBme280, 0, 1));
  telemetry.sensorType = lil::protocol::EnvironmentalSensorType::kBme280;
  telemetry.flags = 0;
  const uint32_t recent = static_cast<uint32_t>(time(nullptr)) - 1;
  assert(registry.registerTelemetry(mac, 4, telemetry, -60, response, txPower, duplicate, generation));
  assert(registry.persistTelemetry(mac, 4, telemetry, -60, generation, recent));
  count = registry.history(mac, samples, 200);
  assert(samples[count - 1].timestamp == recent);
  assert(registry.updateConfig(mac, "Environmental", 60, 0, "", false, {}, 0xff,
             lil::protocol::EnvironmentalSensorType::kBme280, 0, 1));
  count = registry.history(mac, samples, 200);
  assert(samples[count - 1].timestamp == recent); // Resizing keeps measured times.
  assert(registry.updateConfig(mac, "Motion", 1, 0, "", false, {}, 0xff,
             lil::protocol::EnvironmentalSensorType::kLsm6dsox, 0, 1));
  telemetry.flags = 0;
  telemetry.sensorType = lil::protocol::EnvironmentalSensorType::kLsm6dsox;
  telemetry.capabilities = lil::protocol::kMotion;
  telemetry.motion.accelerationG[0] = -1.234F;
  telemetry.motion.angularRateDps[1] = -123.4F;
  telemetry.motion.peakAccelerationG = 2.5F;
  telemetry.motion.fifoOverrun = 1;
  assert(registry.registerTelemetry(mac, 5, telemetry, -60, response, txPower, duplicate, generation));
  assert(registry.persistTelemetry(mac, 5, telemetry, -60, generation, received));
  const unsigned fastWrites = writes;
  for (uint32_t i = 1; i <= 901; ++i) {
    testMillis += 100;
    testTimeUs = static_cast<int64_t>(testMillis) * 1000;
    assert(registry.registerTelemetry(mac, 5 + i, telemetry, -60, response, txPower, duplicate, generation));
    assert(registry.persistTelemetry(mac, 5 + i, telemetry, -60, generation, received + i));
  }
  assert(writes == fastWrites); // No per-report flash writes in fast mode.
  station::LiveSample motionSamples[station::kLiveCapacity]{};
  assert(registry.liveHistory(mac, motionSamples, station::kLiveCapacity) == 601);
  assert(motionSamples[600].receivedMs - motionSamples[0].receivedMs == 60000);
  assert(motionSamples[600].telemetry.motion.accelerationG[0] == -1.234F);
  assert(motionSamples[600].telemetry.motion.fifoOverrun);
  telemetry.motion.fifoOverrun = 0;
  telemetry.live.flags = lil::protocol::kLiveTimingKnown;
  // Cached-value heartbeats update the card/contact status, not the graph.
  assert(registry.registerTelemetry(mac, 910, telemetry, -60, response, txPower, duplicate, generation));
  assert(registry.liveHistory(mac, motionSamples, station::kLiveCapacity) == 601);
  telemetry.live.flags |= lil::protocol::kLiveSampleFresh;
  assert(registry.registerTelemetry(mac, 911, telemetry, -60, response, txPower, duplicate, generation));
  assert(registry.liveHistory(mac, motionSamples, station::kLiveCapacity) == 602);
  testMillis += 60001;
  testTimeUs = static_cast<int64_t>(testMillis) * 1000;
  assert(registry.liveHistory(mac, motionSamples, station::kLiveCapacity) == 0);
  telemetry.flags = lil::protocol::kBatteryProtectionActive;
  telemetry.operatingMode = lil::protocol::SensorOperatingMode::kBatteryProtection;
  telemetry.capabilities = lil::protocol::kBattery;
  telemetry.batteryMillivolts = 2790;
  assert(registry.registerTelemetry(mac, 1000, telemetry, -60, response, txPower, duplicate, generation));
  assert(registry.persistTelemetry(mac, 1000, telemetry, -60, generation, received + 200));
  assert(registry.liveHistory(mac, motionSamples, station::kLiveCapacity) == 0);
  assert(registry.findView(mac, view) && view.runtime.telemetry.capabilities == lil::protocol::kBattery);
  assert(view.runtime.hasPersistedTelemetry && view.runtime.telemetry.batteryMillivolts == 2790);
  telemetry.flags = 0;
  // References are saved by MAC, survive re-pairing/reboots, and reject movement.
  telemetry.capabilities = lil::protocol::kMotion | lil::protocol::kBattery;
  telemetry.sensorType = lil::protocol::EnvironmentalSensorType::kLsm6dsox;
  telemetry.live.flags = lil::protocol::kLiveTimingKnown | lil::protocol::kLiveSampleFresh;
  telemetry.live.acquisitionAgeMs = 5;
  telemetry.motion = {};
  telemetry.motion.accelerationG[0] = .01F; telemetry.motion.accelerationG[2] = 1.0F;
  telemetry.motion.angularRateDps[0] = .2F;
  assert(registry.registerTelemetry(mac, 1001, telemetry, -60, response, txPower, duplicate, generation));
  assert(registry.setMotionReference(mac, false));
  assert(registry.findView(mac, view) && view.motionReference.enabled);
  assert(view.motionReference.accelerationG[2] == 1.0F);
  const auto beforeLongOutage = testTimeUs;
  testTimeUs += (uint64_t(UINT32_MAX) + 1) * 1000;
  assert(!registry.setMotionReference(mac, false));
  testTimeUs = beforeLongOutage;
  telemetry.motion.angularRateDps[0] = 50;
  assert(registry.registerTelemetry(mac, 1002, telemetry, -60, response, txPower, duplicate, generation));
  assert(!registry.setMotionReference(mac, false));
  assert(registry.findView(mac, view) && view.motionReference.angularRateDps[0] == .2F);
  telemetry.motion.angularRateDps[0] = .2F;
  assert(registry.updateConfig(mac, "Table sensor", 10, 0, "", false, {}, 0xff,
             lil::protocol::EnvironmentalSensorType::kLsm6dsox, 0.0F, 1.0F));
  failHistoryDelete = true;
  assert(!registry.deleteSensor(mac));
  assert(saved[0].occupied && registry.findView(mac, view));
  failHistoryDelete = false;
  assert(registry.deleteSensor(mac));
  assert(registry.registerTelemetry(mac, 1, telemetry, -60, response,
                                     txPower, duplicate, generation));
  assert(generation != initialGeneration);
  assert(!strcmp(response.name, "Table sensor"));
  assert(registry.findView(mac, view) && view.motionReference.enabled);
  assert(registry.setMotionReference(mac, true));
  assert(registry.findView(mac, view) && !view.motionReference.enabled);
  assert(registry.deleteSensor(mac));
  station::SensorRegistry afterReboot;
  assert(afterReboot.begin(store, 600));
  assert(afterReboot.registerTelemetry(mac, 1, telemetry, -60, response, txPower, duplicate, generation));
  assert(!strcmp(response.name, "Table sensor"));
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
  // A failed battery ADC must not become a valid history voltage point.
  telemetry = {}; telemetry.bootCount = 99;
  telemetry.sensorType = lil::protocol::EnvironmentalSensorType::kBme280;
  telemetry.capabilities = lil::protocol::kBattery | lil::protocol::kTemperature;
  telemetry.flags = lil::protocol::kBatteryReadFailed | lil::protocol::kSensorReadFailed;
  telemetry.temperatureC = std::numeric_limits<float>::quiet_NaN();
  assert(migrated.registerTelemetry(mac, 1, telemetry, -60, response, txPower, duplicate, generation));
  assert(migrated.persistTelemetry(mac, 1, telemetry, -60, generation, recent + 1200));
  count = migrated.history(mac, samples, 200);
  assert(count && samples[count - 1].capabilities == 0);
  // Finite extreme values must saturate before multiplication/rounding.
  telemetry.flags = 0;
  telemetry.capabilities = lil::protocol::kTemperature | lil::protocol::kHumidity |
      lil::protocol::kPressure | lil::protocol::kIaq;
  telemetry.temperatureC = telemetry.humidityPercent = telemetry.pressureHpa =
      telemetry.iaq = std::numeric_limits<float>::max();
  telemetry.iaqAccuracy = 3;
  assert(migrated.registerTelemetry(mac, 2, telemetry, -60, response, txPower, duplicate, generation));
  assert(migrated.persistTelemetry(mac, 2, telemetry, -60, generation, recent + 1800));
  count = migrated.history(mac, samples, 200);
  assert(samples[count - 1].temperatureCentiC == INT16_MAX);
  assert(samples[count - 1].humidityCentiPercent == UINT16_MAX);
  assert(samples[count - 1].pressureDeciHpa == UINT16_MAX && samples[count - 1].iaqDeci == UINT16_MAX);
  telemetry.temperatureC = std::numeric_limits<float>::lowest();
  assert(migrated.registerTelemetry(mac, 3, telemetry, -60, response, txPower, duplicate, generation));
  assert(migrated.persistTelemetry(mac, 3, telemetry, -60, generation, recent + 2400));
  count = migrated.history(mac, samples, 200);
  assert(samples[count - 1].temperatureCentiC == INT16_MIN);
  // Even unsupported gas placeholders reach the legacy compact encoder.
  // Bound before float-to-integer conversion; preserve ordinary truncation.
  for (const float gas : {1234.75F, std::numeric_limits<float>::max(),
                         -1.0F, std::numeric_limits<float>::quiet_NaN(),
                         std::numeric_limits<float>::infinity()}) {
    telemetry.gasResistanceOhms = gas;
    static uint32_t gasSequence = 4;
    const uint32_t gasTime = recent + 3000 + (gasSequence - 4) * 600;
    assert(migrated.registerTelemetry(mac, gasSequence, telemetry, -60, response, txPower, duplicate, generation));
    assert(migrated.persistTelemetry(mac, gasSequence++, telemetry, -60, generation, gasTime));
    count = migrated.history(mac, samples, 200);
    const uint32_t expectedGas = gas == 1234.75F ? 1234 :
        gas == std::numeric_limits<float>::max() ? UINT32_MAX : 0;
    assert(samples[count - 1].gasResistanceOhms == expectedGas);
  }
  #pragma pack(push, 1)
  struct LegacyV2Sample {
    uint32_t timestamp; uint16_t capabilities;
    float temperature, humidity, pressure, iaq, gasKohms, batteryVolts;
    uint8_t iaqAccuracy, sensorType, pcbVersion;
  };
  #pragma pack(pop)
  struct LegacyV2History { uint32_t magic; uint16_t version; uint32_t revision;
                           LegacyV2Sample samples[48]; } legacy{};
  static_assert(sizeof(legacy) == 1596);
  legacy.magic = 0x48495354; legacy.version = 2; legacy.revision = 1;
  legacy.samples[0].timestamp = recent;
  legacy.samples[0].temperature = std::numeric_limits<float>::max();
  legacy.samples[0].humidity = legacy.samples[0].pressure = legacy.samples[0].iaq =
      legacy.samples[0].batteryVolts = std::numeric_limits<float>::max();
  legacy.samples[1].timestamp = recent + 600;
  legacy.samples[1].temperature = std::numeric_limits<float>::lowest();
  histories[0].resize(sizeof(legacy)); memcpy(histories[0].data(), &legacy, sizeof(legacy));
  station::SensorRegistry migratedV2;
  assert(migratedV2.begin(store, 600));
  assert(migratedV2.history(mac, samples, 200) == 2);
  assert(samples[0].temperatureCentiC == INT16_MAX && samples[1].temperatureCentiC == INT16_MIN);
  assert(samples[0].humidityCentiPercent == UINT16_MAX && samples[0].pressureDeciHpa == UINT16_MAX);
  assert(samples[0].iaqDeci == UINT16_MAX && samples[0].batteryMillivolts == UINT16_MAX);
  // A full remembered-name cache evicts only an inactive identity.
  for (size_t i = 0; i < station::kMaxSensorIdentities; ++i) {
    identities[i] = {}; identities[i].mac[0] = 2; identities[i].mac[5] = i;
    strcpy(identities[i].name, "Historical sensor");
  }
  station::SensorRegistry fullCache;
  assert(fullCache.begin(store, 600));
  assert(fullCache.updateConfig(mac, "Active sensor", 600, 0, "", false, {}, 0xff,
      lil::protocol::EnvironmentalSensorType::kBme280, 0, 1));
  assert(fullCache.findView(mac, view) && !strcmp(view.config.name, "Active sensor"));
  // The production paged backend migrates legacy values without changing Wi-Fi
  // or sensor configuration, and normal precision samples do not need a live ring.
  station::PagedHistory paged;
  assert(paged.begin(true));
  const auto configBeforeMigration = saved[0];
  station::SensorRegistry withPages;
  assert(withPages.begin(store, 600, &paged));
  assert(!memcmp(&saved[0], &configBeforeMigration, sizeof(saved[0])));
  assert(histories[0].empty() && paged.info(mac).durablePoints == 2);
  assert(withPages.updateConfig(mac, "Temperature", 1, 0, "", false, {}, 0xff,
      lil::protocol::EnvironmentalSensorType::kTmp117, 0, 1));
  telemetry = {}; telemetry.sensorType = lil::protocol::EnvironmentalSensorType::kTmp117;
  telemetry.operatingMode = lil::protocol::SensorOperatingMode::kEnergySaving;
  telemetry.capabilities = lil::protocol::kTemperature | lil::protocol::kBattery;
  telemetry.temperatureC = 21.123046875F; telemetry.batteryMillivolts = 3805;
  assert(station::usesNormalHistory(telemetry));
  auto manualTelemetry = telemetry;
  manualTelemetry.operatingMode = lil::protocol::SensorOperatingMode::kContinuousPrecision;
  assert(!station::usesNormalHistory(manualTelemetry));
  telemetry.live.flags = lil::protocol::kLiveTimingKnown | lil::protocol::kLiveSampleFresh;
  assert(withPages.registerTelemetry(mac, 500, telemetry, -60, response, txPower, duplicate, generation));
  const uint32_t normalTime = recent + 3600;
  assert(withPages.persistTelemetry(mac, 500, telemetry, -60, generation, normalTime));
  assert(withPages.liveHistory(mac, motionSamples, station::kLiveCapacity) == 0);
  const auto normalCount = paged.info(mac).points;
  telemetry.live.flags = lil::protocol::kLiveTimingKnown; // Cached contact, no fresh measurement.
  assert(withPages.registerTelemetry(mac, 501, telemetry, -60, response, txPower, duplicate, generation));
  assert(withPages.persistTelemetry(mac, 501, telemetry, -60, generation, normalTime + 1));
  assert(paged.info(mac).points == normalCount);
  station::NormalHistoryPoint page[4]; bool stale, readError;
  assert(paged.read(mac, normalTime - 1, normalTime, 0, page, 4, stale, readError) == 1);
  assert(!stale && !readError && page[0].telemetry.temperatureC == telemetry.temperatureC);

  // A complete millis cycle must not revive expired live samples, including
  // after a cached heartbeat or the first fresh measurement following an outage.
  station::SensorRegistry longRunning;
  assert(longRunning.begin(store, 600));
  const uint64_t liveStarted = uint64_t(UINT32_MAX) - 15;
  const auto setLiveTime = [](uint64_t milliseconds) {
    testMillis = static_cast<uint32_t>(milliseconds);
    testTimeUs = static_cast<int64_t>(milliseconds * 1000);
  };
  setLiveTime(liveStarted);
  auto longLived = manualTelemetry;
  longLived.live.flags = lil::protocol::kLiveTimingKnown | lil::protocol::kLiveSampleFresh;
  assert(longRunning.registerTelemetry(mac, 1, longLived, -60, response, txPower, duplicate, generation));
  setLiveTime(liveStarted + 20);
  assert(longRunning.liveHistory(mac, motionSamples, station::kLiveCapacity) == 1);
  setLiveTime(liveStarted + 60000);
  assert(longRunning.liveHistory(mac, motionSamples, station::kLiveCapacity) == 1);
  setLiveTime(liveStarted + 60001);
  assert(longRunning.liveHistory(mac, motionSamples, station::kLiveCapacity) == 0);
  setLiveTime(liveStarted + (uint64_t(UINT32_MAX) + 1) + 20);
  assert(longRunning.liveHistory(mac, motionSamples, station::kLiveCapacity) == 0);
  longLived.live.flags = lil::protocol::kLiveTimingKnown;
  assert(longRunning.registerTelemetry(mac, 2, longLived, -60, response, txPower, duplicate, generation));
  assert(longRunning.liveHistory(mac, motionSamples, station::kLiveCapacity) == 0);
  longLived.live.flags |= lil::protocol::kLiveSampleFresh;
  longLived.temperatureC = 22.5F;
  assert(longRunning.registerTelemetry(mac, 3, longLived, -60, response, txPower, duplicate, generation));
  assert(longRunning.liveHistory(mac, motionSamples, station::kLiveCapacity) == 1);
  assert(motionSamples[0].receivedMs == testMillis && motionSamples[0].telemetry.temperatureC == 22.5F);
  liveRingRegression(argc == 2 && !strcmp(argv[1], "--benchmark"));
  if (argc == 2 && !strcmp(argv[1], "--benchmark")) registrationBenchmark();
  puts("Registry: radio/storage interleaving, durable settings, cloud cancellation, semantic validation and bounded history passed");
}

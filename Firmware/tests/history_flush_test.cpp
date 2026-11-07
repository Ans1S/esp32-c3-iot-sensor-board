#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif
#include <cassert>
#include <cstdio>
#include <filesystem>
#include <LittleFS.h>
#include <esp_heap_caps.h>
#include "paged_history.h"
#include "sensor_registry.h"

// Set to zero only when measuring the unchanged ff7afa3 implementation.
#ifndef EXPECT_HISTORY_FLUSH_SEEKS
#define EXPECT_HISTORY_FLUSH_SEEKS 2
#endif
int64_t testTimeUs = 0;
void testRadioTick() { ++testMillis; }
namespace {
constexpr uint32_t epoch = 1792886400;
void resetCounters() {
  testFileSeekCount = testFileReadCount = 0;
  testFileSeekFailAt = testFileReadFailAt = testFileReadCorruptAt = 0;
}
void clearFiles() {
  for (const auto& entry : std::filesystem::directory_iterator(NORMAL_HISTORY_MOUNT_PATH))
    std::filesystem::remove(entry.path());
  LittleFS.extraUsed = 0;
  resetCounters();
}
void verify(station::PagedHistory& store, const uint8_t* mac, size_t expected) {
  station::NormalHistoryPoint points[126];
  bool stale, error;
  assert(store.read(mac, 0, 0, 0, points, 126, stale, error) == expected);
  assert(!stale && !error);
  for (size_t i = 0; i < expected; ++i) {
    assert(points[i].timestamp == epoch + i);
    assert(points[i].telemetry.batteryMillivolts == 3800 + i);
  }
}
}  // namespace
int main() {
  using Type = lil::protocol::EnvironmentalSensorType;
  testSpiramFreeBytes = 4 * 1024 * 1024;
  for (const auto type : {Type::kBme280, Type::kTmp117, Type::kLsm6dsox}) {
    clearFiles();
    station::SensorConfig config{};
    config.occupied = config.provisioned = true;
    config.mac[0] = 2; config.mac[5] = 1;
    config.sleepSeconds = 1; config.environmentalSensorType = type;
    const size_t rows = type == Type::kBme280 ? 91 : type == Type::kTmp117 ? 126 : 63;
    station::PagedHistory store;
    assert(store.begin(true)); store.configure(&config, 1);
    lil::protocol::TelemetryPayload value{};
    value.sensorType = type;
    value.capabilities = lil::protocol::kBattery;
    for (size_t i = 0; i < rows; ++i) {
      value.batteryMillivolts = 3800 + i;
      if (i + 1 == rows) resetCounters();
      assert(store.append(config.mac, epoch + i, value, 1));
    }
    const size_t seeks = testFileSeekCount, reads = testFileReadCount;
    assert(seeks == (EXPECT_HISTORY_FLUSH_SEEKS ? EXPECT_HISTORY_FLUSH_SEEKS : rows + 1));
    assert(reads == rows);
    assert(store.info(config.mac).durablePoints == rows && !store.info(config.mac).pendingPoints);
    verify(store, config.mac, rows);
    station::PagedHistory reboot;
    assert(reboot.begin(true)); reboot.configure(&config, 1);
    verify(reboot, config.mac, rows);
    printf("History flush type=%u rows=%zu: %zu seeks, %zu checked row reads\n",
           static_cast<unsigned>(type), rows, seeks, reads);

    // Positioning failure, short middle read and corrupted readback must leave
    // the entire RAM batch pending. A retry must retain every original sample.
    for (int fault = 0; fault < 3; ++fault) {
      clearFiles();
      station::PagedHistory failed;
      assert(failed.begin(true)); failed.configure(&config, 1);
      for (size_t i = 0; i < 40; ++i) {
        value.batteryMillivolts = 3800 + i;
        assert(failed.append(config.mac, epoch + i, value, 1));
      }
      resetCounters();
      if (fault == 0) testFileSeekFailAt = 2;
      if (fault == 1) testFileReadFailAt = 20;
      if (fault == 2) testFileReadCorruptAt = 20;
      assert(!failed.flush(config.mac));
      const auto pending = failed.info(config.mac);
      assert(pending.writeFailed && pending.pendingPoints == 40 && !pending.durablePoints);
      resetCounters();
      assert(failed.flush(config.mac));
      assert(!failed.info(config.mac).writeFailed && !failed.info(config.mac).pendingPoints);
      verify(failed, config.mac, 40);
    }
  }
  puts("History flush: complete readback, reboot, seek/read/corruption faults and retry passed");
}

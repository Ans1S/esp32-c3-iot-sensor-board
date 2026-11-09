#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif
#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <vector>
#include <LittleFS.h>
#include <esp_heap_caps.h>
#include "paged_history.h"
#include "sensor_registry.h"
#include "filesystem_guard.h"

int64_t testTimeUs = 0;
void testRadioTick() { ++testMillis; }
namespace {
constexpr uint32_t epoch = 1792886400;
station::SensorConfig configs[station::kMaxSensors];
void configure(size_t count, uint32_t interval = 1) {
  for (size_t i = 0; i < station::kMaxSensors; ++i) {
    configs[i] = {};
    configs[i].occupied = configs[i].provisioned = i < count;
    configs[i].mac[0] = 2; configs[i].mac[5] = i + 1;
    configs[i].sleepSeconds = interval;
    configs[i].environmentalSensorType = lil::protocol::EnvironmentalSensorType::kBme280;
  }
}
void clearFiles() {
  for (const auto& entry : std::filesystem::directory_iterator(NORMAL_HISTORY_MOUNT_PATH))
    std::filesystem::remove(entry.path());
  LittleFS.extraUsed = 0;
}
std::vector<station::NormalHistoryPoint> read(station::PagedHistory& store, size_t index,
                                            uint32_t ceiling = 0) {
  const auto before = store.info(configs[index].mac);
  std::vector<station::NormalHistoryPoint> result;
  station::NormalHistoryPoint batch[37];
  uint32_t after = 0;
  bool stale, error;
  while (true) {
    const auto count = store.read(configs[index].mac, after, ceiling ? ceiling : before.newest,
                                 before.oldest, batch, 37, stale, error);
    assert(!stale && !error);
    if (!count) break;
    result.insert(result.end(), batch, batch + count);
    after = batch[count - 1].timestamp;
  }
  return result;
}
}
int main() {
  using namespace lil::protocol;
  // A long-idle archive reserve must not revive when 32-bit millis wraps.
  testTimeUs = 100000;
  station::noteArchiveWork();
  assert(station::historyWorkspaceReserve() == 320U * 1024U);
  testTimeUs += 60000LL * 1000;
  assert(station::historyWorkspaceReserve() == 256U * 1024U);
  testTimeUs += (uint64_t(UINT32_MAX) + 1) * 1000;
  assert(station::historyWorkspaceReserve() == 256U * 1024U);
  clearFiles(); configure(1);
  station::PagedHistory history;
  assert(history.begin(true)); history.configure(configs, station::kMaxSensors);
  TelemetryPayload value{};
  value.sensorType = EnvironmentalSensorType::kBme280;
  value.capabilities = kTemperature | kBattery;
  value.temperatureC = 21.25F; value.batteryMillivolts = 3805;
  assert(history.append(configs[0].mac, epoch, value, 1));
  assert(history.info(configs[0].mac).pendingPoints == 1);
  assert(read(history, 0).at(0).telemetry.batteryMillivolts == 3805);
  testMillis = 31000;
  history.flushDue();
  assert(history.info(configs[0].mac).durablePoints == 1);
  // A fixed snapshot ceiling excludes measurements appended during pagination.
  for (uint32_t i = 1; i <= 201; ++i) assert(history.append(configs[0].mac, epoch + i, value, 1));
  assert(history.flush(configs[0].mac));
  assert(read(history, 0, epoch + 100).size() == 101);
  assert(read(history, 0).size() == 202);
  // A selected page must retain every row while avoiding per-row seeks.
  station::NormalHistoryPoint measuredPage[80];
  bool pageStale, pageError;
  testFileReadCount = testFileSeekCount = 0;
  assert(history.read(configs[0].mac, 0, epoch + 79, epoch,
                      measuredPage, 80, pageStale, pageError) == 80);
  const size_t pageSeeks = testFileSeekCount;
  assert(!pageStale && !pageError && measuredPage[79].timestamp == epoch + 79);
  printf("History page: %zu file reads and %zu seeks for 80 rows\n", testFileReadCount, pageSeeks);
  assert(testFileReadCount == 10 && pageSeeks == 1);
  testFileReadCount = testFileSeekCount = 0;
  assert(history.read(configs[0].mac, epoch + 10, epoch + 12, epoch,
                      measuredPage, 80, pageStale, pageError) == 2);
  printf("Bounded history window: %zu file reads for two requested rows\n", testFileReadCount);
  assert(!pageStale && !pageError && measuredPage[0].timestamp == epoch + 11 &&
         measuredPage[1].timestamp == epoch + 12);
  assert(testFileReadCount == 2 && testFileSeekCount == 1);
  assert(history.append(configs[0].mac, epoch, value, 1));
  assert(history.info(configs[0].mac).points == 202); // Duplicate radio delivery.
  station::PagedHistory reboot;
  assert(reboot.begin(true)); reboot.configure(configs, station::kMaxSensors);
  assert(read(reboot, 0).size() == 202);
  assert(read(reboot, 0).back().telemetry.temperatureC == 21.25F);

  // CRC recovery retains the valid prefix, ignores a torn tail, then resumes.
  auto tail = std::filesystem::path();
  for (const auto& entry : std::filesystem::directory_iterator(NORMAL_HISTORY_MOUNT_PATH))
    if (entry.path().filename().string() > tail.filename().string()) tail = entry.path();
  FILE* file = fopen(tail.string().c_str(), "r+b"); assert(file);
  assert(!fseek(file, -1, SEEK_END)); const uint8_t corrupt = 0xa5;
  assert(fwrite(&corrupt, 1, 1, file) == 1); fclose(file);
  station::PagedHistory recovered;
  assert(recovered.begin(true)); recovered.configure(configs, station::kMaxSensors);
  assert(read(recovered, 0).size() == 201);
  assert(recovered.append(configs[0].mac, epoch + 202, value, 1));
  assert(recovered.flush(configs[0].mac));
  assert(read(recovered, 0).size() == 202);

  clearFiles(); configure(1);
  configs[0].environmentalSensorType = EnvironmentalSensorType::kTmp117;
  station::PagedHistory precision;
  assert(precision.begin(true)); precision.configure(configs, station::kMaxSensors);
  value.sensorType = EnvironmentalSensorType::kTmp117; value.temperatureC = 21.123046875F;
  assert(precision.append(configs[0].mac, epoch, value, 1));
  assert(precision.flush(configs[0].mac));
  assert(read(precision, 0).at(0).telemetry.temperatureC == value.temperatureC);
  value.flags = kBatteryReadFailed;
  assert(precision.append(configs[0].mac, epoch + 1, value, 1));
  assert(!(read(precision, 0).back().telemetry.capabilities & kBattery));
  value.flags = kSensorReadFailed;
  assert(precision.append(configs[0].mac, epoch + 2, value, 1));
  assert(read(precision, 0).back().telemetry.live.flags & kLiveGap);
  value = {}; value.sensorType = EnvironmentalSensorType::kLsm6dsox;
  value.capabilities = kMotion; value.motion.accelerationG[0] = 0.123456F;
  assert(precision.append(configs[0].mac, epoch + 3, value, 1));
  assert(precision.flush(configs[0].mac));
  assert(read(precision, 0).back().telemetry.motion.accelerationG[0] == value.motion.accelerationG[0]);

  clearFiles(); configure(station::kMaxSensors);
  station::PagedHistory full;
  assert(full.begin(true)); full.configure(configs, station::kMaxSensors);
  value = {}; value.sensorType = EnvironmentalSensorType::kBme280;
  value.capabilities = kBattery; value.batteryMillivolts = 3805;
  file = fopen(NORMAL_HISTORY_MOUNT_PATH "/saved-recording.bin", "wb"); assert(file);
  const uint8_t sentinel = 0x42; assert(fwrite(&sentinel, 1, 1, file) == 1); fclose(file);
  for (uint32_t row = 0; row < 1400; ++row)
    for (size_t node = 0; node < station::kMaxSensors; ++node)
      assert(full.append(configs[node].mac, epoch + row, value, 1));
  for (size_t node = 0; node < station::kMaxSensors; ++node) {
    assert(full.flush(configs[node].mac));
    const auto info = full.info(configs[node].mac);
    assert(info.points > 900 && info.points < 1400 && info.oldest > epoch);
    assert(info.points <= info.capacityPoints);
    const auto points = read(full, node);
    assert(points.size() == info.points && points.front().timestamp == info.oldest &&
           points.back().timestamp == info.newest && info.newest == epoch + 1399);
  }
  assert(LittleFS.totalBytes() - LittleFS.usedBytes() >= station::PagedHistory::kReserveBytes);
  bool stale, error; station::NormalHistoryPoint batch[2];
  assert(full.read(configs[0].mac, 0, 0, epoch, batch, 2, stale, error) == 0 && stale && !error);
  file = fopen(NORMAL_HISTORY_MOUNT_PATH "/saved-recording.bin", "rb"); assert(file);
  assert(fgetc(file) == sentinel); fclose(file);
  const auto before = full.info(configs[0].mac).points;
  LittleFS.extraUsed = LittleFS.totalBytes();
  for (uint32_t i = 1400; i < 1440; ++i) full.append(configs[0].mac, epoch + i, value, 1);
  assert(full.info(configs[0].mac).writeFailed);
  assert(full.info(configs[0].mac).pendingPoints <= 91);
  assert(full.info(configs[0].mac).dropped > 0);
  assert(full.info(configs[0].mac).points <= before + 91);
  file = fopen(NORMAL_HISTORY_MOUNT_PATH "/saved-recording.bin", "rb"); assert(file);
  assert(fgetc(file) == sentinel); fclose(file);
  LittleFS.extraUsed = 0;
  assert(full.flush(configs[0].mac));
  assert(!full.info(configs[0].mac).writeFailed);
  station::PagedHistory unavailable;
  assert(unavailable.begin(false)); unavailable.configure(configs, station::kMaxSensors);
  assert(!unavailable.append(configs[0].mac, epoch, value, 1));
  assert(!unavailable.info(configs[0].mac).available);

  // At global space pressure pick the oldest page of the largest owner,
  // including retained pages from a sensor that is no longer configured.
  clearFiles(); configure(4);
  testSpiramFreeBytes = 4 * 1024 * 1024;
  station::PagedHistory eviction;
  assert(eviction.begin(true)); eviction.configure(configs, station::kMaxSensors);
  uint32_t nextTime[2] = {epoch, epoch};
  for (size_t owner : {1, 0, 0, 1, 0, 1}) {
    for (size_t row = 0; row < 91; ++row)
      assert(eviction.append(configs[owner].mac, nextTime[owner]++, value, 1));
  }
  configs[1].provisioned = false;
  eviction.configure(configs, station::kMaxSensors);
  const auto retiredFirst = std::filesystem::path(NORMAL_HISTORY_MOUNT_PATH) / "hist2-020000000002-00000001.bin";
  const auto activeFirst = std::filesystem::path(NORMAL_HISTORY_MOUNT_PATH) / "hist2-020000000001-00000002.bin";
  const auto activeSecond = std::filesystem::path(NORMAL_HISTORY_MOUNT_PATH) / "hist2-020000000001-00000003.bin";
  assert(std::filesystem::exists(retiredFirst) && std::filesystem::exists(activeFirst));
  LittleFS.extraUsed = LittleFS.totalBytes() - LittleFS.usedBytes() -
      station::historyWorkspaceReserve() - station::PagedHistory::kPageBytes;
  assert(eviction.append(configs[2].mac, epoch, value, 1) && eviction.flush(configs[2].mac));
  assert(!std::filesystem::exists(retiredFirst) && std::filesystem::exists(activeFirst));
  assert(eviction.append(configs[3].mac, epoch, value, 1) && eviction.flush(configs[3].mac));
  assert(!std::filesystem::exists(activeFirst) && std::filesystem::exists(activeSecond));
  assert(eviction.info(configs[0].mac).durablePoints == 182 &&
         eviction.info(configs[0].mac).oldest == epoch + 91);

  // Refuse allocation under memory pressure, then resume without formatting
  // storage or presenting the rejected sample as a retained measurement.
  clearFiles(); configure(1);
  station::PagedHistory memoryPressure;
  assert(memoryPressure.begin(true)); memoryPressure.configure(configs, station::kMaxSensors);
  testSpiramFreeBytes = 0; testInternalFreeBytes = 32 * 1024;
  assert(!memoryPressure.append(configs[0].mac, epoch, value, 1));
  assert(memoryPressure.info(configs[0].mac).points == 0);
  assert(memoryPressure.info(configs[0].mac).writeFailed && memoryPressure.info(configs[0].mac).dropped == 1);
  testInternalFreeBytes = 1024 * 1024; testHeapAllocationFailures = 1;
  assert(!memoryPressure.append(configs[0].mac, epoch + 1, value, 1));
  assert(memoryPressure.info(configs[0].mac).points == 0);
  assert(memoryPressure.append(configs[0].mac, epoch + 2, value, 1));
  assert(memoryPressure.flush(configs[0].mac));
  assert(read(memoryPressure, 0).size() == 1);
  assert(read(memoryPressure, 0)[0].timestamp == epoch + 2);
  assert(!memoryPressure.info(configs[0].mac).writeFailed);

  // Exercise full PSRAM pages as well as the smaller internal-RAM fallback.
  clearFiles(); configure(1);
  configs[0].environmentalSensorType = EnvironmentalSensorType::kTmp117;
  testSpiramFreeBytes = 4 * 1024 * 1024;
  station::PagedHistory fullBuffers;
  assert(fullBuffers.begin(true)); fullBuffers.configure(configs, station::kMaxSensors);
  value = {}; value.sensorType = EnvironmentalSensorType::kTmp117;
  value.capabilities = kTemperature; value.temperatureC = 21.123046875F;
  for (uint32_t i = 0; i < 127; ++i) assert(fullBuffers.append(configs[0].mac, epoch + i, value, 1));
  assert(fullBuffers.info(configs[0].mac).durablePoints == 126);
  assert(fullBuffers.info(configs[0].mac).pendingPoints == 1);
  value = {}; value.sensorType = EnvironmentalSensorType::kLsm6dsox;
  value.capabilities = kMotion; value.motion.accelerationG[0] = .123456F;
  for (uint32_t i = 127; i < 254; ++i) assert(fullBuffers.append(configs[0].mac, epoch + i, value, 1));
  assert(fullBuffers.flush(configs[0].mac));
  const auto mixed = read(fullBuffers, 0);
  assert(mixed.size() == 254 && mixed[126].telemetry.temperatureC == 21.123046875F);
  assert(mixed[127].telemetry.motion.accelerationG[0] == .123456F);
  assert(mixed.back().timestamp == epoch + 253);

  value = {}; value.sensorType = EnvironmentalSensorType::kTmp117;
  value.capabilities = kTemperature; value.temperatureC = 21.123046875F;
  for (uint32_t i = 254; i < 379; ++i) assert(fullBuffers.append(configs[0].mac, epoch + i, value, 1));
  const auto narrower = fullBuffers.info(configs[0].mac);
  printf("TMP117 after 64-byte rows: %lu pending points out of 125\n",
         static_cast<unsigned long>(narrower.pendingPoints));
  assert(narrower.durablePoints == 254 && narrower.pendingPoints == 125);
  assert(fullBuffers.flush(configs[0].mac));
  assert(read(fullBuffers, 0).size() == 379);
  configs[0].provisioned = false;
  fullBuffers.configure(configs, station::kMaxSensors);
  assert(!fullBuffers.info(configs[0].mac).available);
  configs[0].provisioned = true;
  fullBuffers.configure(configs, station::kMaxSensors);
  const auto reenabled = fullBuffers.info(configs[0].mac);
  assert(reenabled.durablePoints == 379 && reenabled.oldest == epoch && reenabled.newest == epoch + 378);

  // Corruption discovered by an active reader must stop a page with an error.
  tail.clear();
  for (const auto& entry : std::filesystem::directory_iterator(NORMAL_HISTORY_MOUNT_PATH))
    if (tail.empty() || entry.path().filename().string() < tail.filename().string()) tail = entry.path();
  file = fopen(tail.string().c_str(), "r+b"); assert(file);
  assert(!fseek(file, 64 + 10 * 32, SEEK_SET));
  const int oldByte = fgetc(file); assert(oldByte != EOF);
  assert(!fseek(file, -1, SEEK_CUR)); assert(fputc(oldByte ^ 1, file) != EOF); assert(!fclose(file));
  station::NormalHistoryPoint corruptPage[128];
  assert(fullBuffers.read(configs[0].mac, 0, epoch + 8, epoch, corruptPage, 128, stale, error) == 9);
  assert(!error && !stale);
  assert(fullBuffers.read(configs[0].mac, 0, epoch + 253, epoch, corruptPage, 128, stale, error) == 10);
  assert(error && !stale);
  // A short batch can contain complete rows and an unreadable partial row.
  std::filesystem::resize_file(tail, 64 + 10 * 32 + 15);
  assert(fullBuffers.read(configs[0].mac, 0, epoch + 8, epoch, corruptPage, 128, stale, error) == 9);
  assert(!error && !stale);
  assert(fullBuffers.read(configs[0].mac, 0, epoch + 253, epoch, corruptPage, 128, stale, error) == 10);
  assert(error && !stale);
  assert(fullBuffers.clear(configs[0].mac));
  const auto cleared = fullBuffers.info(configs[0].mac);
  assert(cleared.available && !cleared.points && !cleared.oldest && !cleared.newest);
  assert(fullBuffers.append(configs[0].mac, epoch + 4, value, 1));
  assert(fullBuffers.flush(configs[0].mac));
  const auto afterClear = fullBuffers.info(configs[0].mac);
  assert(afterClear.durablePoints == 1 && afterClear.oldest == epoch + 4 && afterClear.newest == epoch + 4);
  testSpiramFreeBytes = testInternalFreeBytes = 1024 * 1024;
  puts("paged history: bounded retention, precision, paging, recovery and reserves passed");
}

#include "recording_archive.h"
#include <LittleFS.h>
#include <cassert>
#include <cstdio>
#include <limits>

int64_t testTimeUs = 0;
void testRadioTick() { ++testMillis; }
int main() {
  using namespace lil;
  const uint8_t mac[6] = {1,2,3,4,5,6};
  station::RecordingArchive archive; assert(archive.begin());
  protocol::TelemetryPayload payload{};
  payload.sensorType = protocol::EnvironmentalSensorType::kLsm6dsox;
  recording::Upload upload{}; upload.totalRecords = 100; upload.durationMs = 10000;
  upload.sessionEpochMs = 1700000000000ULL;
  payload.capabilities = protocol::kMotion;
  payload.motion.accelerationG[0] = std::numeric_limits<float>::quiet_NaN();
  upload.record = recording::encode(1, 0, payload);
  assert(!archive.append(mac, upload));
  payload.motion.accelerationG[0] = 0;
  for (uint32_t i = 0; i < 100; ++i) {
    payload.motion.accelerationG[0] = float(i)/123;
    upload.record = recording::encode(1,i*100,payload);
    assert(archive.append(mac, upload));
    assert(archive.append(mac, upload)); // Lost ACK retry is not a second row.
  }
  auto sessions = archive.list(mac);
  assert(sessions.size() == 1 && sessions[0].stored == 100 && sessions[0].expected == 100);
  assert(sessions[0].availableMs == 9900);
  // Measure a complete sequential page independently of timestamp searches.
  recording::Record page[100]{}; station::RecordingInfo pageInfo{};
  uint32_t pageOffset = 0;
  testFileSeekCount = 0;
  assert(archive.read(mac, 1, pageOffset, page, 100, pageInfo) == 100);
  const size_t pageSeeks = testFileSeekCount;
  assert(pageOffset == 100 && page[0].sampleMs == 0 && page[99].sampleMs == 9900);
  assert(pageSeeks <= 4); // Header, end, tail validation and one page seek.
  printf("Archive page: %zu file seeks for 100 rows\n", pageSeeks);
  recording::Record points[16]{}; station::RecordingInfo info{};
  uint32_t offset = 0;
  assert(archive.read(mac, 1, offset, points, 16, info, 5050) == 16);
  assert(points[0].sampleMs == 5100 && offset == 67 && info.epochMs == upload.sessionEpochMs);
  assert(info.availableMs == 9900);
  testFileSeekCount = testFileReadCount = 0;
  assert(archive.append(mac, upload)); // Immediate replay of the newest durable row.
  printf("Latest archive retry: %zu seeks and %zu reads\n", testFileSeekCount, testFileReadCount);
  assert(testFileSeekCount == 3 && testFileReadCount == 2);
  // An older duplicate after a station reboot must still be recognized.
  station::RecordingArchive reboot; assert(reboot.begin());
  payload.motion.accelerationG[0] = float(7)/123;
  upload.record = recording::encode(1,700,payload); assert(reboot.append(mac, upload));
  payload.motion.accelerationG[0] = -50;
  upload.record = recording::encode(1,700,payload); assert(!reboot.append(mac, upload));
  // Storage pressure does not acknowledge / erase the sensor's pending data.
  upload.record = recording::encode(2,0,payload); LittleFS.used = LittleFS.totalBytes();
  assert(!reboot.append(mac, upload)); LittleFS.used = 0;
  assert(reboot.append(mac, upload));
  // An incomplete archive exposes only the acquisition time already stored.
  auto partial = reboot.list(mac);
  for (const auto& session : partial) if (session.session == 2) {
    assert(session.stored == 1 && session.expected == 100 && session.availableMs == 0);
  }
  for (uint32_t time : {100U, 200U}) {
    upload.record = recording::encode(2, time, payload);
    assert(reboot.append(mac, upload));
  }
  upload.record = recording::encode(2, 300, payload);
  testFileReadFailAt = testFileReadCount + 2; // Header succeeds, tail read fails.
  assert(!reboot.append(mac, upload));
  testFileReadFailAt = 0;
  testFileSeekFailAt = testFileSeekCount + 3; // Header/end succeed, tail seek fails.
  assert(!reboot.append(mac, upload));
  testFileSeekFailAt = 0;
  testFileFailNextOpen = true;
  assert(!reboot.append(mac, upload));
  testFileReadFailAt = testFileReadCount + 2;
  offset = 0;
  assert(reboot.read(mac, 2, offset, points, 16, info) == 0);
  assert(offset == 0 && info.session == 2 && info.readError);
  testFileReadFailAt = 0;
  offset = 0;
  assert(reboot.read(mac, 2, offset, points, 16, info) == 3);
  assert(offset == 3 && points[2].sampleMs == 200 && !info.readError);
  assert(reboot.append(mac, upload));
  assert(reboot.read(mac, 2, offset, points, 16, info) == 1 && points[0].sampleMs == 300);
  assert(reboot.remove(mac, 2));
  assert(reboot.list(mac).size() == 1);
  // A torn whole tail record is not counted as complete and can be replayed.
  char path[512];
  snprintf(path, sizeof(path), RECORDING_MOUNT_PATH "/rec-010203040506-0000000000000001.bin");
  FILE* file = fopen(path, "r+b"); assert(file);
  assert(!fseek(file, 64 + 99*recording::size(payload.sensorType), SEEK_SET));
  assert(fputc(0xFF, file) != EOF); assert(!fclose(file));
  assert(reboot.list(mac)[0].stored == 99);
  offset = 98;
  assert(reboot.read(mac, 1, offset, points, 16, info) == 1);
  assert(offset == 99 && info.stored == 99 && !info.readError);
  payload.motion.accelerationG[0] = float(99)/123;
  upload.record = recording::encode(1,9900,payload);
  assert(reboot.append(mac, upload));
  assert(reboot.list(mac)[0].stored == 100);
  // Corruption within a page is reported without skipping a record offset.
  file = fopen(path, "r+b"); assert(file);
  assert(!fseek(file, 64 + 10*recording::size(payload.sensorType), SEEK_SET));
  assert(fputc(0xFF, file) != EOF); assert(!fclose(file));
  offset = 9;
  assert(reboot.read(mac, 1, offset, points, 16, info) == 1);
  assert(offset == 10 && info.readError);
  assert(reboot.remove(mac, 1));
  assert(reboot.list(mac).empty());
  puts("Station recording archive: durable append, duplicate ACKs, reboot, paging, full storage and deletion passed");
}

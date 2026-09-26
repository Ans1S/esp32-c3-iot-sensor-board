#include "recording_journal.h"
#include "recording_button.h"
#include <cassert>
#include <cstdio>
#include <vector>
#include <algorithm>

struct MemoryFlash : lil::recording::Flash {
  std::vector<uint8_t> data;
  int writeBudget = -1;
  explicit MemoryFlash(size_t bytes) : data(bytes, 0xff) {}
  size_t bytes() const override { return data.size(); }
  bool read(size_t offset, void* out, size_t length) override {
    if (offset + length > data.size()) return false;
    memcpy(out, data.data() + offset, length); return true;
  }
  bool write(size_t offset, const void* input, size_t length) override {
    if (offset + length > data.size()) return false;
    const auto* source = static_cast<const uint8_t*>(input);
    for (size_t i = 0; i < length; ++i) {
      if (writeBudget == 0) return false;
      if (writeBudget > 0) --writeBudget;
      assert((data[offset+i] & source[i]) == source[i]);
      data[offset+i] &= source[i];
    }
    return true;
  }
  bool erase(size_t offset, size_t length) override {
    assert(offset % 4096 == 0 && length % 4096 == 0);
    std::fill(data.begin() + offset, data.begin() + offset + length, 0xff); return true;
  }
};
int main() {
  using namespace lil;
  {
    protocol::TelemetryPayload payload{};
    payload.sensorType = protocol::EnvironmentalSensorType::kLsm6dsox;
    payload.live.flags = protocol::kMotionFeedbackPresent;
    payload.motion.accelerationG[2] = 1.012345F;
    payload.motion.sampleCount = 65535; payload.motion.fifoOverrun = 1;
    payload.motionFeedback = {65535,65535};
    const auto record = recording::encode(42,1800000,payload);
    assert(recording::valid(record) && recording::size(record.type) == 64);
    const auto decoded = recording::decode(record);
    assert(decoded.motionFeedback.steps == 65535 && decoded.motionFeedback.activeSeconds == 65535);
    assert(decoded.motion.sampleCount == 65535 && decoded.motion.fifoOverrun == 1);
    assert(decoded.motion.accelerationG[2] == payload.motion.accelerationG[2]);
  }
  recording::Button button;
  button.begin(true, 0); // A held boot strap is not a recording request.
  assert(!button.poll(true, 100));
  assert(!button.poll(false, 110)); assert(!button.poll(false, 150));
  assert(!button.poll(true, 160)); assert(!button.poll(false, 165));
  assert(!button.poll(true, 170)); assert(!button.poll(true, 204));
  assert(button.poll(true, 205)); assert(!button.poll(true, 5000));
  button.begin(false, UINT32_MAX-20);
  assert(!button.poll(true, UINT32_MAX-10)); assert(button.poll(true, 30));
  assert(timing::imuRecordingIntervalMs(299999) == 50 && timing::imuRecordingIntervalMs(300000) == 100);
  for (auto type : {protocol::EnvironmentalSensorType::kLsm6dsox,
                    protocol::EnvironmentalSensorType::kTmp117,
                    protocol::EnvironmentalSensorType::kMax30102}) {
    protocol::TelemetryPayload payload{}; payload.sensorType = type;
    payload.temperatureC = 37.0078125f;
    payload.motion.accelerationG[2] = .987654f;
    payload.motion.sampleCount = 11;
    payload.live.count = 8; payload.live.optical[7] = {262143,123456,299};
    const auto record = recording::encode(0x123456789ULL,100,payload);
    assert(recording::valid(record));
    const auto decoded = recording::decode(record);
    if (type == protocol::EnvironmentalSensorType::kLsm6dsox)
      assert(decoded.motion.accelerationG[2] == payload.motion.accelerationG[2]);
    if (type == protocol::EnvironmentalSensorType::kTmp117)
      assert(decoded.temperatureC == payload.temperatureC);
    if (type == protocol::EnvironmentalSensorType::kMax30102)
      assert(decoded.live.optical[7].red == 262143 && decoded.live.optical[7].ageMs == 299);
    MemoryFlash flash(8192); recording::Journal journal;
    assert(journal.begin(flash));
    const auto count = journal.capacity(type);
    for (size_t i = 0; i < count; ++i) assert(journal.append(recording::encode(1,i,payload)));
    assert(!journal.append(record)); // Full never overwrites unacknowledged records.
    assert(journal.pending() == count);
    recording::Journal reboot; assert(reboot.begin(flash) && reboot.pending() == count);
    for (size_t i = 0; i < count; ++i) {
      recording::Record next{}; assert(reboot.peek(next) && next.sampleMs == i);
      assert(!reboot.acknowledge(record)); // Wrong session / record is not an ACK.
      assert(reboot.acknowledge(next));
    }
    assert(reboot.pending() == 0 && reboot.append(record));
    recording::Journal afterAck; assert(afterAck.begin(flash) && afterAck.pending() == 1);
  }
  // Every possible interruption during a record write and its commit bitmap.
  protocol::TelemetryPayload payload{}; payload.sensorType = protocol::EnvironmentalSensorType::kLsm6dsox;
  for (int cut = 0; cut <= 68; ++cut) {
    MemoryFlash flash(8192); recording::Journal journal; assert(journal.begin(flash));
    assert(journal.append(recording::encode(1,1,payload)));
    flash.writeBudget = cut;
    journal.append(recording::encode(1,2,payload));
    flash.writeBudget = -1;
    recording::Journal reboot; assert(reboot.begin(flash));
    recording::Record record{}; assert(reboot.peek(record) && record.sampleMs == 1);
    assert(reboot.acknowledge(record));
    if (reboot.peek(record)) assert(record.sampleMs == 2 && recording::valid(record));
  }
  MemoryFlash capacityFlash(0x160000); recording::Journal capacity; assert(capacity.begin(capacityFlash));
  const size_t adaptiveMotionRecords = 300000 / timing::kImuShortSessionMs + 1500000 / timing::kImuReportMs;
  assert(adaptiveMotionRecords == 21000);
  assert(capacity.capacity(protocol::EnvironmentalSensorType::kLsm6dsox) >= adaptiveMotionRecords);
  assert(capacity.capacity(protocol::EnvironmentalSensorType::kMax30102) >= 9000);
  // Explicit new-session replacement makes old pages reusable, without erase
  // stalls or reappearing old data after reboot.
  MemoryFlash replacementFlash(8192); recording::Journal replacement;
  assert(replacement.begin(replacementFlash));
  assert(replacement.append(recording::encode(1,100,payload)));
  assert(replacement.replaceSession(2) && replacement.pending() == 0);
  assert(replacement.append(recording::encode(2,50,payload)));
  recording::Journal recovered; assert(recovered.begin(replacementFlash,2) && recovered.pending() == 1);
  recording::Record remaining; assert(recovered.peek(remaining) && remaining.session == 2);
  puts("Recording codec/journal: exact values, 30-minute capacity, reboot, torn writes, ACK identity and full retention passed");
}

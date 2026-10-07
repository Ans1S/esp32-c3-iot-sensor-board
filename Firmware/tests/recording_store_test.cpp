#include <Arduino.h>
#include "recording_store.h"
#include <cassert>
void testRadioTick() { ++testMillis; }
int main() {
  using Type = lil::protocol::EnvironmentalSensorType;
  using State = lil::recording::State;
  sensor::RecordingStore missing;
  recordingPartitionAvailable = false;
  assert(!missing.begin(Type::kTmp117) && !missing.start());
  recordingPartitionAvailable = true;
  sensor::RecordingStore store; assert(store.begin(Type::kTmp117));
  assert(store.status(0).state == State::Ready);
  lil::protocol::TelemetryPayload payload{}; payload.sensorType = Type::kTmp117;
  payload.temperatureC = 37.125f;
  assert(!store.append(testMillis,payload)); // No implicit recording at boot.
  const uint32_t firstStartedMs = testMillis;
  assert(store.start()); testMillis += 1000; assert(store.append(testMillis,payload));
  const auto first = store.status(0).session;
  lil::recording::Upload upload{}; assert(!store.next(upload)); // Replay only after stop.
  store.stop(); assert(store.next(upload) && upload.totalRecords == 1 && upload.sessionEpochMs == 0);
  const uint32_t frozenDuration = store.status(0).elapsedMs;
  testMillis += 60000;
  assert(store.status(0).elapsedMs == frozenDuration);
  store.stop();
  assert(store.status(0).elapsedMs == frozenDuration);
  // First station contact can arrive after recording has stopped. Do not
  // anchor the beginning using the frozen duration and the later wall clock.
  testMillis += 20;
  store.anchor(1700000060010ULL, frozenDuration, 20);
  assert(store.next(upload) && upload.sessionEpochMs ==
      1700000060010ULL - (testMillis - firstStartedMs - 10));
  // Keep the following reboot fixture's original unknown UTC condition.
  recordingPreferences.erase("epoch");
  sensor::RecordingStore reboot; testMillis = 10;
  assert(reboot.begin(Type::kTmp117) && reboot.status(0).state == State::Pending);
  reboot.anchor(1700000000000ULL,1000,10);
  assert(reboot.next(upload) && upload.sessionEpochMs == 0); // Unknown power-off time stays unknown.
  assert(reboot.start()); assert(reboot.status(0).session != first && reboot.status(0).pending == 0);
  testMillis += 1000; assert(reboot.append(testMillis,payload));
  auto status = reboot.status(0); reboot.anchor(1700000000000ULL,status.elapsedMs,20);
  reboot.stop(); assert(reboot.next(upload) && upload.sessionEpochMs > 0);
  auto wrong = upload.record; wrong.sampleMs++;
  assert(!reboot.acknowledge(wrong) && reboot.status(0).pending == 1);
  assert(reboot.acknowledge(upload.record) && reboot.status(0).state == State::Synced);
  sensor::RecordingStore synced; assert(synced.begin(Type::kTmp117) && synced.status(0).pending == 0);
  assert(synced.start()); testMillis+=1000; assert(synced.append(testMillis,payload));
  testMillis=0; sensor::RecordingStore interrupted;
  assert(interrupted.begin(Type::kTmp117) && !interrupted.recording());
  assert(interrupted.next(upload) && upload.totalRecords == 1 && upload.durationMs >= 1000);
  puts("Recording lifecycle: manual start, stop-only sync, reboot recovery, UTC uncertainty, replacement and ACK reclamation passed");
}

#include "recording_store.h"
#include <Arduino.h>
#include <esp_random.h>
#include <esp_timer.h>

namespace sensor {
using State = lil::recording::State;
bool RecordingFlash::begin() {
  partition_ = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
      static_cast<esp_partition_subtype_t>(0x41), "recordings");
  return partition_ && partition_->size >= 0x160000;
}
bool RecordingFlash::read(size_t offset, void* data, size_t length) {
  return partition_ && esp_partition_read(partition_, offset, data, length) == ESP_OK;
}
bool RecordingFlash::write(size_t offset, const void* data, size_t length) {
  return partition_ && esp_partition_write(partition_, offset, data, length) == ESP_OK;
}
bool RecordingFlash::erase(size_t offset, size_t length) {
  const bool ok = partition_ && esp_partition_erase_range(partition_, offset, length) == ESP_OK;
  delay(1); return ok;
}
bool RecordingStore::begin(lil::protocol::EnvironmentalSensorType type) {
  type_ = type;
  if (!flash_.begin() || !prefs_.begin("recording", false)) return false;
  session_ = prefs_.getULong64("session", 0); epochMs_ = prefs_.getULong64("epoch", 0);
  if (!journal_.begin(flash_, session_)) return false;
  available_ = true;
  total_ = prefs_.getUInt("total", 0); durationMs_ = prefs_.getUInt("duration", 0);
  state_ = static_cast<State>(prefs_.getUChar("state", static_cast<uint8_t>(State::Ready)));
  if (state_ == State::Preparing) {
    state_ = State::Ready;
  } else if (state_ == State::Recording) {
    // A reset ends a session. Never invent elapsed time while power was absent.
    total_ = journal_.pending(); durationMs_ = journal_.lastSampleMs();
    stop();
  } else if (journal_.pending()) state_ = State::Pending;
  else state_ = session_ ? State::Synced : State::Ready;
  if (journal_.fault()) state_ = State::Fault;
  return true;
}
bool RecordingStore::start() {
  if (!available_) return false;
  // A new explicit press authorizes replacing the previous unsynced session.
  if (prefs_.putUChar("state", static_cast<uint8_t>(State::Preparing)) != 1) {
    state_ = State::Fault; return false;
  }
  session_ = (uint64_t(esp_random()) << 32) | esp_random();
  if (!session_) session_ = 1;
  epochMs_ = total_ = durationMs_ = 0;
  startedMs_ = esp_timer_get_time() / 1000; sameBoot_ = true;
  if (prefs_.putULong64("session", session_) != 8 || !journal_.replaceSession(session_) || prefs_.putULong64("epoch", 0) != 8 ||
      prefs_.putUChar("state", static_cast<uint8_t>(State::Recording)) != 1) { state_ = State::Fault; return false; }
  state_ = State::Recording; return true;
}
void RecordingStore::stop(bool full) {
  if (!available_ || !recording()) return;
  if (sameBoot_) durationMs_ = (esp_timer_get_time() / 1000) - startedMs_;
  total_ = journal_.pending();
  state_ = full ? State::Full : total_ ? State::Pending : State::Ready;
  if (prefs_.putUInt("duration", durationMs_) != 4 || prefs_.putUInt("total", total_) != 4 ||
      prefs_.putUChar("state", static_cast<uint8_t>(state_)) != 1) state_ = State::Fault;
}
bool RecordingStore::append(uint64_t capturedMs, const lil::protocol::TelemetryPayload& payload) {
  if (!recording() || capturedMs < startedMs_ || capturedMs - startedMs_ > UINT32_MAX) return false;
  const auto record = lil::recording::encode(session_, capturedMs - startedMs_, payload);
  if (journal_.append(record)) return true;
  stop(!journal_.fault());
  if (journal_.fault()) state_ = State::Fault;
  return false;
}
bool RecordingStore::next(lil::recording::Upload& upload) {
  if (recording() || !available_ || state_ == State::Fault || !journal_.peek(upload.record)) return false;
  if (upload.record.session != session_) { state_ = State::Fault; return false; }
  upload.sessionEpochMs = epochMs_; upload.totalRecords = total_; upload.durationMs = durationMs_;
  return true;
}
bool RecordingStore::acknowledge(const lil::recording::Record& record) {
  if (!journal_.acknowledge(record)) return false;
  if (!journal_.pending()) {
    state_ = State::Synced;
    if (prefs_.putUChar("state", static_cast<uint8_t>(state_)) != 1) { state_ = State::Fault; return false; }
    // ACK bits make pages reusable. Lazy erasure avoids a long idle flash stall.
  }
  return true;
}
void RecordingStore::anchor(uint64_t stationEpochMs, uint32_t elapsedAtRequestMs, uint32_t roundTripMs) {
  if (!sameBoot_ || epochMs_ || stationEpochMs < 1577836800000ULL || roundTripMs > 500) return;
  uint64_t elapsedAtRequest = elapsedAtRequestMs;
  if (!recording()) {
    // The public duration freezes at stop. UTC anchoring still needs the time
    // since start when the request was sent, including a delayed station return.
    const uint64_t now = esp_timer_get_time() / 1000;
    if (now < startedMs_ + roundTripMs) return;
    elapsedAtRequest = now - startedMs_ - roundTripMs;
  }
  const uint64_t elapsed = elapsedAtRequest + roundTripMs / 2;
  if (stationEpochMs <= elapsed) return;
  const uint64_t epoch = stationEpochMs - elapsed;
  if (prefs_.putULong64("epoch", epoch) == 8) epochMs_ = epoch;
}
lil::recording::Status RecordingStore::status(uint32_t dropped) const {
  lil::recording::Status status{};
  status.session = session_; status.state = state_; status.type = type_;
  status.pending = journal_.pending(); status.capacity = journal_.capacity(type_); status.dropped = dropped;
  status.elapsedMs = sameBoot_ && recording() ?
      uint32_t((esp_timer_get_time() / 1000) - startedMs_) : durationMs_;
  return status;
}
}  // namespace sensor

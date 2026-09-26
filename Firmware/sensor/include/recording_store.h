#pragma once
#include <Preferences.h>
#include <esp_partition.h>
#include "recording_journal.h"

namespace sensor {
class RecordingFlash : public lil::recording::Flash {
 public:
  bool begin();
  size_t bytes() const override { return partition_ ? partition_->size : 0; }
  bool read(size_t offset, void* data, size_t length) override;
  bool write(size_t offset, const void* data, size_t length) override;
  bool erase(size_t offset, size_t length) override;
 private:
  const esp_partition_t* partition_ = nullptr;
};
class RecordingStore {
 public:
  bool begin(lil::protocol::EnvironmentalSensorType type);
  bool start();
  void stop(bool full = false);
  bool append(uint64_t capturedMs, const lil::protocol::TelemetryPayload& payload);
  bool next(lil::recording::Upload& upload);
  bool acknowledge(const lil::recording::Record& record);
  void anchor(uint64_t stationEpochMs, uint32_t elapsedAtRequestMs, uint32_t roundTripMs);
  lil::recording::Status status(uint32_t dropped) const;
  bool recording() const { return state_ == lil::recording::State::Recording; }
  bool available() const { return available_; }
 private:
  RecordingFlash flash_;
  lil::recording::Journal journal_;
  Preferences prefs_;
  lil::protocol::EnvironmentalSensorType type_{};
  lil::recording::State state_ = lil::recording::State::Unavailable;
  uint64_t session_ = 0, startedMs_ = 0, epochMs_ = 0;
  uint32_t total_ = 0, durationMs_ = 0;
  bool available_ = false, sameBoot_ = false;
};
}  // namespace sensor

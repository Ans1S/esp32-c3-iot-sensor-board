#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <vector>
#include "recording_protocol.h"

namespace station {
struct RecordingInfo {
  uint64_t session = 0, epochMs = 0;
  uint32_t expected = 0, stored = 0, durationMs = 0;
  lil::protocol::EnvironmentalSensorType type{};
  uint32_t availableMs = 0;
  bool readError = false;
};
class RecordingArchive {
 public:
  bool begin();
  bool append(const uint8_t mac[6], const lil::recording::Upload& upload);
  std::vector<RecordingInfo> list(const uint8_t mac[6]);
  size_t read(const uint8_t mac[6], uint64_t session, uint32_t& offset,
      lil::recording::Record* output, size_t capacity, RecordingInfo& info,
      uint32_t fromMs = UINT32_MAX);
  bool remove(const uint8_t mac[6], uint64_t session);
  size_t freeBytes() const;
 private:
  SemaphoreHandle_t mutex_ = nullptr;
};
extern RecordingArchive recordingArchive;
}  // namespace station

#pragma once

#include <algorithm>
#include <cstring>
#include <ArduinoJson.h>
#include "recording_archive.h"

namespace station {
// Append directly to the response chunk. Buffer small serializer writes so
// the chunk only needs a length update and copy once per block of JSON.
template <typename Text>
class JsonAppendWriter {
 public:
  explicit JsonAppendWriter(Text& text) : text_(text) {}
  size_t write(uint8_t byte) {
    if (failed_) return 0;
    buffer_[used_++] = static_cast<char>(byte);
    return used_ != sizeof(buffer_) || flush();
  }
  size_t write(const uint8_t* bytes, size_t size) {
    size_t written = 0;
    while (!failed_ && written < size) {
      const size_t count = std::min(size - written, sizeof(buffer_) - used_);
      memcpy(buffer_ + used_, bytes + written, count);
      used_ += count;
      if (used_ == sizeof(buffer_) && !flush()) break;
      written += count;
    }
    return written;
  }
  bool flush() {
    if (used_ != 0 && !failed_) failed_ = !text_.concat(buffer_, used_);
    used_ = 0;
    return !failed_;
  }
 private:
  Text& text_;
  char buffer_[64];
  size_t used_ = 0;
  bool failed_ = false;
};

template <typename Text>
bool appendJson(const JsonDocument& document, Text& text) {
  if (document.overflowed()) return false;
  JsonAppendWriter<Text> writer(text);
  serializeJson(document, writer);
  return writer.flush();
}

inline void recordingInfoJson(JsonObject object, const RecordingInfo& info) {
  char session[17];
  snprintf(session, sizeof(session), "%016llx", static_cast<unsigned long long>(info.session));
  object["session"] = session; object["epochMs"] = info.epochMs;
  object["durationMs"] = info.durationMs; object["expected"] = info.expected;
  object["stored"] = info.stored; object["complete"] = !info.readError && info.stored == info.expected;
  if (info.readError) object["readError"] = true;
  object["sensorType"] = static_cast<uint8_t>(info.type);
  object["availableMs"] = info.availableMs;
}

inline void recordingPointJson(JsonDocument& point, const lil::recording::Record& record) {
  const auto t = lil::recording::decode(record);
  // ArduinoJson binds scalar arguments by reference. Materialize packed wire
  // fields as values so the library never receives an unaligned scalar reference.
  point["sampleMs"] = uint32_t(record.sampleMs);
  point["t"] = int64_t(record.sampleMs) - (record.ageMs == UINT16_MAX ? 0 : record.ageMs);
  point["estimateT"] = int64_t(record.sampleMs) - (t.live.estimateAgeMs == UINT16_MAX ? 0 : t.live.estimateAgeMs);
  point["windowMs"] = uint16_t(record.windowMs);
  point["gap"] = bool(record.flags & lil::protocol::kLiveGap) || t.motion.fifoOverrun;
  if ((t.capabilities & lil::protocol::kTemperature) && (t.live.flags & lil::protocol::kLiveSampleFresh)) point["temperature"] = float(t.temperatureC);
  if (t.capabilities & lil::protocol::kMotion) {
    const char* keys[] = {"ax","ay","az","gx","gy","gz"};
    for (int axis = 0; axis < 6; ++axis) point[keys[axis]] = float(axis < 3 ? t.motion.accelerationG[axis] : t.motion.angularRateDps[axis-3]);
    point["peakAcceleration"] = float(t.motion.peakAccelerationG); point["peakAngularRate"] = float(t.motion.peakAngularRateDps);
    point["sampleCount"] = uint16_t(t.motion.sampleCount);
    if (t.live.flags & lil::protocol::kMotionFeedbackPresent) {
      point["steps"] = uint16_t(t.motionFeedback.steps);
      point["activeSeconds"] = uint16_t(t.motionFeedback.activeSeconds);
    }
  }
  if ((t.capabilities & lil::protocol::kHeartRate) && (t.live.flags & lil::protocol::kLiveEstimateFresh)) point["heartRate"] = float(t.pulse.beatsPerMinute);
  point["pulseStatus"] = uint8_t(t.pulse.status); point["quality"] = uint8_t(t.pulse.quality);
  point["battery"] = record.batteryMv / 1000.0F;
  auto wave = point["wave"].to<JsonArray>();
  for (uint8_t j = 0; j < t.live.count; ++j) {
    auto frame = wave.add<JsonObject>(); frame["t"] = int64_t(record.sampleMs) - t.live.optical[j].ageMs;
    frame["red"] = uint32_t(t.live.optical[j].red); frame["infrared"] = uint32_t(t.live.optical[j].infrared);
  }
}
}  // namespace station

#pragma once

#include <algorithm>
#include <cstring>
#include <ArduinoJson.h>
#include "history_json.h"
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
  const uint8_t frameCount = std::min<uint8_t>(t.live.count, lil::protocol::kOpticalBatchSize);
  for (uint8_t j = 0; j < frameCount; ++j) {
    auto frame = wave.add<JsonObject>(); frame["t"] = int64_t(record.sampleMs) - t.live.optical[j].ageMs;
    frame["red"] = uint32_t(t.live.optical[j].red); frame["infrared"] = uint32_t(t.live.optical[j].infrared);
  }
}

// Direct zero-DOM recording-point JSON serializer: avoids building and tearing
// down an ArduinoJson document per record when streaming up to 21,000 saved
// samples to the browser.
#if defined(__GNUC__) && !defined(__clang__)
[[gnu::optimize("O2")]]
#endif
inline bool appendRecordingPointJson(char* dst, size_t cap, size_t& len,
                                     const lil::recording::Record& record) {
  const auto t = lil::recording::decode(record);
  const uint32_t sampleMs = uint32_t(record.sampleMs);
  const int64_t measT =
      int64_t(sampleMs) - (record.ageMs == UINT16_MAX ? 0 : record.ageMs);
  const int64_t estT =
      int64_t(sampleMs) - (t.live.estimateAgeMs == UINT16_MAX ? 0 : t.live.estimateAgeMs);
  const bool gap = bool(record.flags & lil::protocol::kLiveGap) || t.motion.fifoOverrun;
  bool ok = appendLit(dst, cap, len, "{\"sampleMs\":") &&
            appendU32(dst, cap, len, sampleMs) &&
            appendLit(dst, cap, len, ",\"t\":") &&
            appendSigned(dst, cap, len, measT) &&
            appendLit(dst, cap, len, ",\"estimateT\":") &&
            appendSigned(dst, cap, len, estT) &&
            appendLit(dst, cap, len, ",\"windowMs\":") &&
            appendU32(dst, cap, len, uint16_t(record.windowMs)) &&
            (gap ? appendLit(dst, cap, len, ",\"gap\":true")
                 : appendLit(dst, cap, len, ",\"gap\":false"));
  if (ok && (t.capabilities & lil::protocol::kTemperature) &&
      (t.live.flags & lil::protocol::kLiveSampleFresh)) {
    ok = appendLit(dst, cap, len, ",\"temperature\":") &&
         appendScaledFloat(dst, cap, len, float(t.temperatureC), 6);
  }
  if (ok && (t.capabilities & lil::protocol::kMotion)) {
    ok = appendLit(dst, cap, len, ",\"ax\":") &&
         appendScaledFloat(dst, cap, len, float(t.motion.accelerationG[0]), 6) &&
         appendLit(dst, cap, len, ",\"ay\":") &&
         appendScaledFloat(dst, cap, len, float(t.motion.accelerationG[1]), 6) &&
         appendLit(dst, cap, len, ",\"az\":") &&
         appendScaledFloat(dst, cap, len, float(t.motion.accelerationG[2]), 6) &&
         appendLit(dst, cap, len, ",\"gx\":") &&
         appendScaledFloat(dst, cap, len, float(t.motion.angularRateDps[0]), 4) &&
         appendLit(dst, cap, len, ",\"gy\":") &&
         appendScaledFloat(dst, cap, len, float(t.motion.angularRateDps[1]), 4) &&
         appendLit(dst, cap, len, ",\"gz\":") &&
         appendScaledFloat(dst, cap, len, float(t.motion.angularRateDps[2]), 4) &&
         appendLit(dst, cap, len, ",\"peakAcceleration\":") &&
         appendScaledFloat(dst, cap, len, float(t.motion.peakAccelerationG), 6) &&
         appendLit(dst, cap, len, ",\"peakAngularRate\":") &&
         appendScaledFloat(dst, cap, len, float(t.motion.peakAngularRateDps), 4) &&
         appendLit(dst, cap, len, ",\"sampleCount\":") &&
         appendU32(dst, cap, len, uint16_t(t.motion.sampleCount));
    if (ok && (t.live.flags & lil::protocol::kMotionFeedbackPresent)) {
      ok = appendLit(dst, cap, len, ",\"steps\":") &&
           appendU32(dst, cap, len, uint16_t(t.motionFeedback.steps)) &&
           appendLit(dst, cap, len, ",\"activeSeconds\":") &&
           appendU32(dst, cap, len, uint16_t(t.motionFeedback.activeSeconds));
    }
  }
  if (ok && (t.capabilities & lil::protocol::kHeartRate) &&
      (t.live.flags & lil::protocol::kLiveEstimateFresh)) {
    ok = appendLit(dst, cap, len, ",\"heartRate\":") &&
         appendScaledFloat(dst, cap, len, float(t.pulse.beatsPerMinute), 4);
  }
  ok = ok && appendLit(dst, cap, len, ",\"pulseStatus\":") &&
       appendU32(dst, cap, len, uint8_t(t.pulse.status)) &&
       appendLit(dst, cap, len, ",\"quality\":") &&
       appendU32(dst, cap, len, uint8_t(t.pulse.quality)) &&
       appendLit(dst, cap, len, ",\"battery\":") &&
       appendMilliU32(dst, cap, len, uint16_t(record.batteryMv)) &&
       appendLit(dst, cap, len, ",\"wave\":[");
  const uint8_t frameCount =
      std::min<uint8_t>(t.live.count, lil::protocol::kOpticalBatchSize);
  for (uint8_t j = 0; ok && j < frameCount; ++j) {
    ok = (!j || appendRawChar(dst, cap, len, ',')) &&
         appendLit(dst, cap, len, "{\"t\":") &&
         appendSigned(dst, cap, len, int64_t(sampleMs) - t.live.optical[j].ageMs) &&
         appendLit(dst, cap, len, ",\"red\":") &&
         appendU32(dst, cap, len, uint32_t(t.live.optical[j].red)) &&
         appendLit(dst, cap, len, ",\"infrared\":") &&
         appendU32(dst, cap, len, uint32_t(t.live.optical[j].infrared)) &&
         appendRawChar(dst, cap, len, '}');
  }
  return ok && appendLit(dst, cap, len, "]}");
}
}  // namespace station

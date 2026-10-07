#pragma once

#include <ArduinoJson.h>
#include "recording_archive.h"

namespace station {
// ArduinoJson's String/std::string serializers clear their destination. A
// writer appends instead, preserving the page header and preceding points.
template <typename Text>
class JsonAppendWriter {
 public:
  explicit JsonAppendWriter(Text& text) : text_(text) {}
  size_t write(uint8_t byte) { text_ += static_cast<char>(byte); return 1; }
  size_t write(const uint8_t* bytes, size_t size) {
    for (size_t i = 0; i < size; ++i) write(bytes[i]);
    return size;
  }
 private:
  Text& text_;
};

template <typename Text>
void appendRecordingJson(const JsonDocument& document, Text& text) {
  JsonAppendWriter<Text> writer(text);
  serializeJson(document, writer);
}

inline void recordingInfoJson(JsonObject object, const RecordingInfo& info) {
  char session[17];
  snprintf(session, sizeof(session), "%016llx", static_cast<unsigned long long>(info.session));
  object["session"] = session; object["epochMs"] = info.epochMs;
  object["durationMs"] = info.durationMs; object["expected"] = info.expected;
  object["stored"] = info.stored; object["complete"] = info.stored == info.expected;
  object["sensorType"] = static_cast<uint8_t>(info.type);
  object["availableMs"] = info.availableMs;
}

inline void recordingPointJson(JsonDocument& point, const lil::recording::Record& record) {
  const auto t = lil::recording::decode(record);
  point["sampleMs"] = record.sampleMs;
  point["t"] = int64_t(record.sampleMs) - (record.ageMs == UINT16_MAX ? 0 : record.ageMs);
  point["estimateT"] = int64_t(record.sampleMs) - (t.live.estimateAgeMs == UINT16_MAX ? 0 : t.live.estimateAgeMs);
  point["windowMs"] = record.windowMs;
  point["gap"] = bool(record.flags & lil::protocol::kLiveGap) || t.motion.fifoOverrun;
  if ((t.capabilities & lil::protocol::kTemperature) && (t.live.flags & lil::protocol::kLiveSampleFresh)) point["temperature"] = t.temperatureC;
  if (t.capabilities & lil::protocol::kMotion) {
    const char* keys[] = {"ax","ay","az","gx","gy","gz"};
    for (int axis = 0; axis < 6; ++axis) point[keys[axis]] = axis < 3 ? t.motion.accelerationG[axis] : t.motion.angularRateDps[axis-3];
    point["peakAcceleration"] = t.motion.peakAccelerationG; point["peakAngularRate"] = t.motion.peakAngularRateDps;
    point["sampleCount"] = t.motion.sampleCount;
    if (t.live.flags & lil::protocol::kMotionFeedbackPresent) {
      point["steps"] = t.motionFeedback.steps;
      point["activeSeconds"] = t.motionFeedback.activeSeconds;
    }
  }
  if ((t.capabilities & lil::protocol::kHeartRate) && (t.live.flags & lil::protocol::kLiveEstimateFresh)) point["heartRate"] = t.pulse.beatsPerMinute;
  point["pulseStatus"] = t.pulse.status; point["quality"] = t.pulse.quality;
  point["battery"] = record.batteryMv / 1000.0F;
  auto wave = point["wave"].to<JsonArray>();
  for (uint8_t j = 0; j < t.live.count; ++j) {
    auto frame = wave.add<JsonObject>(); frame["t"] = int64_t(record.sampleMs) - t.live.optical[j].ageMs;
    frame["red"] = t.live.optical[j].red; frame["infrared"] = t.live.optical[j].infrared;
  }
}
}  // namespace station

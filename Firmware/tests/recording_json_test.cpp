#include "recording_json.h"
#include <cassert>
#include <string>
#include <vector>
#include <new>

struct CountedAppend {
  std::string& text;
  size_t calls = 0, failAt = 0;
  bool concat(const char* data, size_t size) {
    if (++calls == failAt) return false;
    text.append(data, size);
    return true;
  }
};

void testRadioTick() { ++testMillis; }
int main() {
  using namespace lil;
  {
    JsonDocument doc;
    doc["escaped"] = std::string(132, '"') + "\n\\";
    std::string expected;
    serializeJson(doc, expected);
    std::string output = "prefix";
    CountedAppend append{output};
    assert(station::appendJson(doc, append));
    assert(output == "prefix" + expected && append.calls == (expected.size() + 63) / 64);
    output = "prefix";
    append.calls = 0; append.failAt = 2;
    assert(!station::appendJson(doc, append));
    assert(output == "prefix" + expected.substr(0, 64) && append.calls == 2);
  }
  {
    station::RecordingInfo info{};
    info.stored = info.expected = 10;
    info.readError = true;
    JsonDocument doc;
    station::recordingInfoJson(doc.to<JsonObject>(), info);
    assert(doc["readError"].as<bool>() && !doc["complete"].as<bool>());
  }
  for (auto type : {protocol::EnvironmentalSensorType::kTmp117,
                   protocol::EnvironmentalSensorType::kLsm6dsox,
                   protocol::EnvironmentalSensorType::kMax30102}) {
    for (unsigned count : {0U, 1U, 16U, 128U}) {
      station::RecordingInfo info{42, 1700000000000ULL, count, count, count*1000, type};
      JsonDocument metadata; station::recordingInfoJson(metadata.to<JsonObject>(), info);
      metadata["nextOffset"] = count;
      std::string chunk; serializeJson(metadata, chunk); chunk.pop_back(); chunk += ",\"points\":[";
      CountedAppend append{chunk};
      std::string response; unsigned flushes = 0;
      size_t pointBytes = 0;
      for (unsigned i = 0; i < count; ++i) {
        protocol::TelemetryPayload payload{};
        payload.sensorType = type; payload.batteryMillivolts = 3900;
        payload.live.flags = protocol::kLiveTimingKnown | protocol::kLiveSampleFresh | protocol::kLiveEstimateFresh;
        payload.live.acquisitionAgeMs = 5; payload.live.estimateAgeMs = 10;
        if (type == protocol::EnvironmentalSensorType::kTmp117) {
          payload.capabilities = protocol::kTemperature; payload.temperatureC = 20 + i/128.0F;
        } else if (type == protocol::EnvironmentalSensorType::kLsm6dsox) {
          payload.capabilities = protocol::kMotion;
          payload.motion.accelerationG[0] = i/100.0F; payload.motion.accelerationG[2] = 1;
          payload.motion.angularRateDps[0] = i;
        } else {
          payload.capabilities = protocol::kOptical | protocol::kHeartRate;
          payload.pulse.beatsPerMinute = 75; payload.live.count = 2;
          payload.live.optical[0] = {123456, 234567, 40};
          payload.live.optical[1] = {123457, 234568, 0};
        }
        const auto record = recording::encode(info.session, i*1000, payload);
        JsonDocument point;
        // A packed record is valid at any byte address. Exercise all scalar
        // alignments against the actual ArduinoJson reference-taking API.
        alignas(4) unsigned char storage[sizeof(record) + 3];
        for (size_t offset = 0; offset < 4; ++offset) {
          const auto* packed = new (storage + offset) recording::Record(record);
          station::recordingPointJson(point, *packed);
          assert(point["sampleMs"].as<unsigned>() == i * 1000);
        }
        if (i) chunk += ',';
        std::string expected;
        serializeJson(point, expected);
        const size_t start = chunk.size();
        assert(station::appendJson(point, append));
        assert(chunk.compare(start, expected.size(), expected) == 0 && chunk.size() - start == expected.size());
        pointBytes += expected.size();
        if (chunk.size() >= 4096) { response += chunk; chunk.clear(); ++flushes; }
      }
      chunk += "]}"; response += chunk;
      JsonDocument parsed; assert(!deserializeJson(parsed, response));
      assert(parsed["session"] == "000000000000002a");
      assert(parsed["nextOffset"].as<unsigned>() == count);
      assert(parsed["points"].size() == count);
      if (count == 128) {
        assert(flushes > 1 && append.calls < pointBytes / 32);
        printf("Recording type %u: %zu bytewise string appends reduced to %zu buffered appends\n",
               unsigned(type), pointBytes, append.calls);
      }
      for (unsigned i = 0; i < count; ++i) {
        auto point = parsed["points"][i];
        assert(point["sampleMs"].as<unsigned>() == i*1000);
        assert(point["t"].as<int64_t>() == int64_t(i*1000)-5);
        if (type == protocol::EnvironmentalSensorType::kTmp117) assert(fabsf(point["temperature"].as<float>() - (20+i/128.0F)) < .0001F);
        else if (type == protocol::EnvironmentalSensorType::kLsm6dsox) assert(fabsf(point["ax"].as<float>()-i/100.0F) < .0006F);
        else { assert(point["wave"].size() == 2); assert(point["heartRate"] == 75); }
      }
    }
  }
  puts("Recording JSON: real ArduinoJson serializer, complete chunked pages and all three sensor types passed");
}

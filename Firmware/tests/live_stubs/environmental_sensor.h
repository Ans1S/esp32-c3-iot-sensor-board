#pragma once
#include "Arduino.h"
#include "environmental_reading.h"
#include <vector>
#include <cassert>
namespace sensor {
class PowerController {};
class EnvironmentalSensor {
 public:
  std::vector<uint32_t> starts;
  uint32_t conversionStarted = 0, consumed = 0;
  int resets = 0, ends = 0;
  unsigned polls = 0, reads = 0;
  std::vector<uint32_t> pollTimes;
  bool powered = false;
  bool beginFails = false, readingFails = false;
  lil::protocol::EnvironmentalSensorType type{};
  bool begin(PowerController&, lil::protocol::EnvironmentalSensorType selected, float) {
    assert(!powered); powered = true; type = selected; starts.push_back(millis());
    // Model the existing checked IMU initialization and transient discard.
    delay(type == lil::protocol::EnvironmentalSensorType::kLsm6dsox ? 125 : 17);
    conversionStarted = millis(); consumed = 0; return !beginFails;
  }
  void end() { powered = false; ++ends; }
  void resetMotionFeedback() { ++resets; }
  bool pollLive() { assert(powered); ++polls; pollTimes.push_back(millis()); return true; }
  bool freshTemperature() const {
    return powered && millis() - conversionStarted >= 124 + consumed * 1000;
  }
  EnvironmentalReading read(bool pollFirst = true) {
    assert(powered);
    if (pollFirst) pollLive();
    ++reads; // Both real precision read paths support a checked snapshot.
    EnvironmentalReading reading{}; reading.sensorType = type;
    reading.live.flags = lil::protocol::kLiveTimingKnown | lil::protocol::kLiveSampleFresh |
        lil::protocol::kMotionFeedbackPresent;
    if (type == lil::protocol::EnvironmentalSensorType::kTmp117) {
      reading.valid = freshTemperature();
      reading.temperatureC = 25.0078125F;
      reading.capabilities = reading.valid ? lil::protocol::kTemperature : 0;
      reading.live.windowMs = 124; ++consumed;
    } else {
      reading.valid = millis() - conversionStarted >= 50;
      reading.capabilities = reading.valid ? lil::protocol::kMotion : 0;
      reading.live.windowMs = millis() - conversionStarted;
      reading.motion.sampleCount = reading.live.windowMs * 104 / 1000;
      conversionStarted = millis();
    }
    if (readingFails) reading.valid = false;
    return reading;
  }
};
}

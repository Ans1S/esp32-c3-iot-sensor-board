// Execute the production acquisition loop with a deterministic RTOS/driver clock.
#include "Arduino.h"
#include "live_acquisition.h"
#include "environmental_sensor.h"
#include <cassert>
#include <functional>
#include <vector>

struct Finished {};
sensor::LiveAcquisition* current = nullptr;
uint32_t deadline = 0;
std::function<void()> steer;
std::vector<sensor::LiveCapture> captures;
void testRadioTick() {
  ++testMillis;
  sensor::LiveCapture capture{};
  while (current && current->take(capture)) captures.push_back(capture);
  if (steer) steer();
  if (testMillis >= deadline) throw Finished{};
}
void run(sensor::LiveAcquisition& acquisition, uint32_t until) {
  current = &acquisition; deadline = until;
  try { testAcquisitionEntry(testAcquisitionContext); } catch (const Finished&) {}
  current = nullptr;
}
int main() {
  using Type = lil::protocol::EnvironmentalSensorType;
  assert(lil::timing::normalPrecisionIntervalMs(0) == 1000);
  assert(lil::timing::normalPrecisionIntervalMs(100000) == 86400000);
  lil::timing::PrecisionCadence wrap;
  wrap.reset(UINT32_MAX - 50); wrap.started(UINT32_MAX - 50, 1000);
  assert(!wrap.due(948) && wrap.due(949));
  for (auto type : {Type::kTmp117, Type::kLsm6dsox}) {
    for (uint32_t seconds : {1,10}) {
      testMillis = 5000; testButtonLow = false; captures.clear(); steer = {};
      sensor::EnvironmentalSensor sensor; sensor::PowerController power;
      sensor::LiveAcquisition acquisition;
      assert(acquisition.begin(sensor,power,type,0)); acquisition.normal(seconds);
      run(acquisition,seconds == 1 ? 8300 : 25400);
      assert(captures.size() == (seconds == 1 ? 4 : 3));
      assert(sensor.starts.size() == captures.size() && !sensor.powered);
      for (size_t i = 0; i < captures.size(); ++i) {
        assert(captures[i].reading.valid && !captures[i].recording);
        assert(!(captures[i].reading.live.flags & lil::protocol::kMotionFeedbackPresent));
        if (i) assert(sensor.starts[i] - sensor.starts[i-1] >= seconds * 1000 &&
            sensor.starts[i] - sensor.starts[i-1] <= seconds * 1000 + 5);
        assert(captures[i].capturedMs - sensor.starts[i] >= (type == Type::kTmp117 ? 141 : 225));
      }
    }
  }
  // A changed normal interval cannot slow down a button recording. Stopping
  // resumes fresh normal measurements and leaves no automatic journal data.
  testMillis = 100; testButtonLow = false; captures.clear();
  sensor::EnvironmentalSensor sensor; sensor::PowerController power;
  sensor::LiveAcquisition acquisition;
  assert(acquisition.begin(sensor,power,Type::kLsm6dsox,0)); acquisition.normal(1);
  steer = [&]() {
    if (testMillis == 1000) acquisition.start();
    if (testMillis == 1200) acquisition.setNormalInterval(10);
    if (testMillis == 2200) { acquisition.stop(); acquisition.normal(10); }
  };
  run(acquisition,13000);
  size_t recorded = 0, normal = 0;
  uint64_t previousRecord = 0;
  for (const auto& capture : captures) {
    if (capture.recording) {
      ++recorded;
      assert(capture.reading.live.flags & lil::protocol::kMotionFeedbackPresent);
      if (previousRecord) assert(capture.capturedMs - previousRecord <= 55);
      previousRecord = capture.capturedMs;
    } else ++normal;
  }
  assert(recorded >= 18 && normal == 3 && !sensor.powered);
  // A button press pauses even automatic acquisition until the main task
  // chooses the recording profile. Held presses generate exactly one event.
  testMillis = 100; testButtonLow = false; captures.clear();
  sensor::EnvironmentalSensor sensor2; sensor::LiveAcquisition acquisition2;
  assert(acquisition2.begin(sensor2,power,Type::kTmp117,0)); acquisition2.normal(1);
  steer = [&]() {
    if (testMillis == 300) testButtonLow = true;
    if (testMillis == 500) { assert(acquisition2.presses() == 1); acquisition2.start(); }
    if (testMillis == 2000) { acquisition2.stop(); acquisition2.normal(10); }
    if (testMillis == 2200) testButtonLow = false;
  };
  run(acquisition2,2600);
  assert(acquisition2.presses() == 1);
  assert(captures.size() == 4 && !captures[0].recording && captures[1].recording &&
      captures[2].recording && !captures[3].recording);
  // Each failed normal window reports one current gap, without silently
  // retaining stale readings or using the manual fast retry cadence.
  for (bool startupFault : {false,true}) {
    testMillis = 5000; testButtonLow = false; captures.clear(); steer = {};
    sensor::EnvironmentalSensor faulty; sensor::LiveAcquisition faultAcquisition;
    faulty.beginFails = startupFault; faulty.readingFails = !startupFault;
    assert(faultAcquisition.begin(faulty,power,Type::kTmp117,0)); faultAcquisition.normal(10);
    run(faultAcquisition,25400);
    assert(captures.size() == 3 && faulty.starts.size() == 3 && !faulty.powered);
    for (const auto& capture : captures) {
      assert(!capture.recording && !capture.reading.valid);
      assert(capture.reading.sensorType == Type::kTmp117);
      assert(capture.reading.live.acquisitionAgeMs == 0);
      assert((capture.reading.live.flags & (lil::protocol::kLiveGap | lil::protocol::kLiveTimingKnown)) ==
          (lil::protocol::kLiveGap | lil::protocol::kLiveTimingKnown));
    }
    assert(faulty.starts[1] - faulty.starts[0] >= 10000 &&
        faulty.starts[2] - faulty.starts[1] >= 10000);
  }
  // Stop during the yielding startup delay. The rail must be released before
  // any polling/read/report of the profile which was cancelled.
  testMillis = 100; testButtonLow = false; captures.clear();
  sensor::EnvironmentalSensor cancelled; sensor::LiveAcquisition cancelledAcquisition;
  assert(cancelledAcquisition.begin(cancelled,power,Type::kLsm6dsox,0));
  cancelledAcquisition.start();
  steer = [&]() {
    if (testMillis == 150) cancelledAcquisition.stop();
    if (testMillis == 226) assert(!cancelled.powered && !cancelledAcquisition.active());
  };
  run(cancelledAcquisition,400);
  assert(captures.empty() && cancelled.starts.size() == 1 && !cancelled.powered);
  // MAX30102 is explicitly manual. An idle node cannot accumulate eight
  // seconds of pulse data, and a configured interval must not start its LEDs.
  testMillis = 100; testButtonLow = false; captures.clear(); steer = {};
  sensor::EnvironmentalSensor optical; sensor::LiveAcquisition opticalAcquisition;
  assert(opticalAcquisition.begin(optical,power,Type::kMax30102,0));
  opticalAcquisition.normal(1);
  run(opticalAcquisition,1000);
  assert(optical.starts.empty() && captures.empty() && !optical.powered);
  opticalAcquisition.start();
  run(opticalAcquisition,12000);
  assert(optical.starts.size() == 1 && optical.powered && captures.size() >= 50);
  for (const auto& capture : captures) assert(capture.recording);
  opticalAcquisition.stop();
  run(opticalAcquisition,12100);
  assert(!optical.powered);
  puts("Production acquisition: fresh windows, rail shutdown, manual optical continuity, recording independence and SW2 debounce passed");
}

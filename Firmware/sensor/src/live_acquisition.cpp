#include "live_acquisition.h"
#include "environmental_sensor.h"
#include <esp_timer.h>
#include "recording_button.h"

namespace sensor {
bool LiveAcquisition::begin(EnvironmentalSensor& sensor, PowerController& power,
    lil::protocol::EnvironmentalSensorType type, float offset) {
  if (!lil::protocol::isLiveSensor(type)) return false;
  sensor_ = &sensor; power_ = &power; type_ = type; offset_ = offset;
  sensor_->end();
  queue_ = xQueueCreate(64, sizeof(LiveCapture));
  if (!queue_) return false;
  if (xTaskCreate(entry, "live-acquire", 6144, this, 2, &task_) == pdPASS) return true;
  vQueueDelete(queue_); queue_ = nullptr; return false;
}
void LiveAcquisition::entry(void* context) { static_cast<LiveAcquisition*>(context)->loop(); }
void LiveAcquisition::loop() {
  // SW2 on both PCB V3 and V4 shorts GPIO9 to GND. Require a release after boot.
  pinMode(9, INPUT_PULLUP);
  lil::recording::Button button; button.begin(digitalRead(9) == LOW, millis());
  uint32_t lastReport = 0, retryMs = 0, sessionStarted = 0;
  uint8_t failures = 0;
  bool gap = false;
  Profile previous = Profile::Stopped;
  lil::timing::PrecisionCadence normalCadence;
  uint32_t configuredInterval = normalIntervalMs_.load();
  auto enqueue = [&](LiveCapture& capture) {
    if (!capture.reading.valid) {
      capture.reading.live.flags |= lil::protocol::kLiveTimingKnown | lil::protocol::kLiveGap;
      capture.reading.live.acquisitionAgeMs = 0;
    }
    if (gap) capture.reading.live.flags |= lil::protocol::kLiveGap;
    if (xQueueSend(queue_, &capture, 0) == pdTRUE) gap = false;
    else { dropped_.fetch_add(1); gap = true; }
  };
  for (;;) {
    uint32_t now = millis();
    if (button.poll(digitalRead(9) == LOW, now)) {
      // Stop acquisition at the debounced press even if the network is busy.
      requested_.store(Profile::Stopped);
      presses_.fetch_add(1);
    }
    const Profile profile = requested_.load();
    const bool normal = profile == Profile::Normal;
    if (profile != previous) {
      if (active_.load()) { sensor_->end(); active_.store(false); }
      sessionStarted = 0; retryMs = now;
      if (normal) normalCadence.reset(now);
      previous = profile;
    }
    const uint32_t selectedInterval = normalIntervalMs_.load();
    if (selectedInterval != configuredInterval) {
      configuredInterval = selectedInterval;
      normalCadence.reset(now);
    }
    if (profile == Profile::Stopped || (normal && normalPaused_.load())) {
      if (active_.load()) { sensor_->end(); active_.store(false); }
      sessionStarted = 0;
      delay(5); continue;
    }
    if (!active_.load()) {
      if (normal && !normalCadence.due(now)) { delay(5); continue; }
      if (static_cast<int32_t>(now - retryMs) < 0) { delay(5); continue; }
      if (normal || !sessionStarted) sensor_->resetMotionFeedback();
      // Include startup in active() so the main task can wait for I2C and rail
      // ownership to be released before entering battery protection. Recheck
      // the request after publishing active to close the stop/start race.
      active_.store(true);
      if (requested_.load() != profile || (normal && normalPaused_.load())) {
        active_.store(false); continue;
      }
      if (normal) normalCadence.started(now, configuredInterval);
      if (!sensor_->begin(*power_, type_, offset_)) {
        sensor_->end(); active_.store(false);
        gap = true;
        if (normal) {
          LiveCapture capture{};
          capture.reading.sensorType = type_;
          capture.capturedMs = esp_timer_get_time() / 1000;
          enqueue(capture);
          // A failed window reports its fault once. Long bounded startup
          // failures must not create an immediate series of retry windows.
          if (normalCadence.due(millis())) normalCadence.started(millis(), configuredInterval);
        }
        retryMs = normal ? millis() : millis() + 1000;
        delay(5); continue;
      }
      // Initialization can yield to the main task for hundreds of ms. A stop
      // or profile change during that time must release the rail immediately.
      if (requested_.load() != profile || (normal && normalPaused_.load())) {
        sensor_->end(); active_.store(false); continue;
      }
      now = millis(); active_.store(true); lastReport = now; failures = 0;
      if (!sessionStarted) sessionStarted = lastReport;
    }
    sensor_->pollLive();
    const bool temperature = type_ == lil::protocol::EnvironmentalSensorType::kTmp117;
    const uint32_t interval = type_ == lil::protocol::EnvironmentalSensorType::kLsm6dsox ?
        (normal ? lil::timing::kImuReportMs : lil::timing::imuRecordingIntervalMs(now - sessionStarted)) :
        lil::protocol::liveReportIntervalMs(type_);
    if (temperature ? (!sensor_->freshTemperature() && now - lastReport < 1500) :
        now - lastReport < interval) { delay(5); continue; }
    lastReport = millis();
    LiveCapture capture{}; capture.reading = sensor_->read();
    if (requested_.load() != profile || (normal && normalPaused_.load())) {
      sensor_->end(); active_.store(false); continue;
    }
    capture.capturedMs = esp_timer_get_time() / 1000;
    capture.recording = profile == Profile::Recording;
    if (normal) {
      // Sparse snapshots do not provide continuous raw-rate step feedback.
      capture.reading.live.flags &= ~lil::protocol::kMotionFeedbackPresent;
      sensor_->end(); active_.store(false);
    }
    enqueue(capture);
    failures = capture.reading.valid ? 0 : failures + 1;
    if (failures >= 3) { sensor_->end(); active_.store(false); retryMs = now + 1000; gap = true; }
    delay(1);
  }
}
}  // namespace sensor

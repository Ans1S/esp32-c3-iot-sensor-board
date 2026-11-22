#include "live_acquisition.h"
#include "environmental_sensor.h"
#include <esp_timer.h>
#include "recording_button.h"

namespace sensor {
namespace {
#ifdef ESP_PLATFORM
void IRAM_ATTR sw2EdgeIsr(void* arg) {
  auto* task = static_cast<TaskHandle_t>(arg);
  if (task == nullptr) return;
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(task, &higherPriorityTaskWoken);
  if (higherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}
#endif
}  // namespace

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
#ifdef ESP_PLATFORM
  attachInterruptArg(digitalPinToInterrupt(9), sw2EdgeIsr,
                     xTaskGetCurrentTaskHandle(), CHANGE);
#endif
  lil::recording::Button button; button.begin(digitalRead(9) == LOW, millis());
  uint32_t lastReport = 0, lastImuPoll = 0, sessionStarted = 0, nextTempPollMs = 0;
  bool sessionActive = false, tempPollBackoff = false;
  lil::timing::RetryBackoff retry;
  uint8_t failures = 0;
  bool gap = false;
  Profile previous = Profile::Stopped;
  lil::timing::PrecisionCadence normalCadence;
  uint32_t configuredInterval = normalIntervalMs_.load();
  auto publishNormalCadence = [&](bool waiting, uint32_t started, uint32_t interval) {
    normalStartedMs_.store(started, std::memory_order_relaxed);
    normalDueIntervalMs_.store(interval, std::memory_order_relaxed);
    normalWaiting_.store(waiting, std::memory_order_release);
  };
  auto idleWait = [&](uint32_t maxMs) {
#ifdef ESP_PLATFORM
    if (button.debouncing() || digitalRead(9) == LOW) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
      return;
    }
    const uint32_t clamped = maxMs < 5UL ? 5UL : (maxMs > 1000UL ? 1000UL : maxMs);
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(clamped));
#else
    (void)maxMs;
    delay(5);
#endif
  };
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
      sessionActive = false; sessionStarted = 0; retry.clear();
      tempPollBackoff = false;
      if (profile == Profile::Recording) gap = false;
      if (normal) {
        normalCadence.reset(now);
        publishNormalCadence(false, now, 0);
      } else {
        publishNormalCadence(false, now, 0);
      }
      previous = profile;
    }
    const uint32_t selectedInterval = normalIntervalMs_.load();
    if (selectedInterval != configuredInterval) {
      configuredInterval = selectedInterval;
      normalCadence.reset(now);
      publishNormalCadence(false, now, 0);
    }
    if (profile == Profile::Stopped || (normal && normalPaused_.load())) {
      if (active_.load()) { sensor_->end(); active_.store(false); }
      sessionActive = false; sessionStarted = 0;
      tempPollBackoff = false;
      idleWait(100); continue;
    }
    if (!active_.load()) {
      if (normal && !normalCadence.due(now)) {
        idleWait(normalCadence.remainingMs(now));
        continue;
      }
      if (!retry.ready(now)) {
        idleWait(retry.remainingMs(now));
        continue;
      }
      if (normal || !sessionActive) sensor_->resetMotionFeedback();
      // Include startup in active() so the main task can wait for I2C and rail
      // ownership to be released before entering battery protection. Recheck
      // the request after publishing active to close the stop/start race.
      active_.store(true);
      if (requested_.load() != profile || (normal && normalPaused_.load())) {
        active_.store(false); continue;
      }
      if (normal) {
        normalCadence.started(now, configuredInterval);
        publishNormalCadence(true, now, configuredInterval);
      }
      if (!sensor_->begin(*power_, type_, offset_)) {
        sensor_->end(); active_.store(false);
        gap = true;
        if (normal) {
          LiveCapture capture{};
          capture.reading.sensorType = type_;
          capture.capturedMs = lil::timing::signedUsToMs(esp_timer_get_time());
          enqueue(capture);
          // A failed window reports its fault once. Long bounded startup
          // failures must not create an immediate series of retry windows.
          const uint32_t failNow = millis();
          if (normalCadence.due(failNow)) {
            normalCadence.started(failNow, configuredInterval);
            publishNormalCadence(true, failNow, configuredInterval);
          }
        }
        if (!normal) retry.wait(millis(), 1000);
        idleWait(5); continue;
      }
      // Initialization can yield to the main task for hundreds of ms. A stop
      // or profile change during that time must release the rail immediately.
      if (requested_.load() != profile || (normal && normalPaused_.load())) {
        sensor_->end(); active_.store(false); continue;
      }
      now = millis(); active_.store(true); lastReport = now; lastImuPoll = now; failures = 0;
      if (type_ == lil::protocol::EnvironmentalSensorType::kTmp117) {
        // TMP117 8-average conversion takes ~124 ms; avoid dead I2C polls in the first 100 ms.
        nextTempPollMs = now + 100;
        tempPollBackoff = true;
      } else {
        tempPollBackoff = false;
      }
      if (!sessionActive) { sessionStarted = lastReport; sessionActive = true; }
    }
    const bool motion = type_ == lil::protocol::EnvironmentalSensorType::kLsm6dsox;
    const bool temperature = type_ == lil::protocol::EnvironmentalSensorType::kTmp117;
    const bool optical = type_ == lil::protocol::EnvironmentalSensorType::kMax30102;
    const uint32_t interval = motion ?
        (normal ? lil::timing::kImuReportMs : lil::timing::imuRecordingIntervalMs(now - sessionStarted)) :
        (temperature ? 1500 : lil::protocol::liveReportIntervalMs(type_));
    const bool reportDue = now - lastReport >= interval;
    if (tempPollBackoff && static_cast<int32_t>(now - nextTempPollMs) >= 0) {
      tempPollBackoff = false;
    }
    // At 104 Hz per sensor a 100 ms IMU report window fills at most 22 of 512
    // FIFO words, and a 200 ms MAX30102 report window fills 5 of 32 frames.
    // Drain IMU and optical FIFOs at report time instead of polling at 200 Hz.
    // During 100 ms IMU recording windows (after 5 min), also drain at mid-window
    // (50 ms) so a flash sector erase on the main task cannot push the poll gap
    // past the driver's 200 ms staleness guard.
    const bool imuMidPoll = motion && !normal &&
        interval > lil::timing::kImuShortSessionMs &&
        now - lastImuPoll >= lil::timing::kImuShortSessionMs;
    const bool shouldPoll = motion ? (reportDue || imuMidPoll)
                                   : (optical ? reportDue : (!tempPollBackoff || reportDue));
    if (shouldPoll) {
      if (motion) lastImuPoll = now;
      const bool pollOk = sensor_->pollLive();
      if (!pollOk && !sensor_->liveInitialized()) {
        sensor_->end(); active_.store(false);
        gap = true;
        if (normal) {
          LiveCapture capture{};
          capture.reading.sensorType = type_;
          capture.capturedMs = lil::timing::signedUsToMs(esp_timer_get_time());
          enqueue(capture);
          const uint32_t failNow = millis();
          if (normalCadence.due(failNow)) {
            normalCadence.started(failNow, configuredInterval);
            publishNormalCadence(true, failNow, configuredInterval);
          }
        } else {
          retry.wait(now, 1000);
        }
        idleWait(5); continue;
      }
    }
    if (!reportDue && !(temperature && sensor_->freshTemperature())) {
      uint32_t waitMs = interval - (now - lastReport);
      if (motion && !normal && interval > lil::timing::kImuShortSessionMs) {
        const uint32_t midElapsed = now - lastImuPoll;
        const uint32_t midRem = midElapsed >= lil::timing::kImuShortSessionMs
                                    ? 5UL
                                    : lil::timing::kImuShortSessionMs - midElapsed;
        if (midRem < waitMs) waitMs = midRem;
      }
      if (temperature) {
        if (tempPollBackoff) {
          const int32_t pollRem = static_cast<int32_t>(nextTempPollMs - now);
          if (pollRem > 0 && static_cast<uint32_t>(pollRem) < waitMs) {
            waitMs = static_cast<uint32_t>(pollRem);
          }
        } else {
          waitMs = 5;
        }
      }
      idleWait(waitMs);
      continue;
    }
    lastReport = motion ? now : millis();
    // pollLive() just drained/checked the driver under this task's ownership.
    // Read that snapshot without issuing a second status/FIFO transaction.
    LiveCapture capture{}; capture.reading = sensor_->read(false);
    if (requested_.load() != profile || (normal && normalPaused_.load())) {
      sensor_->end(); active_.store(false); continue;
    }
    capture.capturedMs = lil::timing::signedUsToMs(esp_timer_get_time());
    capture.recording = profile == Profile::Recording;
    if (temperature && capture.reading.valid && !normal) {
      // Next TMP117 1 Hz conversion completes in ~1000 ms; back off I2C polling.
      nextTempPollMs = millis() + 850;
      tempPollBackoff = true;
    }
    if (normal) {
      // Sparse snapshots do not provide continuous raw-rate step feedback.
      capture.reading.live.flags &= ~lil::protocol::kMotionFeedbackPresent;
      sensor_->end(); active_.store(false);
    }
    enqueue(capture);
    failures = capture.reading.valid ? 0 : failures + 1;
    if (failures >= 3) { sensor_->end(); active_.store(false); retry.wait(now, 1000); gap = true; }
    delay(1);
  }
}
}  // namespace sensor

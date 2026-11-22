#pragma once
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "environmental_reading.h"
#include "precision_cadence.h"

namespace sensor {
class EnvironmentalSensor;
class PowerController;
struct LiveCapture {
  uint64_t capturedMs = 0;
  EnvironmentalReading reading{};
  bool recording = false;
};
// This task is the sole owner of I2C during normal and manual live sessions. Radio waits
// and station replay never call the drivers or determine their sampling clock.
class LiveAcquisition {
 public:
  bool begin(EnvironmentalSensor& sensor, PowerController& power,
             lil::protocol::EnvironmentalSensorType type, float offset);
  void start() {
    dropped_.store(0);
    requested_.store(Profile::Recording);
    notifyTask();
  }
  void stop() {
    requested_.store(Profile::Stopped);
    notifyTask();
  }
  void normal(uint32_t seconds) {
    normalIntervalMs_.store(lil::timing::normalPrecisionIntervalMs(seconds));
    if (lil::timing::supportsNormalPrecisionMeasurements(type_)) {
      requested_.store(Profile::Normal);
      notifyTask();
    }
  }
  void setNormalInterval(uint32_t seconds) {
    normalIntervalMs_.store(lil::timing::normalPrecisionIntervalMs(seconds));
    notifyTask();
  }
  bool normal() const { return requested_.load() == Profile::Normal; }
  void pauseNormal(bool paused) {
    normalPaused_.store(paused);
    notifyTask();
  }
  bool active() const { return active_.load(); }
  bool take(LiveCapture& capture) { return xQueueReceive(queue_, &capture, 0) == pdTRUE; }
  uint32_t presses() const { return presses_.load(); }
  uint32_t dropped() const { return dropped_.load(); }
  uint32_t nextNormalDueMs(uint32_t now) const {
    if (!normalWaiting_.load(std::memory_order_acquire)) {
      return requested_.load(std::memory_order_relaxed) == Profile::Normal ? 10UL : 5000UL;
    }
    const uint32_t started = normalStartedMs_.load(std::memory_order_relaxed);
    const uint32_t interval = normalDueIntervalMs_.load(std::memory_order_relaxed);
    const uint32_t elapsed = static_cast<uint32_t>(now - started);
    return elapsed >= interval ? 10UL : interval - elapsed;
  }
 private:
  enum class Profile : uint8_t { Stopped, Normal, Recording };
  void notifyTask() {
#ifdef ESP_PLATFORM
    if (task_ != nullptr) xTaskNotifyGive(task_);
#endif
  }
  static void entry(void* context);
  void loop();
  EnvironmentalSensor* sensor_ = nullptr;
  PowerController* power_ = nullptr;
  lil::protocol::EnvironmentalSensorType type_{};
  float offset_ = 0;
  QueueHandle_t queue_ = nullptr;
  TaskHandle_t task_ = nullptr;
  std::atomic<Profile> requested_{Profile::Stopped};
  std::atomic<bool> active_{false}, normalPaused_{false};
  std::atomic<uint32_t> normalIntervalMs_{600000};
  std::atomic<uint32_t> normalStartedMs_{0}, normalDueIntervalMs_{0};
  std::atomic<bool> normalWaiting_{false};
  std::atomic<uint32_t> presses_{0}, dropped_{0};
};
}  // namespace sensor

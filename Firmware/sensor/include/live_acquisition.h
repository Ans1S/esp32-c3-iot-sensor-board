#pragma once
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "environmental_sensor.h"

namespace sensor {
struct LiveCapture { uint64_t capturedMs = 0; EnvironmentalReading reading{}; };
// This task is the sole owner of I2C during manual live sessions. Radio waits
// and station replay never call the drivers or determine their sampling clock.
class LiveAcquisition {
 public:
  bool begin(EnvironmentalSensor& sensor, PowerController& power,
             lil::protocol::EnvironmentalSensorType type, float offset);
  void start() { dropped_.store(0); requested_.store(true); }
  void stop() { requested_.store(false); }
  bool active() const { return active_.load(); }
  bool take(LiveCapture& capture) { return xQueueReceive(queue_, &capture, 0) == pdTRUE; }
  uint32_t presses() const { return presses_.load(); }
  uint32_t dropped() const { return dropped_.load(); }
 private:
  static void entry(void* context);
  void loop();
  EnvironmentalSensor* sensor_ = nullptr;
  PowerController* power_ = nullptr;
  lil::protocol::EnvironmentalSensorType type_{};
  float offset_ = 0;
  QueueHandle_t queue_ = nullptr;
  TaskHandle_t task_ = nullptr;
  std::atomic<bool> requested_{false}, active_{false};
  std::atomic<uint32_t> presses_{0}, dropped_{0};
};
}  // namespace sensor

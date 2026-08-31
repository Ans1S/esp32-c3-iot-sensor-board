#include "sleep_controller.h"

#include <WiFi.h>
#include <esp_sleep.h>
#include <esp_wifi.h>

namespace sensor {

[[noreturn]] void SleepController::deepSleep(uint32_t seconds,
                                             PowerController& power,
                                             uint32_t additionalMilliseconds) {
  power.prepareForDeepSleep();
  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  Serial.flush();

  const uint64_t sleepMicroseconds =
      static_cast<uint64_t>(max(seconds, 1UL)) * 1000000ULL +
      static_cast<uint64_t>(additionalMilliseconds) * 1000ULL;
  esp_sleep_enable_timer_wakeup(sleepMicroseconds);
  esp_deep_sleep_start();
  __builtin_unreachable();
}

}  // namespace sensor

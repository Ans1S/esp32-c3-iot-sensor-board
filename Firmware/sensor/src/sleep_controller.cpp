#include "sleep_controller.h"

#include <WiFi.h>
#include <esp_sleep.h>

#include "sensor_log.h"
#include "logical_clock.h"

namespace sensor {

[[noreturn]] void SleepController::deepSleep(uint32_t seconds,
                                             PowerController& power,
                                             uint32_t additionalMilliseconds) {
  power.prepareForDeepSleep();
  if (WiFi.getMode() != WIFI_OFF) {
    WiFi.mode(WIFI_OFF);
  }
  flushDiagnosticLogging();

  uint64_t sleepMicroseconds =
      static_cast<uint64_t>(seconds) * 1000000ULL +
      static_cast<uint64_t>(additionalMilliseconds) * 1000ULL;
  if (sleepMicroseconds == 0) {
    sleepMicroseconds = 1000000ULL;
  }
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
  esp_sleep_enable_timer_wakeup(sleepMicroseconds);
  accountForDeepSleep(sleepMicroseconds);
  esp_deep_sleep_start();
  __builtin_unreachable();
}

}  // namespace sensor

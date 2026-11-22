#include "logical_clock.h"
#include <esp_attr.h>
#include <esp_timer.h>
#ifdef ESP_PLATFORM
#include <esp_rtc_time.h>
#endif

namespace sensor {
namespace {
RTC_DATA_ATTR uint64_t elapsedBeforeBootUs = 0;
#ifdef ESP_PLATFORM
RTC_DATA_ATTR uint64_t lastLogicalUs = 0;
#endif
}
uint64_t logicalTimeUs() {
#ifdef ESP_PLATFORM
  // Use the continuous hardware RTC counter so bootloader startup, teardown,
  // and software resets do not accumulate timing drift or jump backwards.
  const uint64_t rtcUs = esp_rtc_get_time_us();
  uint64_t now = elapsedBeforeBootUs + rtcUs;
  if (now < lastLogicalUs) {
    elapsedBeforeBootUs = lastLogicalUs - rtcUs;
    now = lastLogicalUs;
  } else {
    lastLogicalUs = now;
  }
  return now;
#else
  return elapsedBeforeBootUs + static_cast<uint64_t>(esp_timer_get_time());
#endif
}
void accountForDeepSleep(uint64_t sleepMicroseconds) {
#ifdef ESP_PLATFORM
  // Hardware RTC keeps counting across deep sleep; anchor the current monotonic
  // watermark without double-counting sleepMicroseconds.
  (void)sleepMicroseconds;
  lastLogicalUs = logicalTimeUs();
#else
  elapsedBeforeBootUs = logicalTimeUs() + sleepMicroseconds;
#endif
}
}  // namespace sensor

#include "logical_clock.h"
#include <esp_attr.h>
#include <esp_timer.h>

namespace sensor {
namespace {
RTC_DATA_ATTR uint64_t elapsedBeforeBootUs = 0;
}
uint64_t logicalTimeUs() {
  return elapsedBeforeBootUs + static_cast<uint64_t>(esp_timer_get_time());
}
void accountForDeepSleep(uint64_t sleepMicroseconds) {
  elapsedBeforeBootUs = logicalTimeUs() + sleepMicroseconds;
}
}  // namespace sensor

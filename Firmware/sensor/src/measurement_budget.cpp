#include "measurement_budget.h"

#include <esp_timer.h>
#include "low_power_wait.h"

namespace sensor {
namespace {
int64_t deadlineUs = 0;
int64_t fetchDeadlineUs = 0;

int64_t effectiveDeadlineUs() {
  if (deadlineUs == 0) return fetchDeadlineUs;
  if (fetchDeadlineUs == 0) return deadlineUs;
  return deadlineUs < fetchDeadlineUs ? deadlineUs : fetchDeadlineUs;
}
}

void beginMeasurementBudget(uint32_t milliseconds) {
  fetchDeadlineUs = 0;
  deadlineUs = esp_timer_get_time() + static_cast<int64_t>(milliseconds) * 1000;
}

void endMeasurementBudget() { deadlineUs = 0; fetchDeadlineUs = 0; }

void beginFetchWindow() { fetchDeadlineUs = esp_timer_get_time() + 500000; }
void endFetchWindow() { fetchDeadlineUs = 0; }

bool measurementBudgetExpired() {
  const int64_t deadline = effectiveDeadlineUs();
  return deadline != 0 && esp_timer_get_time() >= deadline;
}

void measurementWaitUs(uint32_t microseconds) {
  const int64_t deadline = effectiveDeadlineUs();
  if (deadline != 0) {
    const int64_t remaining = deadline - esp_timer_get_time();
    if (remaining <= 0) return;
    if (remaining < microseconds) microseconds = static_cast<uint32_t>(remaining);
  }
  lowPowerSensorWaitUs(microseconds);
}
}  // namespace sensor

extern "C" bool wchargerSensorBudgetExpired() {
  return sensor::measurementBudgetExpired();
}
extern "C" void wchargerBeginFetchWindow() { sensor::beginFetchWindow(); }
extern "C" void wchargerEndFetchWindow() { sensor::endFetchWindow(); }

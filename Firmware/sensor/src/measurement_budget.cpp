#include "measurement_budget.h"

#ifdef ESP_PLATFORM
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#else
#include <atomic>
#endif
#include <esp_timer.h>
#include "low_power_wait.h"

namespace sensor {
namespace {
#ifdef ESP_PLATFORM
portMUX_TYPE budgetMux = portMUX_INITIALIZER_UNLOCKED;
int64_t deadlineUs = 0;
int64_t fetchDeadlineUs = 0;

int64_t effectiveDeadlineUs() {
  portENTER_CRITICAL(&budgetMux);
  const int64_t mainDeadline = deadlineUs;
  const int64_t fetchDeadline = fetchDeadlineUs;
  portEXIT_CRITICAL(&budgetMux);
  if (mainDeadline == 0) return fetchDeadline;
  if (fetchDeadline == 0) return mainDeadline;
  return mainDeadline < fetchDeadline ? mainDeadline : fetchDeadline;
}
#else
std::atomic<int64_t> deadlineUs{0};
std::atomic<int64_t> fetchDeadlineUs{0};

int64_t effectiveDeadlineUs() {
  const int64_t mainDeadline = deadlineUs.load(std::memory_order_acquire);
  const int64_t fetchDeadline = fetchDeadlineUs.load(std::memory_order_acquire);
  if (mainDeadline == 0) return fetchDeadline;
  if (fetchDeadline == 0) return mainDeadline;
  return mainDeadline < fetchDeadline ? mainDeadline : fetchDeadline;
}
#endif
}

void beginMeasurementBudget(uint32_t milliseconds) {
  const int64_t next =
      esp_timer_get_time() + static_cast<int64_t>(milliseconds) * 1000;
#ifdef ESP_PLATFORM
  portENTER_CRITICAL(&budgetMux);
  fetchDeadlineUs = 0;
  deadlineUs = next;
  portEXIT_CRITICAL(&budgetMux);
#else
  fetchDeadlineUs.store(0, std::memory_order_release);
  deadlineUs.store(next, std::memory_order_release);
#endif
}

void endMeasurementBudget() {
#ifdef ESP_PLATFORM
  portENTER_CRITICAL(&budgetMux);
  deadlineUs = 0;
  fetchDeadlineUs = 0;
  portEXIT_CRITICAL(&budgetMux);
#else
  deadlineUs.store(0, std::memory_order_release);
  fetchDeadlineUs.store(0, std::memory_order_release);
#endif
}

void beginFetchWindow() {
  const int64_t next = esp_timer_get_time() + 500000;
#ifdef ESP_PLATFORM
  portENTER_CRITICAL(&budgetMux);
  fetchDeadlineUs = next;
  portEXIT_CRITICAL(&budgetMux);
#else
  fetchDeadlineUs.store(next, std::memory_order_release);
#endif
}
void endFetchWindow() {
#ifdef ESP_PLATFORM
  portENTER_CRITICAL(&budgetMux);
  fetchDeadlineUs = 0;
  portEXIT_CRITICAL(&budgetMux);
#else
  fetchDeadlineUs.store(0, std::memory_order_release);
#endif
}

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

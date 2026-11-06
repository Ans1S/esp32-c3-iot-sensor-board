#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "logical_clock.h"
#include "measurement_budget.h"
#include "precision_cadence.h"
#include "radio_send_guard.h"

int64_t testTimeUs = 0;
namespace sensor {
void lowPowerSensorWaitUs(uint32_t period) { testTimeUs += period; }
}
int main() {
  station::RadioSendGuard radio;
  assert(!radio.stalled(999999, 10000));
  radio.start(UINT32_MAX - 4999);
  assert(radio.pending() && !radio.stalled(4999, 10000));
  assert(radio.stalled(5000, 10000));
  radio.complete();
  assert(!radio.pending() && !radio.stalled(5001, 10000));
  radio.start(5001); radio.complete();
  assert(!radio.stalled(15001, 10000));
  lil::timing::RetryBackoff retry;
  assert(retry.ready(0));
  assert(retry.ready(0x80000001UL)); // First recording after 24.9 days idle.
  retry.wait(100, 1000);
  assert(!retry.ready(1099) && retry.ready(1100));
  assert(retry.ready(0x80001000UL)); // An expired retry never reactivates.
  retry.wait(UINT32_MAX - 499, 1000);
  assert(!retry.ready(499) && retry.ready(500));
  retry.wait(100, 1000);
  assert(retry.ready(0x80001000UL)); // A long outage is already past the retry.
  retry.wait(0, 1000); retry.clear();
  assert(retry.ready(1));
  // A restart scheduled exactly 2500 ms before millis wraps still fires.
  retry.wait(UINT32_MAX - 2499, 2500);
  assert(!retry.ready(UINT32_MAX));
  assert(retry.ready(0));
  testTimeUs = 12000;
  sensor::beginMeasurementBudget(100);
  sensor::measurementWaitUs(20000);
  assert(testTimeUs == 32000 && !sensor::measurementBudgetExpired());
  sensor::measurementWaitUs(200000);
  assert(testTimeUs == 112000 && wchargerSensorBudgetExpired());
  sensor::measurementWaitUs(1000);
  assert(testTimeUs == 112000);
  sensor::endMeasurementBudget();
  sensor::measurementWaitUs(1000);
  assert(testTimeUs == 113000);
  sensor::beginMeasurementBudget(6000);
  wchargerBeginFetchWindow();
  sensor::measurementWaitUs(1000000);
  assert(testTimeUs == 613000 && sensor::measurementBudgetExpired());
  wchargerEndFetchWindow();
  assert(!sensor::measurementBudgetExpired());
  sensor::endMeasurementBudget();
  testTimeUs = 113000;
  sensor::accountForDeepSleep(300000000);
  testTimeUs = 0; // Simulated RTC-retaining deep-sleep restart.
  assert(sensor::logicalTimeUs() == 300113000);
  testTimeUs = 42000;
  assert(sensor::logicalTimeMs() == 300155);
  sensor::accountForDeepSleep(1000000); // Early error sleep uses the same clock.
  testTimeUs = 0;
  assert(sensor::logicalTimeMs() == 301155);
  puts("Measurement budget, long-uptime retry backoff and deep-sleep clock tests passed");
}

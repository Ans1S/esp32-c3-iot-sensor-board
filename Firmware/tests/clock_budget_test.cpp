#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "logical_clock.h"
#include "measurement_budget.h"

int64_t testTimeUs = 0;
namespace sensor {
void lowPowerSensorWaitUs(uint32_t period) { testTimeUs += period; }
}
int main() {
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
  puts("Measurement budget and deep-sleep clock tests passed");
}

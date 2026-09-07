#pragma once

#include <stdint.h>

namespace sensor {
void beginMeasurementBudget(uint32_t milliseconds = 6000);
void endMeasurementBudget();
bool measurementBudgetExpired();
void measurementWaitUs(uint32_t microseconds);
}  // namespace sensor

extern "C" bool wchargerSensorBudgetExpired();
extern "C" void wchargerBeginFetchWindow();
extern "C" void wchargerEndFetchWindow();

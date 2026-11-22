#pragma once
#include <stdint.h>
#include "sensor_timing.h"

namespace sensor {
uint64_t logicalTimeUs();
inline uint64_t logicalTimeMs() { return lil::timing::usToMs(logicalTimeUs()); }
void accountForDeepSleep(uint64_t sleepMicroseconds);
}  // namespace sensor

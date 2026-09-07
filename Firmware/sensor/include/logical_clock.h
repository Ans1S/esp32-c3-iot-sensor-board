#pragma once
#include <stdint.h>

namespace sensor {
uint64_t logicalTimeUs();
inline uint64_t logicalTimeMs() { return logicalTimeUs() / 1000ULL; }
void accountForDeepSleep(uint64_t sleepMicroseconds);
}  // namespace sensor

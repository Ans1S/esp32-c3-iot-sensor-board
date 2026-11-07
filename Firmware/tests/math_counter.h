#pragma once
#include <cmath>
#include <cstddef>

// Instrument actual driver sqrtf calls without changing its arithmetic.
inline size_t testSqrtCalls = 0;
inline float testCountedSqrt(float value) {
  ++testSqrtCalls;
  return std::sqrt(value);
}
#define sqrtf(value) testCountedSqrt(value)

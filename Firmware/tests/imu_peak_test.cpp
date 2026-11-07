#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif
#include <Arduino.h>
#include <Wire.h>
#include <cassert>
#include <cstring>
#include "lsm6dsox_driver.h"

#ifndef EXPECT_DEFERRED_GYRO_SQRT
#define EXPECT_DEFERRED_GYRO_SQRT 1
#endif
void testRadioTick() { ++testMillis; }
int main() {
  sensor::Lsm6dsoxDriver driver;
  Wire.registers[0x0F] = 0x6C;
  assert(driver.begin(0x6A));
  uint32_t random = 0x61F3A9U;
  size_t compared = 0;
  // Each window is a bounded snapshot below the 512-entry FIFO limit. Include
  // empty/single windows, signed endpoints and peaks followed by quiet tails.
  for (size_t window = 0; window < 1024; ++window) {
    const size_t count = window % 257;
    float expected = 0;
    for (size_t i = 0; i < count; ++i) {
      int16_t axes[3];
      float squared = 0;
      for (size_t axis = 0; axis < 3; ++axis) {
        random = random * 1664525U + 1013904223U;
        axes[axis] = window < 4 ? (i ? 0 : INT16_MIN) : static_cast<int16_t>(random >> 16);
        const float scaled = axes[axis] * 0.0175F;
        squared += scaled * scaled;
      }
      Wire.sample(1, axes[0], axes[1], axes[2]);
      expected = fmaxf(expected, std::sqrt(squared));
    }
    testSqrtCalls = 0;
    const auto result = driver.read();
    assert(!memcmp(&expected, &result.motion.peakAngularRateDps, sizeof(expected)));
    assert(testSqrtCalls == (EXPECT_DEFERRED_GYRO_SQRT ? size_t(count != 0) : count));
    compared += count;
    if (count) {
      assert(driver.read().motion.peakAngularRateDps == 0);
      // Polls may accumulate a window before read(); a FIFO fault must discard
      // its peak so it cannot contaminate the next complete window.
      Wire.sample(1, INT16_MIN, INT16_MIN, INT16_MIN);
      assert(driver.poll());
      Wire.overrun = true;
      assert(!driver.read().valid);
      Wire.sample(1, 1, 0, 0);
      assert(driver.read().motion.peakAngularRateDps == 0.0175F);
    }
  }
  Wire.sample(2, 100, 0, 8000);
  for (size_t i = 0; i < 104; ++i) Wire.sample(1, int16_t(i), 0, 0);
  testSqrtCalls = 0;
  assert(driver.read().valid);
  const size_t roots = testSqrtCalls;
  assert(roots == (EXPECT_DEFERRED_GYRO_SQRT ? 2 : 105));
  for (size_t i = 0; i < 104; ++i) {
    Wire.sample(2, 100, 0, 8000);
    Wire.sample(1, int16_t(i), 0, 0);
  }
  testSqrtCalls = 0;
  assert(driver.read().valid);
  const size_t balancedRoots = testSqrtCalls;
  assert(balancedRoots == (EXPECT_DEFERRED_GYRO_SQRT ? 105 : 208));
  printf("IMU peak: %zu gyro vectors in 1024 windows match original sqrt/max bits; mixed 104-gyro/1-accel window uses %zu sqrt calls; fault/reset passed\n", compared, roots);
  printf("IMU balanced 104-gyro/104-accel window: %zu sqrt calls\n", balancedRoots);
}

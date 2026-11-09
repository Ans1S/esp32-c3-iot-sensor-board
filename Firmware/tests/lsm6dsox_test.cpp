#include <Arduino.h>
#include <Wire.h>
#include <cassert>
#include "lsm6dsox_driver.h"
#include "protocol_counter.h"
void testRadioTick() { ++testMillis; }
int main() {
  sensor::Lsm6dsoxDriver driver;
  assert(!driver.begin(0x6A)); // Wrong WHO_AM_I.
  Wire.registers[0x0F] = 0x6C;
  Wire.stuckReset = true;
  auto started = millis();
  assert(!driver.begin(0x6A) && millis() - started <= 101);
  Wire.stuckReset = false;
  assert(driver.begin(0x6A));
  assert(Wire.registers[0x10] == 0x48 && Wire.registers[0x11] == 0x44);
  assert(Wire.registers[0x12] == 0x44 && Wire.registers[0x09] == 0x44 && Wire.registers[0x0A] == 6);
  assert(!driver.read().valid); // No fabricated zero measurement.
  Wire.sample(2, 10000, -10000, 0);
  Wire.sample(1, -10000, 0, 10000);
  Wire.sample(2, 1000, 0, 0); // Peak must survive a quieter latest sample.
  Wire.sample(1, 100, 0, 0);
  auto r = driver.read();
  assert(r.valid && r.motion.sampleCount == 2);
  assert(fabs(r.motion.accelerationG[0] - .671F) < .00001F);
  assert(fabs(r.motion.angularRateDps[0] - -86.625F) < .00001F);
  assert(r.motion.peakAccelerationG > 1.72F && r.motion.peakAngularRateDps > 247);
  assert(!driver.read().valid);
  Wire.sample(2, -10000, 0, 0); Wire.sample(1, -10000, 0, 0);
  Wire.overrun = true;
  r = driver.read();
  assert(!r.valid && r.motion.fifoOverrun && (r.live.flags & lil::protocol::kLiveGap));
  assert(Wire.fifo.empty() && r.motion.sampleCount == 0);
  Wire.sample(2, 10000, 0, 0); Wire.sample(1, 10000, 0, 0);
  r = driver.read();
  assert(r.valid && fabs(r.motion.accelerationG[0] - 1.22F) < .0001F && r.motion.angularRateDps[0] == 175);
  Wire.sample(2, 10000, 0, 0); Wire.sample(1, 10000, 0, 0);
  assert(driver.poll());
  Wire.latchedOverrun = true;
  r = driver.read(); assert(!r.valid && r.motion.sampleCount == 0 && r.motion.fifoOverrun);
  Wire.shortRead = true;
  assert(!driver.read().valid);
  Wire.shortRead = false; Wire.overrun = false;
  assert(driver.begin(0x6A)); // A partial I2C transfer requires reinitialization.
  Wire.sample(2, 0, 0, 8197); Wire.sample(1, 0, 0, 0);
  assert(driver.poll());
  testMillis += 201;
  assert(!driver.read().valid); // Stale axes are not reported as fresh.
  assert(driver.begin(0x6A));
  Wire.sample(2, 8197, 0, 0); Wire.sample(1, 0, 0, 0);
  assert(driver.poll());
  testMillis += 500;
  // Samples queued while acquisition was suspended are not freshly acquired
  // just because I2C drains them now. Require a complete new window.
  Wire.sample(2, 0, 8197, 0); Wire.sample(1, 0, 0, 0);
  r = driver.read();
  assert(!r.valid && !r.motion.sampleCount && !r.motion.fifoOverrun &&
      (r.live.flags & lil::protocol::kLiveGap) && Wire.fifo.empty());
  testMillis += 10;
  Wire.sample(2, 0, 0, 8197); Wire.sample(1, 0, 0, 0);
  assert(driver.read().valid);
  // A corrupt FIFO length must not cause hundreds of unbounded bus reads.
  for (unsigned i = 0; i < 513; ++i) Wire.sample(2, 0, 0, 8197);
  r = driver.read();
  assert(!r.valid && r.motion.fifoOverrun && Wire.fifo.empty());
  assert(driver.begin(0x6A));
  Wire.sample(2, 10000, 0, 0); Wire.sample(1, 10000, 0, 0);
  assert(driver.poll()); Wire.readRequests = 0;
#if FLOW_OPTIMIZED
  r = driver.read(false);
#else
  r = driver.read();
#endif
  assert(r.valid && fabs(r.motion.accelerationG[0] - 1.22F) < .0001F && r.motion.angularRateDps[0] == 175);
  assert(Wire.readRequests == (FLOW_OPTIMIZED ? 0 : 1));
  printf("IMU checked snapshot: %u additional I2C read requests after poll\n", Wire.readRequests);
  Wire.fifoRequests.clear();
  Wire.readRequests = 0;
  for (int i = 0; i < 9; ++i) {
    Wire.sample(2, 10000 + 100*i, -1000*i, 0);
    Wire.sample(1, -1000*i, 10000 + 100*i, 0);
  }
  r = driver.read();
  printf("IMU snapshot of 18 FIFO words: %u read requests, %zu FIFO transactions\n",
         Wire.readRequests, Wire.fifoRequests.size());
  assert(Wire.readRequests == 4 && (Wire.fifoRequests == std::vector<uint8_t>{56, 56, 14}));
  assert(r.valid && r.motion.sampleCount == 9 && Wire.fifo.empty());
  assert(fabs(r.motion.accelerationG[0] - 10400 * .000122F) < .00001F);
  assert(fabs(r.motion.accelerationG[1] - -4000 * .000122F) < .00001F);
  assert(fabs(r.motion.angularRateDps[0] - -4000 * .0175F) < .00001F);
  assert(fabs(r.motion.angularRateDps[1] - 10400 * .0175F) < .0001F);
  Wire.fifoRequests.clear();
  Wire.shortFifoRequest = 2;
  for (int i = 0; i < 9; ++i) {
    Wire.sample(2, 8197, 0, 0);
    Wire.sample(1, 100, 0, 0);
  }
  assert(!driver.poll());
  r = driver.read(false);
  assert(!r.valid && r.motion.sampleCount == 0 && (r.live.flags & lil::protocol::kLiveGap));
  assert(!driver.poll());
  Wire.shortFifoRequest = 0;
  assert(driver.begin(0x6A));
#if FLOW_OPTIMIZED
  Wire.sample(2, 10000, 0, 0); Wire.sample(1, 10000, 0, 0);
  assert(driver.poll()); testMillis += 201; Wire.readRequests = 0;
  assert(!driver.read(false).valid && Wire.readRequests == 0);
#endif
  assert(driver.begin(0x6A));
  uint32_t seed = 17;
  float maxAccelerationDelta = 0, maxGyroDelta = 0;
  for (unsigned window = 1; window <= 256; ++window) {
    float previous[2][3]{};
    int64_t sums[2][3]{};
    const float scales[] = {.000122F, .0175F};
    for (unsigned i = 0; i < window; ++i) {
      for (unsigned sensor = 0; sensor < 2; ++sensor) {
        int16_t axes[3];
        for (unsigned axis = 0; axis < 3; ++axis) {
          seed = seed * 1664525U + 1013904223U;
          axes[axis] = int32_t(seed >> 16) - 32768;
          sums[sensor][axis] += axes[axis];
          previous[sensor][axis] += axes[axis] * scales[sensor];
        }
        Wire.sample(sensor == 0 ? 2 : 1, axes[0], axes[1], axes[2]);
      }
    }
    const auto average = driver.read();
    assert(average.valid && average.motion.sampleCount == window);
    for (unsigned axis = 0; axis < 3; ++axis) {
      const float acceleration = average.motion.accelerationG[axis];
      const float gyro = average.motion.angularRateDps[axis];
      const double exactAcceleration = double(sums[0][axis]) * scales[0] / window;
      const double exactGyro = double(sums[1][axis]) * scales[1] / window;
      assert(fabs(acceleration - exactAcceleration) <= .000001);
      assert(fabs(gyro - exactGyro) <= .0001);
      maxAccelerationDelta = fmaxf(maxAccelerationDelta, fabsf(acceleration - previous[0][axis] / window));
      maxGyroDelta = fmaxf(maxGyroDelta, fabsf(gyro - previous[1][axis] / window));
    }
  }
  assert(maxAccelerationDelta <= .00001F && maxGyroDelta <= .001F);
  printf("IMU 256 signed windows: maximum mean delta %.9g g and %.9g dps\n",
         double(maxAccelerationDelta), double(maxGyroDelta));
  // A caller that drains the FIFO but delays readout may exceed a signed
  // 32-bit total. Keep all six axes safe at either signed ADC extreme.
  for (unsigned batch = 0; batch < 257; ++batch) {
    for (unsigned i = 0; i < 256; ++i) {
      Wire.sample(2, -32768, 32767, 0);
      Wire.sample(1, 32767, -32768, 0);
    }
    assert(driver.poll());
  }
  r = driver.read(false);
  assert(r.valid && r.motion.sampleCount == UINT16_MAX);
  assert(r.motion.accelerationG[0] == -32768 * .000122F && r.motion.accelerationG[1] == 32767 * .000122F);
  assert(r.motion.angularRateDps[0] == 32767 * .0175F && r.motion.angularRateDps[1] == -32768 * .0175F);
  puts("LSM6DSOX identity, reset timeout, FIFO, signed scale, peaks, overflow and stale/error checks passed");
}

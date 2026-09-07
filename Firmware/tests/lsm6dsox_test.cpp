#include <Arduino.h>
#include <Wire.h>
#include <cassert>
#include "lsm6dsox_driver.h"
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
  assert(fabs(r.motion.accelerationG[0] - .122F) < .00001F);
  assert(fabs(r.motion.angularRateDps[0] - 1.75F) < .00001F);
  assert(r.motion.peakAccelerationG > 1.72F && r.motion.peakAngularRateDps > 247);
  assert(!driver.read().valid);
  Wire.sample(2, -10000, 0, 0); Wire.sample(1, -10000, 0, 0);
  Wire.overrun = true;
  r = driver.read();
  assert(r.valid && r.motion.fifoOverrun && r.motion.peakAccelerationG < 1.23F);
  assert(r.motion.accelerationG[0] < -1.21F && r.motion.angularRateDps[0] == -175);
  Wire.shortRead = true;
  assert(!driver.read().valid);
  Wire.shortRead = false; Wire.overrun = false;
  Wire.sample(2, 0, 0, 8197); Wire.sample(1, 0, 0, 0);
  assert(driver.poll());
  testMillis += 201;
  assert(!driver.read().valid); // Stale axes are not reported as fresh.
  puts("LSM6DSOX identity, reset timeout, FIFO, signed scale, peaks, overflow and stale/error checks passed");
}

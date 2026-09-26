#include <cassert>
#include <Arduino.h>
#include <Wire.h>
#include "bme280_driver.h"
void testRadioTick() { ++testMillis; }
bool expired = false;
namespace sensor {
bool measurementBudgetExpired() { return expired; }
void measurementWaitUs(uint32_t us) { testMillis += (us + 999) / 1000; }
}
int main() {
  sensor::Bme280Driver driver;
  assert(!driver.read().valid);
  assert(driver.begin(0x76));
  assert(Wire.forcedWrites == 0); // Initialization must stay asleep.
  auto r = driver.read();
  assert(r.valid && Wire.forcedWrites == 1 && Wire.dataBursts == 1);
  // Bosch reference T/P trimming and ADC example, double-precision results.
  assert(fabs(r.temperatureC - 25.0824779) < 0.0001);
  assert(fabs(r.pressureHpa - 1006.5326678) < 0.001);
  assert(fabs(r.humidityPercent - 78.4552267) < 0.0001);
  Wire = BmeWire{}; Wire.humidity(-30, -50); Wire.raw(519888,415148,1000);
  assert(driver.begin(0x76));
  assert(fabs(driver.read().humidityPercent - 17.3483688) < 0.0001);
  for (int reg : {0xD0,0xF3,0x88,0xE1,0xF2,0xF4,0xF5}) {
    Wire = BmeWire{}; Wire.failRead = reg; assert(!driver.begin(0x76));
  }
  for (int reg : {0xE0,0xF2,0xF4,0xF5}) {
    Wire = BmeWire{}; Wire.failWrite = reg; assert(!driver.begin(0x76));
  }
  Wire = BmeWire{}; Wire.ignoreWrite = 0xF2; assert(!driver.begin(0x76));
  Wire = BmeWire{}; Wire.inconsistentTrim = true; assert(!driver.begin(0x76));
  for (uint16_t trim : {0,65535}) {
    Wire = BmeWire{}; Wire.word(0x8E,trim); assert(!driver.begin(0x76));
  }
  Wire = BmeWire{}; Wire.stuck = true;
  auto start = millis(); assert(!driver.begin(0x76) && millis()-start <= 103);
  for (int reg : {0xF3,0xF4,0xF7}) {
    Wire = BmeWire{}; assert(driver.begin(0x76));
    Wire.failRead = reg; assert(!driver.read().valid);
  }
  Wire = BmeWire{}; assert(driver.begin(0x76));
  Wire.failWrite = 0xF4; assert(!driver.read().valid);
  Wire.failWrite = -1; Wire.stuck = true; start = millis();
  assert(!driver.read().valid && millis()-start <= 60);
  Wire = BmeWire{}; assert(driver.begin(0x76));
  Wire.raw(0x80000,415148,35000); assert(!driver.read().valid);
  expired = true; assert(!driver.begin(0x76)); expired = false;
  Wire = BmeWire{}; Wire.present = 0x77; assert(driver.begin(0x77));
  assert(driver.read().valid);
  puts("BME280: Bosch reference compensation, burst coherence, no init conversion, NVM/config checks, bus faults and deadlines passed");
}

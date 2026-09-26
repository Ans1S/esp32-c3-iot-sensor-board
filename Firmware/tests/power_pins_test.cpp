#include <Arduino.h>
#include "power_controller.h"
void testRadioTick() { ++testMillis; }
int main() {
  sensor::PowerController power;
  expectInactiveEnable = true; power.begin(); expectInactiveEnable = false;
  assert(padLatch[10] == 1 && padLatch[6] == 0);
  assert(padMode[4] == INPUT && padMode[5] == INPUT);
  power.sensorPower(true); power.adcPower(true);
  assert(padLatch[10] == 0 && padLatch[6] == 1);
  power.prepareForDeepSleep();
  assert(padLatch[10] == 1 && padLatch[6] == 0);
  assert(padMode[4] == INPUT && padMode[5] == INPUT);
  puts("V4 power gates: inactive latch before output enable, correct polarities and released I2C pads passed");
}

#include <Arduino.h>
#include "power_controller.h"
void testRadioTick() { ++testMillis; }
int main() {
  sensor::PowerController power;
  expectInactiveEnable = true; power.begin(); expectInactiveEnable = false;
#if PCB_VERSION == 4
  assert(padLatch[10] == 1 && padLatch[6] == 0);
#else
  assert(padLatch[10] == 0 && padMode[6] == 0);
#endif
  assert(padMode[4] == INPUT && padMode[5] == INPUT);
  power.sensorPower(true); power.adcPower(true);
#if PCB_VERSION == 4
  assert(padLatch[10] == 0 && padLatch[6] == 1);
#else
  assert(padLatch[10] == 1 && padMode[6] == 0);
#endif
  power.prepareForDeepSleep();
#if PCB_VERSION == 4
  assert(padLatch[10] == 1 && padLatch[6] == 0);
#else
  assert(padLatch[10] == 0 && padMode[6] == 0);
#endif
  assert(padMode[4] == INPUT && padMode[5] == INPUT);
#if PCB_VERSION == 4
  puts("V4 power gates: inactive latch before output enable, correct polarities and released I2C pads passed");
#else
  puts("V3 power gates: inactive latch before output enable, active-high switching, no ADC gate and released I2C pads passed");
#endif
}

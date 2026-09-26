#pragma once
#include "../stubs/Arduino.h"
#include <cassert>
using gpio_num_t = int;
constexpr int GPIO_NUM_NC = -1, GPIO_NUM_3 = 3, GPIO_NUM_4 = 4, GPIO_NUM_5 = 5,
    GPIO_NUM_6 = 6, GPIO_NUM_10 = 10;
constexpr int HIGH = 1, LOW = 0, INPUT = 1, OUTPUT = 2;
inline int padMode[22]{}, padLatch[22]{};
inline bool expectInactiveEnable = false;
inline void pinMode(uint8_t pin, int mode) {
  if (expectInactiveEnable && mode == OUTPUT) {
    if (pin == 10) assert(padLatch[pin] == 1); // Active-low V4 PMOS.
    if (pin == 6) assert(padLatch[pin] == 0);
  }
  padMode[pin] = mode;
}
inline void digitalWrite(uint8_t pin, int level) {
  // Arduino 3.x does not write the output latch before GPIO registration.
  assert(padMode[pin] != 0); padLatch[pin] = level;
}

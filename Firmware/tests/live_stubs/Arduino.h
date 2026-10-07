#pragma once
#include "../stubs/Arduino.h"
constexpr int INPUT_PULLUP = 1, LOW = 0, HIGH = 1;
inline bool testButtonLow = false;
inline void pinMode(int, int) {}
inline int digitalRead(int) { return testButtonLow ? LOW : HIGH; }

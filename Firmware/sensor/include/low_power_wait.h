#pragma once

#include <stdint.h>

namespace sensor {

// Keeps the environmental-sensor and optional ADC-enable GPIOs latched while
// the CPU sleeps.
// Any setup or wake-up overhead is included in periodUs; if Light-sleep is not
// available, the remaining time is completed with a normal cooperative delay.
void lowPowerSensorWaitUs(uint32_t periodUs);

}  // namespace sensor

extern "C" void wchargerBsecLowPowerWait(uint32_t periodUs);

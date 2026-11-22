#pragma once

#include <stdint.h>

namespace sensor {

// Keeps the environmental-sensor and optional ADC-enable GPIOs latched while
// the CPU sleeps.
// Any setup or wake-up overhead is included in periodUs; if Light-sleep is not
// available, the remaining time is completed with a normal cooperative delay.
void lowPowerSensorWaitUs(uint32_t periodUs);

#ifdef ESP_PLATFORM
// Reference-counted inhibit used while Wi-Fi/ESP-NOW or multi-task live
// acquisition is active so sensor waits never invoke esp_light_sleep_start()
// concurrently with active radio or another FreeRTOS task.
void pushLowPowerWaitInhibit();
void popLowPowerWaitInhibit();
bool lowPowerWaitAllowed();
#else
inline void pushLowPowerWaitInhibit() {}
inline void popLowPowerWaitInhibit() {}
inline bool lowPowerWaitAllowed() { return true; }
#endif

}  // namespace sensor

extern "C" void wchargerBsecLowPowerWait(uint32_t periodUs);

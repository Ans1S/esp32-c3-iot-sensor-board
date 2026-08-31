#pragma once

#include <Arduino.h>

#include "power_controller.h"

namespace sensor {

class SleepController {
 public:
  [[noreturn]] static void deepSleep(uint32_t seconds,
                                     PowerController& power,
                                     uint32_t additionalMilliseconds = 0);
};

}  // namespace sensor

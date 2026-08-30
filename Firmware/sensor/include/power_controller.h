#pragma once

#include "hardware_profile.h"

namespace sensor {

class PowerController {
 public:
  void begin();
  void sensorPower(bool enabled);
  void adcPower(bool enabled);
  void prepareForDeepSleep();

 private:
  static void writeActive(gpio_num_t pin, bool activeHigh, bool enabled);
};

}  // namespace sensor

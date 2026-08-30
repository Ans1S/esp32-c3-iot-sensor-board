#pragma once

#include <Arduino.h>

#include "power_controller.h"

namespace sensor {

struct BatteryReading {
  bool valid = false;
  uint16_t millivolts = 0;
};

class AdcReader {
 public:
  void begin(PowerController& power);
  BatteryReading readBattery(float calibrationFactor = 1.0F);

 private:
  PowerController* power_ = nullptr;
};

}  // namespace sensor

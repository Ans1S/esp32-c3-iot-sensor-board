#pragma once

#include <Adafruit_BME280.h>

#include "environmental_reading.h"

namespace sensor {

class Bme280Driver {
 public:
  bool begin();
  EnvironmentalReading read();

 private:
  Adafruit_BME280 sensor_;
  bool initialized_ = false;
};

}  // namespace sensor

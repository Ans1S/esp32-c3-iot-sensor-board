#pragma once

#include "environmental_reading.h"

namespace sensor {

class Bme280Driver {
 public:
  bool begin(uint8_t address);
  EnvironmentalReading read();

 private:
  bool readBytes(uint8_t reg, uint8_t* bytes, uint8_t count);
  bool write8(uint8_t reg, uint8_t value);
  bool waitReady(uint32_t timeoutMs);
  uint8_t address_ = 0;
  uint16_t t1_ = 0, p1_ = 0;
  int16_t t2_ = 0, t3_ = 0, p_[8]{};
  uint8_t h1_ = 0, h3_ = 0;
  int16_t h2_ = 0, h4_ = 0, h5_ = 0;
  int8_t h6_ = 0;
  bool initialized_ = false;
};

}  // namespace sensor

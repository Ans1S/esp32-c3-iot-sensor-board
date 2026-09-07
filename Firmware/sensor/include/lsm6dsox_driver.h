#pragma once
#include "environmental_reading.h"

namespace sensor {
// ST DS12814 Rev 4: 104 Hz, +/-4 g, +/-500 dps, uncompressed FIFO.
class Lsm6dsoxDriver {
 public:
  bool begin(uint8_t address);
  bool poll();
  EnvironmentalReading read();
 private:
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegisters(uint8_t reg, uint8_t* data, uint8_t size);
  uint8_t address_ = 0;
  bool initialized_ = false;
  bool failed_ = false;
  bool overflow_ = false;
  uint32_t lastAccelMs_ = 0, lastGyroMs_ = 0;
  uint32_t accelCount_ = 0, gyroCount_ = 0;
  lil::protocol::MotionReading motion_{};
};
}  // namespace sensor

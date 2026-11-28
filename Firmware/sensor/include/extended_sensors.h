#pragma once

#include <stdint.h>
#include "environmental_reading.h"

namespace sensor {

// Compact, integer-only I2C driver for AS7341 (10-channel spectral),
// TOF400C-VL53L1X (4 m Time-of-Flight) and CCS811 (eCO2/TVOC gas sensor).
// All arithmetic is pure integer to avoid software floating-point calls on
// FPU-less RV32IMC.
class ExtendedSensors {
 public:
  static bool probe(lil::protocol::EnvironmentalSensorType type, uint8_t& address);
  static void clearCcs811Baseline();
  bool resetCcs811();
  bool begin(lil::protocol::EnvironmentalSensorType type, uint8_t address);
  EnvironmentalReading read();
  bool setCcs811Interval(uint32_t reportSeconds);
  EnvironmentalReading readContinuous();

 private:
  static bool readReg8(uint8_t address, uint8_t reg, uint8_t* data, uint8_t count);
  static bool writeReg8(uint8_t address, uint8_t reg, uint8_t value);
  static bool writeReg8Burst(uint8_t address, uint8_t reg, const uint8_t* data, uint8_t count);
  static bool readReg16(uint8_t address, uint16_t reg, uint8_t* data, uint8_t count);
  static bool writeReg16(uint8_t address, uint16_t reg, uint8_t value);
  static bool writeReg16Burst(uint8_t address, uint16_t reg, const uint8_t* data, uint8_t count);

  bool beginAs7341();
  bool as7341LoadSmux(const uint8_t table[20]);
  bool as7341MeasurePass(const uint8_t table[20], uint16_t out[6], bool& saturated);
  EnvironmentalReading readAs7341();

  bool beginVl53l1x();
  bool vl53l1xWaitDataReady(uint32_t timeoutMs);
  EnvironmentalReading readVl53l1x();

  bool beginCcs811();
  static bool ccs811ReadReg(uint8_t address, uint8_t reg, uint8_t* data, uint8_t count);
  static bool ccs811SoftReset(uint8_t address);
  EnvironmentalReading readCcs811();
  uint8_t ccsDriveMode_ = 0;
  bool ccsCooling_ = false;
  bool ccsConditioned_ = false;
  uint32_t ccsModeStartedMs_ = 0;
  uint32_t ccsLastReadyMs_ = 0;
  uint32_t ccsLastSaveMs_ = 0;

  lil::protocol::EnvironmentalSensorType type_ =
      lil::protocol::EnvironmentalSensorType::kAutoDetect;
  uint8_t address_ = 0;
  uint8_t as7341Gain_ = 4;  // 8x default (AGAIN=4)
  bool initialized_ = false;
};

}  // namespace sensor

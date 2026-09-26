#pragma once
#include "environmental_reading.h"

namespace sensor {
// Register-level drivers; no EEPROM writes or assumed body-temperature offset.
class PrecisionSensors {
 public:
  static bool probe(lil::protocol::EnvironmentalSensorType type, uint8_t& address);
  bool begin(lil::protocol::EnvironmentalSensorType type, uint8_t address);
  bool poll();
  EnvironmentalReading read();
  bool freshTemperature() const { return freshTemperature_; }
 private:
  bool readBytes(uint8_t reg, uint8_t* bytes, uint8_t count);
  bool write8(uint8_t reg, uint8_t value);
  bool write16(uint8_t reg, uint16_t value);
  bool clearOpticalFifo();
  bool invalidateOpticalCapture();
  void resetSignal();
  lil::protocol::EnvironmentalSensorType type_ = lil::protocol::EnvironmentalSensorType::kAutoDetect;
  uint8_t address_ = 0, current_ = 0x24;
  bool initialized_ = false, failed_ = false, overflow_ = false;
  uint32_t lastSampleMs_ = 0, lastAdjustMs_ = 0;
  uint32_t red_ = 0, infrared_ = 0;
  float temperature_ = 0;
  bool hasTemperature_ = false, freshTemperature_ = false, hasOptical_ = false;
  uint32_t lastPollMs_ = 0, lastEstimateMs_ = 0, estimateSampleMs_ = 0;
  lil::protocol::PulseReading estimate_{};
  bool hasEstimate_ = false, hasCalculated_ = false, waveGap_ = false;
  lil::protocol::OpticalFrame wave_[lil::protocol::kOpticalBatchSize]{};
  uint32_t waveTimes_[lil::protocol::kOpticalBatchSize]{};
  uint8_t waveCount_ = 0;
  uint32_t signal_[200]{};
  uint16_t head_ = 0, count_ = 0;
};
}

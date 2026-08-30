#pragma once

#include <stdint.h>

#include "lil_protocol.h"

namespace sensor {

struct EnvironmentalReading {
  bool valid = false;
  bool bme680RawFallback = false;
  float temperatureC = 0;
  float humidityPercent = 0;
  float pressureHpa = 0;
  float iaq = 0;
  float gasResistanceOhms = 0;
  uint16_t capabilities = 0;
  lil::protocol::EnvironmentalSensorType sensorType =
      lil::protocol::EnvironmentalSensorType::kAutoDetect;
  uint8_t iaqAccuracy = 0;
  lil::protocol::IaqCalibrationPhase iaqCalibrationPhase =
      lil::protocol::IaqCalibrationPhase::kNotApplicable;
  uint16_t iaqCalibrationElapsedMinutes = 0;
  uint16_t iaqCalibrationRemainingMinutes = 0;
};

}  // namespace sensor

#pragma once

#include "bme280_driver.h"
#include "bme680_driver.h"
#include "environmental_reading.h"
#include "power_controller.h"

namespace sensor {

class EnvironmentalSensor {
 public:
  bool begin(PowerController& power,
             lil::protocol::EnvironmentalSensorType requestedType,
             float temperatureOffsetC);
  EnvironmentalReading read();
  void end();
  void prepareForDeepSleep(
      uint32_t seconds,
      lil::protocol::EnvironmentalSensorType activeType =
          lil::protocol::EnvironmentalSensorType::kAutoDetect);
  uint32_t bme680RecommendedSleepSeconds(uint32_t fallbackSeconds) const;
  void clearIaqState();
  lil::protocol::EnvironmentalSensorType detectedType() const;

 private:
  lil::protocol::EnvironmentalSensorType probeSensorType() const;
  bool startDetectedSensor(
      lil::protocol::EnvironmentalSensorType requestedType,
      float temperatureOffsetC);
  bool beginType(lil::protocol::EnvironmentalSensorType type,
                 float temperatureOffsetC);

  PowerController* power_ = nullptr;
  Bme280Driver bme280_;
  Bme680Driver bme680_;
  lil::protocol::EnvironmentalSensorType requestedType_ =
      lil::protocol::EnvironmentalSensorType::kAutoDetect;
  lil::protocol::EnvironmentalSensorType detectedType_ =
      lil::protocol::EnvironmentalSensorType::kAutoDetect;
  bool initialized_ = false;
};

}  // namespace sensor

#pragma once

#include <variant>
#include "lsm6dsox_driver.h"
#include "precision_sensors.h"
#include "bme280_driver.h"
#include "bme680_driver.h"
#include "extended_sensors.h"
#include "environmental_reading.h"
#include "power_controller.h"

namespace sensor {

class EnvironmentalSensor {
 public:
  bool begin(PowerController& power,
             lil::protocol::EnvironmentalSensorType requestedType,
             float temperatureOffsetC);
  // false is reserved for the acquisition owner immediately after pollLive().
  // Driver validity and sample-age checks still run for the snapshot.
  EnvironmentalReading read(bool pollFirst = true);
  bool setCcs811Interval(uint32_t reportSeconds);
  EnvironmentalReading readContinuous();
  EnvironmentalReading readForReport(float temperatureOffsetC);
  bool pollLive() {
    if (auto* imu = std::get_if<Lsm6dsoxDriver>(&driver_)) return imu->poll();
    if (auto* prec = std::get_if<PrecisionSensors>(&driver_)) return prec->poll();
    return false;
  }
  bool liveInitialized() const {
    if (const auto* imu = std::get_if<Lsm6dsoxDriver>(&driver_)) return imu->initialized();
    if (const auto* prec = std::get_if<PrecisionSensors>(&driver_)) return prec->initialized();
    return false;
  }
  bool freshTemperature() const {
    if (const auto* prec = std::get_if<PrecisionSensors>(&driver_)) return prec->freshTemperature();
    return false;
  }
  void resetMotionFeedback() {
    if (auto* imu = std::get_if<Lsm6dsoxDriver>(&driver_)) imu->resetFeedback();
  }
  void end();
  void prepareForDeepSleep(
      uint32_t seconds,
      lil::protocol::EnvironmentalSensorType activeType =
          lil::protocol::EnvironmentalSensorType::kAutoDetect);
  uint32_t bme680RecommendedSleepSeconds(uint32_t fallbackSeconds) const;
  bool clearIaqState(bool resetRunningCcs811 = false);
  lil::protocol::EnvironmentalSensorType detectedType() const;

 private:
  lil::protocol::EnvironmentalSensorType probeSensorType(uint8_t& address) const;
  bool restartSensor(lil::protocol::EnvironmentalSensorType type, float temperatureOffsetC);
  bool startBusAndSensor(lil::protocol::EnvironmentalSensorType type, float temperatureOffsetC);
  bool startDetectedSensor(
      lil::protocol::EnvironmentalSensorType requestedType,
      float temperatureOffsetC);
  bool beginType(lil::protocol::EnvironmentalSensorType type,
                 uint8_t address, float temperatureOffsetC);

  PowerController* power_ = nullptr;
  // Mutually exclusive sensor drivers share one static slot (~2 KB DRAM saved on C3).
  std::variant<std::monostate, Lsm6dsoxDriver, PrecisionSensors, Bme280Driver, Bme680Driver, ExtendedSensors> driver_;
  lil::protocol::EnvironmentalSensorType requestedType_ =
      lil::protocol::EnvironmentalSensorType::kAutoDetect;
  lil::protocol::EnvironmentalSensorType detectedType_ =
      lil::protocol::EnvironmentalSensorType::kAutoDetect;
  bool recoveryUsed_ = false;
  bool initialized_ = false;
  bool typeMismatch_ = false;
};

}  // namespace sensor

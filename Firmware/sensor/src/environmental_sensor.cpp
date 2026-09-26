#include "environmental_sensor.h"

#include <Wire.h>

#include "hardware_profile.h"
#include "lil_protocol.h"
#include "sensor_log.h"
#include "measurement_budget.h"

namespace sensor {

namespace {
constexpr uint32_t kSensorPowerUpMs = 12;
constexpr uint32_t kPowerCycleOffMs = 100;
constexpr uint32_t kPowerCycleRecoveryMs = 200;
constexpr uint8_t kChipIdRegister = 0xD0;
constexpr uint8_t kBme280ChipId = 0x60;
constexpr uint8_t kBme680ChipId = 0x61;
constexpr uint8_t kAddresses[] = {0x76, 0x77};
void releaseI2cPins() {
  // Wire.end() alone does not guarantee that internal pull-ups are disabled.
  pinMode(static_cast<uint8_t>(kHardware.sdaPin), INPUT);
  pinMode(static_cast<uint8_t>(kHardware.sclPin), INPUT);
}
}

bool EnvironmentalSensor::begin(
    PowerController& power,
    lil::protocol::EnvironmentalSensorType requestedType,
    float temperatureOffsetC) {
  power_ = &power;
  requestedType_ = requestedType;
  detectedType_ = lil::protocol::EnvironmentalSensorType::kAutoDetect;
  initialized_ = false;
  typeMismatch_ = false;
  beginMeasurementBudget();
  if (requestedType == lil::protocol::EnvironmentalSensorType::kDisabled) {
    detectedType_ = lil::protocol::EnvironmentalSensorType::kDisabled;
    initialized_ = true;
    return true;
  }
  power_->sensorPower(true);
  measurementWaitUs(kSensorPowerUpMs * 1000UL);
  // V4 has 5.1 kOhm pull-ups on the switched rail. At 400 kHz the 300 ns
  // rise-time limit leaves only about 69 pF of bus capacitance. Slow Bosch
  // measurements do not need that marginal fast-mode timing.
  const uint32_t busHz = lil::protocol::isLiveSensor(requestedType) ? 400000 : 100000;
  if (!Wire.begin(static_cast<int>(kHardware.sdaPin),
                  static_cast<int>(kHardware.sclPin), busHz)) return false;
  Wire.setTimeOut(20);

  initialized_ = startDetectedSensor(requestedType, temperatureOffsetC);
  if (!initialized_ && !typeMismatch_ && !measurementBudgetExpired()) {
    // The proven PCB-V3 Extra-Sensor firmware recovered slow-starting BME680
    // boards with a complete power cycle. Some breakout-board capacitors do
    // not reach a valid I2C level within the normal 12 ms fast path.
    Wire.end();
    releaseI2cPins();
    SENSOR_LOG_PRINTLN(
        "[I2C] Initial sensor start failed; cycling sensor power");
    power_->sensorPower(false);
    measurementWaitUs(kPowerCycleOffMs * 1000UL);
    power_->sensorPower(true);
    measurementWaitUs(kPowerCycleRecoveryMs * 1000UL);
    if (!Wire.begin(static_cast<int>(kHardware.sdaPin),
                    static_cast<int>(kHardware.sclPin), busHz)) return false;
    Wire.setTimeOut(20);
    initialized_ = startDetectedSensor(requestedType, temperatureOffsetC);
  }
  if (!initialized_) {
    SENSOR_LOG_PRINTLN(
        "[I2C] Environmental sensor still unreadable after power cycle");
  }
  return initialized_;
}

lil::protocol::EnvironmentalSensorType EnvironmentalSensor::probeSensorType(
    uint8_t& detectedAddress) const {
  detectedAddress = 0;
  if ((requestedType_ == lil::protocol::EnvironmentalSensorType::kTmp117 || requestedType_ == lil::protocol::EnvironmentalSensorType::kMax30102) && PrecisionSensors::probe(requestedType_, detectedAddress)) return requestedType_;
  // Prefer an explicitly selected IMU; otherwise preserve Bosch-first auto detection.
  if (requestedType_ == lil::protocol::EnvironmentalSensorType::kLsm6dsox) {
    for (uint8_t address : {uint8_t(0x6A), uint8_t(0x6B)}) {
      Wire.beginTransmission(address); Wire.write(0x0F);
      if (Wire.endTransmission(false) == 0 && Wire.requestFrom(address, uint8_t(1)) == 1 && Wire.read() == 0x6C) {
        detectedAddress = address; return lil::protocol::EnvironmentalSensorType::kLsm6dsox;
      }
    }
  }
  for (const uint8_t address : kAddresses) {
    if (measurementBudgetExpired()) return lil::protocol::EnvironmentalSensorType::kAutoDetect;
    Wire.beginTransmission(address);
    Wire.write(kChipIdRegister);
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom(address, static_cast<uint8_t>(1)) != 1) {
      continue;
    }
    const uint8_t chipId = Wire.read();
    if (chipId == kBme680ChipId) {
      detectedAddress = address;
      return lil::protocol::EnvironmentalSensorType::kBme680;
    }
    if (chipId == kBme280ChipId) {
      detectedAddress = address;
      return lil::protocol::EnvironmentalSensorType::kBme280;
    }
  }
  for (uint8_t address : {uint8_t(0x6A), uint8_t(0x6B)}) {
    Wire.beginTransmission(address); Wire.write(0x0F);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(address, uint8_t(1)) == 1 && Wire.read() == 0x6C) {
      detectedAddress = address; return lil::protocol::EnvironmentalSensorType::kLsm6dsox;
    }
  }
  for (auto type : {lil::protocol::EnvironmentalSensorType::kTmp117, lil::protocol::EnvironmentalSensorType::kMax30102}) {
    if (PrecisionSensors::probe(type, detectedAddress)) return type;
  }
  return lil::protocol::EnvironmentalSensorType::kAutoDetect;
}

bool EnvironmentalSensor::startDetectedSensor(
    lil::protocol::EnvironmentalSensorType requestedType,
    float temperatureOffsetC) {
  uint8_t address = 0;
  if (measurementBudgetExpired()) return false;
  const auto detected = probeSensorType(address);
  if (detected == lil::protocol::EnvironmentalSensorType::kAutoDetect) {
    return false;
  }
  if (requestedType != lil::protocol::EnvironmentalSensorType::kAutoDetect &&
      requestedType != detected) {
    detectedType_ = detected;
    typeMismatch_ = true;
    return false;
  }
  return beginType(detected, address, temperatureOffsetC);
}

bool EnvironmentalSensor::beginType(
    lil::protocol::EnvironmentalSensorType type, uint8_t address,
    float temperatureOffsetC) {
  bool started = false;
  if (type == lil::protocol::EnvironmentalSensorType::kTmp117 || type == lil::protocol::EnvironmentalSensorType::kMax30102) started = precision_.begin(type, address);
  if (type == lil::protocol::EnvironmentalSensorType::kLsm6dsox) {
    started = lsm6dsox_.begin(address);
  } else if (type == lil::protocol::EnvironmentalSensorType::kBme280) {
    started = bme280_.begin(address);
  } else if (type == lil::protocol::EnvironmentalSensorType::kBme680) {
    started = bme680_.begin(address, temperatureOffsetC);
  }
  if (started) {
    detectedType_ = type;
  }
  return started;
}

EnvironmentalReading EnvironmentalSensor::read() {
  if (initialized_ && detectedType_ == lil::protocol::EnvironmentalSensorType::kLsm6dsox) return lsm6dsox_.read();
  if (initialized_ && (detectedType_ == lil::protocol::EnvironmentalSensorType::kTmp117 || detectedType_ == lil::protocol::EnvironmentalSensorType::kMax30102)) return precision_.read();
  if (measurementBudgetExpired()) return EnvironmentalReading{};
  if (requestedType_ == lil::protocol::EnvironmentalSensorType::kDisabled) {
    EnvironmentalReading disabled{};
    disabled.valid = true;
    disabled.sensorType = lil::protocol::EnvironmentalSensorType::kDisabled;
    return disabled;
  }
  if (!initialized_) {
    EnvironmentalReading failed{};
    failed.sensorType = detectedType_;
    return failed;
  }
  return detectedType_ == lil::protocol::EnvironmentalSensorType::kBme680
             ? bme680_.read()
             : bme280_.read();
}

void EnvironmentalSensor::end() {
  Wire.end();
  releaseI2cPins();
  if (power_ != nullptr) {
    power_->sensorPower(false);
  }
  initialized_ = false;
  endMeasurementBudget();
}

void EnvironmentalSensor::prepareForDeepSleep(
    uint32_t seconds, lil::protocol::EnvironmentalSensorType activeType) {
  if (detectedType_ == lil::protocol::EnvironmentalSensorType::kBme680 ||
      activeType == lil::protocol::EnvironmentalSensorType::kBme680) {
    bme680_.prepareForDeepSleep(seconds);
  }
}

uint32_t EnvironmentalSensor::bme680RecommendedSleepSeconds(
    uint32_t fallbackSeconds) const {
  return bme680_.recommendedSleepSeconds(fallbackSeconds);
}

void EnvironmentalSensor::clearIaqState() {
  bme680_.clearPersistentState();
}

lil::protocol::EnvironmentalSensorType EnvironmentalSensor::detectedType()
    const {
  return detectedType_;
}

}  // namespace sensor

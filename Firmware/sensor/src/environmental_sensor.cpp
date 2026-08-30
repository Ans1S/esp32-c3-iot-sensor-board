#include "environmental_sensor.h"

#include <Wire.h>

#include "hardware_profile.h"
#include "lil_protocol.h"

namespace sensor {

namespace {
constexpr uint32_t kSensorPowerUpMs = 12;
constexpr uint32_t kPowerCycleOffMs = 100;
constexpr uint32_t kPowerCycleRecoveryMs = 200;
constexpr uint8_t kChipIdRegister = 0xD0;
constexpr uint8_t kBme280ChipId = 0x60;
constexpr uint8_t kBme680ChipId = 0x61;
constexpr uint8_t kAddresses[] = {0x76, 0x77};
}

bool EnvironmentalSensor::begin(
    PowerController& power,
    lil::protocol::EnvironmentalSensorType requestedType,
    float temperatureOffsetC) {
  power_ = &power;
  requestedType_ = requestedType;
  detectedType_ = lil::protocol::EnvironmentalSensorType::kAutoDetect;
  initialized_ = false;
  if (requestedType == lil::protocol::EnvironmentalSensorType::kDisabled) {
    detectedType_ = lil::protocol::EnvironmentalSensorType::kDisabled;
    initialized_ = true;
    return true;
  }
  power_->sensorPower(true);
  delay(kSensorPowerUpMs);
  Wire.begin(static_cast<int>(kHardware.sdaPin),
             static_cast<int>(kHardware.sclPin), 400000);

  initialized_ = startDetectedSensor(requestedType, temperatureOffsetC);
  if (!initialized_) {
    // The proven PCB-V3 Extra-Sensor firmware recovered slow-starting BME680
    // boards with a complete power cycle. Some breakout-board capacitors do
    // not reach a valid I2C level within the normal 12 ms fast path.
    Wire.end();
    Serial.println("[I2C] Initial sensor start failed; cycling sensor power");
    power_->sensorPower(false);
    delay(kPowerCycleOffMs);
    power_->sensorPower(true);
    delay(kPowerCycleRecoveryMs);
    Wire.begin(static_cast<int>(kHardware.sdaPin),
               static_cast<int>(kHardware.sclPin), 400000);
    initialized_ = startDetectedSensor(requestedType, temperatureOffsetC);
  }
  if (!initialized_) {
    Serial.println("[I2C] Environmental sensor still unreadable after power cycle");
  }
  return initialized_;
}

lil::protocol::EnvironmentalSensorType EnvironmentalSensor::probeSensorType()
    const {
  for (const uint8_t address : kAddresses) {
    Wire.beginTransmission(address);
    Wire.write(kChipIdRegister);
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom(address, static_cast<uint8_t>(1)) != 1) {
      continue;
    }
    const uint8_t chipId = Wire.read();
    if (chipId == kBme680ChipId) {
      return lil::protocol::EnvironmentalSensorType::kBme680;
    }
    if (chipId == kBme280ChipId) {
      return lil::protocol::EnvironmentalSensorType::kBme280;
    }
  }
  return lil::protocol::EnvironmentalSensorType::kAutoDetect;
}

bool EnvironmentalSensor::startDetectedSensor(
    lil::protocol::EnvironmentalSensorType requestedType,
    float temperatureOffsetC) {
  const auto detected = probeSensorType();
  if (detected == lil::protocol::EnvironmentalSensorType::kAutoDetect) {
    return false;
  }
  if (requestedType != lil::protocol::EnvironmentalSensorType::kAutoDetect &&
      requestedType != detected) {
    detectedType_ = detected;
    return false;
  }
  return beginType(detected, temperatureOffsetC);
}

bool EnvironmentalSensor::beginType(
    lil::protocol::EnvironmentalSensorType type, float temperatureOffsetC) {
  bool started = false;
  if (type == lil::protocol::EnvironmentalSensorType::kBme280) {
    started = bme280_.begin();
  } else if (type == lil::protocol::EnvironmentalSensorType::kBme680) {
    started = bme680_.begin(temperatureOffsetC);
  }
  if (started) {
    detectedType_ = type;
  }
  return started;
}

EnvironmentalReading EnvironmentalSensor::read() {
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
  if (power_ != nullptr) {
    power_->sensorPower(false);
  }
  initialized_ = false;
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

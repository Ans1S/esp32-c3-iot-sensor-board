#include "environmental_sensor.h"

#include <Wire.h>
#include <memory>
#include <new>

#include "hardware_profile.h"
#include "lil_protocol.h"
#include <sensor_log.h>
#include "measurement_budget.h"
#include "low_power_wait.h"

namespace sensor {

namespace {
constexpr uint32_t kSensorPowerUpMs = 12;
// CCS811 datasheet DS000459 specifies t_START <= 20 ms after power-on reset;
// allow 50 ms for breakout RC reset circuits before the first I2C access.
constexpr uint32_t kCcs811PowerUpMs = 50;
constexpr uint32_t kPowerCycleOffMs = 100;
constexpr uint32_t kPowerCycleRecoveryMs = 200;
constexpr uint8_t kChipIdRegister = 0xD0;
constexpr uint8_t kBme68xVariantRegister = 0xF0;
constexpr uint8_t kBmp280ChipId = 0x58;
constexpr uint8_t kBme280ChipId = 0x60;
constexpr uint8_t kBme680ChipId = 0x61;
constexpr uint8_t kBme688VariantId = 0x01;
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
  const bool restartingActiveSensor = initialized_;
  if (power_ != nullptr) end();
  power_ = &power;
  requestedType_ = requestedType;
  detectedType_ = lil::protocol::EnvironmentalSensorType::kAutoDetect;
  initialized_ = false;
  typeMismatch_ = false;
  recoveryUsed_ = false;
  beginMeasurementBudget();
  // Release the previous bus/driver before a live type change. Normal BME
  // wakes already start with their rail off and keep the 12 ms fast path.
  if (restartingActiveSensor) measurementWaitUs(kPowerCycleOffMs * 1000UL);
  if (requestedType == lil::protocol::EnvironmentalSensorType::kDisabled) {
    detectedType_ = lil::protocol::EnvironmentalSensorType::kDisabled;
    initialized_ = true;
    return true;
  }
  power_->sensorPower(true);
  const uint32_t powerUpMs =
      requestedType == lil::protocol::EnvironmentalSensorType::kCcs811
          ? kCcs811PowerUpMs
          : kSensorPowerUpMs;
  measurementWaitUs(powerUpMs * 1000UL);
  SENSOR_LOG_PRINTLN("Attempt 1: initial rail power-up and probe");
  initialized_ = startBusAndSensor(requestedType, temperatureOffsetC);
  if (!initialized_ && !measurementBudgetExpired()) {
    pushLowPowerWaitInhibit();
    initialized_ = restartSensor(requestedType, temperatureOffsetC);
    popLowPowerWaitInhibit();
  }
  if (!initialized_) {
    SENSOR_LOG_PRINTLN(
        "[I2C] Selected sensor unavailable after power cycle; check type, address and switched supply");
    end();
  }
  return initialized_;
}

bool EnvironmentalSensor::startBusAndSensor(
    lil::protocol::EnvironmentalSensorType requestedType, float temperatureOffsetC) {
  // V4 has 5.1 kOhm pull-ups on the switched rail. At 400 kHz the 300 ns
  // rise-time limit leaves only about 69 pF of bus capacitance. Slow Bosch
  // measurements do not need that marginal fast-mode timing.
  // The MAX30102's 25 Hz, six-byte FIFO frames fit comfortably at 100 kHz.
  // Keep its bus in standard mode for external modules and the V4 pull-ups.
  const bool isCcs811 =
      requestedType == lil::protocol::EnvironmentalSensorType::kCcs811;
  const uint32_t busHz = isCcs811 ? 50000 :
      (requestedType == lil::protocol::EnvironmentalSensorType::kLsm6dsox ||
       requestedType == lil::protocol::EnvironmentalSensorType::kTmp117 ? 400000 : 100000);
  if (!Wire.begin(static_cast<int>(kHardware.sdaPin),
                  static_cast<int>(kHardware.sclPin), busHz)) {
    SENSOR_LOG_PRINTLN("[I2C] Bus initialization failed");
    return false;
  }
  Wire.setTimeOut(isCcs811 ? 100 : 20);
  return startDetectedSensor(requestedType, temperatureOffsetC);
}

bool EnvironmentalSensor::restartSensor(
    lil::protocol::EnvironmentalSensorType type, float temperatureOffsetC) {
  SENSOR_LOG_PRINTLN("Attempt 2: rail power cycle and probe");
  recoveryUsed_ = true;
  Wire.end();
  releaseI2cPins();
  power_->sensorPower(false);
  measurementWaitUs(kPowerCycleOffMs * 1000UL);
  if (measurementBudgetExpired()) {
    SENSOR_LOG_PRINTLN("Attempt 2: measurement deadline reached with rail off");
    return false;
  }
  power_->sensorPower(true);
  measurementWaitUs(kPowerCycleRecoveryMs * 1000UL);
  if (measurementBudgetExpired()) {
    SENSOR_LOG_PRINTLN("Attempt 2: measurement deadline reached after rail power-up");
    return false;
  }
  detectedType_ = lil::protocol::EnvironmentalSensorType::kAutoDetect;
  typeMismatch_ = false;
  return startBusAndSensor(type, temperatureOffsetC);
}

EnvironmentalReading EnvironmentalSensor::readForReport(float temperatureOffsetC) {
  const auto type = detectedType_;
  const auto first = read();
  SENSOR_LOG_PRINTF("Report read: initialized=%d valid=%d retryAlreadyUsed=%d budgetExpired=%d\n",
           int(initialized_), int(first.valid), int(recoveryUsed_), int(measurementBudgetExpired()));
  if (first.valid || !initialized_ || recoveryUsed_ || measurementBudgetExpired() ||
      (type != lil::protocol::EnvironmentalSensorType::kBme280 &&
       type != lil::protocol::EnvironmentalSensorType::kBmp280 &&
       type != lil::protocol::EnvironmentalSensorType::kBme680 &&
       type != lil::protocol::EnvironmentalSensorType::kBme688 &&
       !lil::protocol::hasExtendedPayload(type))) return first;

  // A successful chip ID and configuration do not guarantee a valid conversion.
  // Re-read calibration and acquire a fresh sample within the original deadline.
  pushLowPowerWaitInhibit();
  initialized_ = restartSensor(type, temperatureOffsetC);
  const auto result = initialized_ ? read() : first;
  SENSOR_LOG_PRINTF("Attempt 2: initialized=%d freshValid=%d reusedFirst=%d\n",
           int(initialized_), int(initialized_ && result.valid), int(!initialized_));
  popLowPowerWaitInhibit();
  if (!initialized_) {
    detectedType_ = type;
    end();
  }
  return result;
}

lil::protocol::EnvironmentalSensorType EnvironmentalSensor::probeSensorType(
    uint8_t& detectedAddress) const {
  detectedAddress = 0;
  if ((requestedType_ == lil::protocol::EnvironmentalSensorType::kTmp117 || requestedType_ == lil::protocol::EnvironmentalSensorType::kMax30102) && PrecisionSensors::probe(requestedType_, detectedAddress)) return requestedType_;
  if (lil::protocol::hasExtendedPayload(requestedType_) &&
      ExtendedSensors::probe(requestedType_, detectedAddress)) {
    return requestedType_;
  }
  // Prefer an explicitly selected IMU; otherwise preserve Bosch-first auto detection.
  if (requestedType_ == lil::protocol::EnvironmentalSensorType::kLsm6dsox) {
    for (uint8_t address : {uint8_t(0x6A), uint8_t(0x6B)}) {
      Wire.beginTransmission(address); Wire.write(0x0F);
      if (Wire.endTransmission(false) == 0 && Wire.requestFrom(address, uint8_t(1)) == 1 && Wire.read() == 0x6C) {
        detectedAddress = address; return lil::protocol::EnvironmentalSensorType::kLsm6dsox;
      }
    }
  }
  auto firstBoschType = lil::protocol::EnvironmentalSensorType::kAutoDetect;
  uint8_t firstBoschAddress = 0;
  for (const uint8_t address : kAddresses) {
    if (measurementBudgetExpired()) return lil::protocol::EnvironmentalSensorType::kAutoDetect;
    Wire.beginTransmission(address);
    Wire.write(kChipIdRegister);
    const uint8_t error = Wire.endTransmission(false);
    if (error != 0) {
      SENSOR_LOG_PRINTF("[I2C] Bosch probe failed: address=0x%02X error=%u\n", address, error);
      continue;
    }
    if (Wire.requestFrom(address, static_cast<uint8_t>(1)) != 1) {
      SENSOR_LOG_PRINTF("[I2C] Bosch chip ID read incomplete: address=0x%02X\n", address);
      continue;
    }
    const uint8_t chipId = Wire.read();
    SENSOR_LOG_PRINTF("[I2C] Bosch probe: address=0x%02X chip_id=0x%02X\n", address, chipId);
    auto type = lil::protocol::EnvironmentalSensorType::kAutoDetect;
    if (chipId == kBme680ChipId) {
      uint8_t variant = 0;
      Wire.beginTransmission(address);
      Wire.write(kBme68xVariantRegister);
      if (Wire.endTransmission(false) == 0 &&
          Wire.requestFrom(address, static_cast<uint8_t>(1)) == 1) {
        variant = Wire.read();
      }
      type = (variant == kBme688VariantId)
                 ? lil::protocol::EnvironmentalSensorType::kBme688
                 : lil::protocol::EnvironmentalSensorType::kBme680;
    } else if (chipId == kBme280ChipId) {
      type = lil::protocol::EnvironmentalSensorType::kBme280;
    } else if (chipId == kBmp280ChipId) {
      type = lil::protocol::EnvironmentalSensorType::kBmp280;
    }
    if (type == lil::protocol::EnvironmentalSensorType::kAutoDetect) continue;
    if (requestedType_ == lil::protocol::EnvironmentalSensorType::kAutoDetect ||
        requestedType_ == type) {
      detectedAddress = address;
      return type;
    }
    // An explicitly selected Bosch type may occupy the other I2C address.
    // Remember a positive mismatch, but search both before rejecting it.
    if (firstBoschAddress == 0) {
      firstBoschAddress = address;
      firstBoschType = type;
    }
  }
  if (firstBoschAddress != 0) {
    detectedAddress = firstBoschAddress;
    return firstBoschType;
  }
  for (uint8_t address : {uint8_t(0x6A), uint8_t(0x6B)}) {
    if (requestedType_ == lil::protocol::EnvironmentalSensorType::kLsm6dsox) break;
    if (measurementBudgetExpired()) return lil::protocol::EnvironmentalSensorType::kAutoDetect;
    Wire.beginTransmission(address); Wire.write(0x0F);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(address, uint8_t(1)) == 1 && Wire.read() == 0x6C) {
      detectedAddress = address; return lil::protocol::EnvironmentalSensorType::kLsm6dsox;
    }
  }
  for (auto type : {lil::protocol::EnvironmentalSensorType::kTmp117, lil::protocol::EnvironmentalSensorType::kMax30102}) {
    if (requestedType_ == type) continue;
    if (measurementBudgetExpired()) return lil::protocol::EnvironmentalSensorType::kAutoDetect;
    if (PrecisionSensors::probe(type, detectedAddress)) return type;
  }
  for (auto type : {lil::protocol::EnvironmentalSensorType::kAs7341,
                    lil::protocol::EnvironmentalSensorType::kVl53l1x,
                    lil::protocol::EnvironmentalSensorType::kCcs811}) {
    if (requestedType_ == type) continue;
    if (measurementBudgetExpired()) return lil::protocol::EnvironmentalSensorType::kAutoDetect;
    if (type == lil::protocol::EnvironmentalSensorType::kCcs811 &&
        requestedType_ == lil::protocol::EnvironmentalSensorType::kAutoDetect) {
      measurementWaitUs((kCcs811PowerUpMs - kSensorPowerUpMs) * 1000UL);
    }
    if (ExtendedSensors::probe(type, detectedAddress)) return type;
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
  // Auto-detect starts the bus at 100 kHz for Bosch/MAX30102 safety; upgrade
  // to 400 kHz Fast Mode once LSM6DSOX or TMP117 is confirmed.
  if (requestedType_ == lil::protocol::EnvironmentalSensorType::kAutoDetect &&
      (type == lil::protocol::EnvironmentalSensorType::kLsm6dsox ||
       type == lil::protocol::EnvironmentalSensorType::kTmp117)) {
    Wire.setClock(lil::timing::kI2cHz);
  }
  bool started = false;
  if (type == lil::protocol::EnvironmentalSensorType::kTmp117 ||
      type == lil::protocol::EnvironmentalSensorType::kMax30102) {
    auto* prec = std::get_if<PrecisionSensors>(&driver_);
    if (!prec) prec = &driver_.emplace<PrecisionSensors>();
    started = prec->begin(type, address);
  } else if (type == lil::protocol::EnvironmentalSensorType::kLsm6dsox) {
    auto* imu = std::get_if<Lsm6dsoxDriver>(&driver_);
    if (!imu) imu = &driver_.emplace<Lsm6dsoxDriver>();
    started = imu->begin(address);
  } else if (type == lil::protocol::EnvironmentalSensorType::kBme280 ||
             type == lil::protocol::EnvironmentalSensorType::kBmp280) {
    auto* bme = std::get_if<Bme280Driver>(&driver_);
    if (!bme) bme = &driver_.emplace<Bme280Driver>();
    started = bme->begin(address);
  } else if (type == lil::protocol::EnvironmentalSensorType::kBme680 ||
             type == lil::protocol::EnvironmentalSensorType::kBme688) {
    auto* bme = std::get_if<Bme680Driver>(&driver_);
    if (!bme) bme = &driver_.emplace<Bme680Driver>();
    started = bme->begin(address, temperatureOffsetC,
                         type == lil::protocol::EnvironmentalSensorType::kBme688);
  } else if (lil::protocol::hasExtendedPayload(type)) {
    auto* ext = std::get_if<ExtendedSensors>(&driver_);
    if (!ext) ext = &driver_.emplace<ExtendedSensors>();
    started = ext->begin(type, address);
  }
  if (started) {
    detectedType_ = type;
  }
  return started;
}

EnvironmentalReading EnvironmentalSensor::readContinuous() {
  if (!initialized_) {
    EnvironmentalReading failed{};
    failed.sensorType = detectedType_ == lil::protocol::EnvironmentalSensorType::kAutoDetect
                            ? requestedType_
                            : detectedType_;
    return failed;
  }
  beginMeasurementBudget();
  EnvironmentalReading result{};
  if (lil::protocol::hasExtendedPayload(detectedType_)) {
    if (auto* ext = std::get_if<ExtendedSensors>(&driver_)) {
      result = ext->readContinuous();
    }
  } else {
    result = read();
  }
  endMeasurementBudget();
  return result;
}

EnvironmentalReading EnvironmentalSensor::read(bool pollFirst) {
  if (initialized_) {
    if (auto* imu = std::get_if<Lsm6dsoxDriver>(&driver_)) {
      if (detectedType_ == lil::protocol::EnvironmentalSensorType::kLsm6dsox) return imu->read(pollFirst);
    }
    if (auto* prec = std::get_if<PrecisionSensors>(&driver_)) {
      if (detectedType_ == lil::protocol::EnvironmentalSensorType::kTmp117 ||
          detectedType_ == lil::protocol::EnvironmentalSensorType::kMax30102) {
        return prec->read(pollFirst);
      }
    }
  }
  if (requestedType_ == lil::protocol::EnvironmentalSensorType::kDisabled) {
    EnvironmentalReading disabled{};
    disabled.valid = true;
    disabled.sensorType = lil::protocol::EnvironmentalSensorType::kDisabled;
    return disabled;
  }
  if (!initialized_ || measurementBudgetExpired()) {
    EnvironmentalReading failed{};
    failed.sensorType = detectedType_ == lil::protocol::EnvironmentalSensorType::kAutoDetect ? requestedType_ : detectedType_;
    return failed;
  }
  if (lil::protocol::isBoschGasSensor(detectedType_)) {
    if (auto* bme = std::get_if<Bme680Driver>(&driver_)) return bme->read();
  }
  if (lil::protocol::hasExtendedPayload(detectedType_)) {
    if (auto* ext = std::get_if<ExtendedSensors>(&driver_)) return ext->read();
  }
  if (auto* bme = std::get_if<Bme280Driver>(&driver_)) return bme->read();
  return EnvironmentalReading{};
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
  if (lil::protocol::isBoschGasSensor(detectedType_) ||
      lil::protocol::isBoschGasSensor(activeType)) {
    if (auto* bme = std::get_if<Bme680Driver>(&driver_)) {
      bme->prepareForDeepSleep(seconds);
    } else if (std::unique_ptr<Bme680Driver> temporary{
                   new (std::nothrow) Bme680Driver()}) {
      temporary->prepareForDeepSleep(seconds);
    }
  }
}

uint32_t EnvironmentalSensor::bme680RecommendedSleepSeconds(
    uint32_t fallbackSeconds) const {
  if (const auto* bme = std::get_if<Bme680Driver>(&driver_)) {
    return bme->recommendedSleepSeconds(fallbackSeconds);
  }
  return fallbackSeconds;
}

bool EnvironmentalSensor::clearIaqState() {
  if (auto* bme = std::get_if<Bme680Driver>(&driver_)) {
    return bme->clearPersistentState();
  }
  std::unique_ptr<Bme680Driver> temporary(new (std::nothrow) Bme680Driver());
  return temporary && temporary->clearPersistentState();
}

lil::protocol::EnvironmentalSensorType EnvironmentalSensor::detectedType()
    const {
  return detectedType_;
}

}  // namespace sensor

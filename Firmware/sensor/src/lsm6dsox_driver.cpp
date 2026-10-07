#include "lsm6dsox_driver.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

namespace sensor {
void Lsm6dsoxDriver::discardWindow() {
  accelCount_ = gyroCount_ = 0;
  for (auto& axes : sums_) for (auto& value : axes) value = 0;
  motion_ = {};
  feedback_.restartFilter();
}
bool Lsm6dsoxDriver::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address_);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}
bool Lsm6dsoxDriver::readRegisters(uint8_t reg, uint8_t* data, uint8_t size) {
  Wire.beginTransmission(address_);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(address_, size) != size)
    return false;
  for (uint8_t i = 0; i < size; ++i) data[i] = Wire.read();
  return true;
}
bool Lsm6dsoxDriver::begin(uint8_t address) {
  const auto feedback = feedback_;
  *this = Lsm6dsoxDriver{};
  feedback_ = feedback;
  feedback_.restartFilter();
  address_ = address;
  uint8_t value = 0;
  if (!readRegisters(0x0F, &value, 1) || value != 0x6C ||
      !writeRegister(0x12, 0x01)) return false;
  const uint32_t start = millis();
  do {
    delay(1);
    if (!readRegisters(0x12, &value, 1)) return false;
    if (millis() - start >= 100) return false;
  } while (value & 1);
  // Preserve default DEN bits, disable unused I3C. BDU and auto increment.
  const uint8_t registers[][2] = {
      {0x18, 0xE2}, {0x12, 0x44}, {0x10, 0x48}, {0x11, 0x44},
      {0x09, 0x44}};
  for (const auto& entry : registers) {
    if (!writeRegister(entry[0], entry[1]) ||
        !readRegisters(entry[0], &value, 1) || value != entry[1]) return false;
  }
  // Discard startup transients before enabling continuous FIFO mode.
  delay(100);
  if (!writeRegister(0x0A, 0x06) || !readRegisters(0x0A, &value, 1) || value != 6)
    return false;
  initialized_ = true;
  lastPollMs_ = millis();
  delay(25);
  return poll();
}
bool Lsm6dsoxDriver::poll() {
  if (!initialized_) return false;
  const uint32_t now = millis();
  const bool pollingGap = now - lastPollMs_ >= 200;
  lastPollMs_ = now;
  uint8_t status[2];
  if (!readRegisters(0x3A, status, 2)) {
    failed_ = true; initialized_ = false; discardWindow(); return false;
  }
  const uint16_t count = status[0] | ((status[1] & 3U) << 8);
  if ((status[1] & 0x48) || pollingGap || count > 512) {
    // Both FIFO_OVR_IA and OVER_RUN_LATCHED indicate missing samples. Do not
    // average the partial interval before the gap with the surviving tail.
    // FIFO entries carry no enabled timestamp: after a scheduling stall their
    // real ages are unknown. Discard them instead of dating old data at drain
    // time. A count above the physical 512-entry FIFO is also invalid.
    overflow_ = overflow_ || (status[1] & 0x48) || count > 512;
    gap_ = true; discardWindow();
    if (!writeRegister(0x0A, 0) || !writeRegister(0x0A, 6)) {
      failed_ = true; initialized_ = false;
    }
    return !failed_;
  }
  // Bound each drain to the FIFO snapshot; new samples wait for the next poll.
  for (uint16_t i = 0; i < count; ++i) {
    uint8_t data[7];
    if (!readRegisters(0x78, data, sizeof(data))) {
      failed_ = true; initialized_ = false; discardWindow(); return false;
    }
    const uint8_t tag = data[0] >> 3;
    if (tag != 1 && tag != 2) continue;
    float magnitudeSquared = 0;
    for (uint8_t axis = 0; axis < 3; ++axis) {
      const int16_t raw = static_cast<int16_t>(uint16_t(data[1 + axis * 2]) |
                                             uint16_t(data[2 + axis * 2]) << 8);
      const float scaled = raw * (tag == 2 ? 0.000122F : 0.0175F);
      sums_[tag == 2 ? 0 : 1][axis] += scaled;
      if (tag == 2) motion_.accelerationG[axis] = scaled;
      else motion_.angularRateDps[axis] = scaled;
      magnitudeSquared += scaled * scaled;
    }
    const float magnitude = sqrtf(magnitudeSquared);
    if (tag == 2) {
      feedback_.observe(magnitude);
      motion_.peakAccelerationG = fmaxf(motion_.peakAccelerationG, magnitude);
      ++accelCount_; lastAccelMs_ = millis();
    } else {
      motion_.peakAngularRateDps = fmaxf(motion_.peakAngularRateDps, magnitude);
      ++gyroCount_; lastGyroMs_ = millis();
    }
  }
  return true;
}
EnvironmentalReading Lsm6dsoxDriver::read() {
  poll();
  EnvironmentalReading result{};
  result.sensorType = lil::protocol::EnvironmentalSensorType::kLsm6dsox;
  result.valid = initialized_ && !failed_ && accelCount_ && gyroCount_ &&
                 millis() - lastAccelMs_ < 200 && millis() - lastGyroMs_ < 200;
  result.motion = motion_;
  result.motionFeedback = feedback_.value();
  for (uint8_t axis = 0; axis < 3; ++axis) {
    if (accelCount_) result.motion.accelerationG[axis] = sums_[0][axis] / accelCount_;
    if (gyroCount_) result.motion.angularRateDps[axis] = sums_[1][axis] / gyroCount_;
    sums_[0][axis] = sums_[1][axis] = 0;
  }
  result.live.flags = lil::protocol::kLiveTimingKnown | (result.valid ? lil::protocol::kLiveSampleFresh : 0);
  result.live.flags |= lil::protocol::kMotionFeedbackPresent;
  if (gap_ || overflow_ || failed_) result.live.flags |= lil::protocol::kLiveGap;
  result.live.samplePeriodUs = 1000000UL / lil::timing::kImuHz;
  result.live.windowMs = min(uint32_t(65534), max(accelCount_, gyroCount_) * 1000UL / lil::timing::kImuHz);
  result.live.acquisitionAgeMs = min(uint32_t(65534), max(millis() - lastAccelMs_, millis() - lastGyroMs_));
  result.motion.sampleCount = static_cast<uint16_t>(min(accelCount_, uint32_t(UINT16_MAX)));
  result.motion.fifoOverrun = overflow_;
  if (result.valid) result.capabilities = lil::protocol::kMotion;
  motion_.peakAccelerationG = motion_.peakAngularRateDps = 0;
  accelCount_ = gyroCount_ = 0;
  gap_ = overflow_ = failed_ = false;
  return result;
}
}  // namespace sensor

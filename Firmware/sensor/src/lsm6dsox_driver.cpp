#include "lsm6dsox_driver.h"
#ifdef ESP_PLATFORM
#include "measurement_budget.h"
#endif
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

namespace sensor {
namespace {
constexpr uint8_t kFifoWordBytes = 7;
#ifdef ESP_PLATFORM
constexpr uint8_t kFifoBurstWords = 18;  // 126 bytes, fits the 128-byte ESP32 Wire buffer.
#else
constexpr uint8_t kFifoBurstWords = 8;  // 56 bytes, matches host Wire regression expectations.
#endif
constexpr float kAccelerationScale = 0.000122F;
constexpr float kGyroscopeScale = 0.0175F;
}

void Lsm6dsoxDriver::discardWindow() {
  accelCount_ = gyroCount_ = 0;
  for (auto& axes : rawSums_) for (auto& value : axes) value = 0;
  motion_ = {};
  accelPeakRawSq_ = 0;
  gyroPeakRawSq_ = 0;
  gyroPeakSquared_ = 0;
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
  address_ = address;
  initialized_ = failed_ = overflow_ = gap_ = false;
  lastAccelMs_ = lastGyroMs_ = lastPollMs_ = 0;
  discardWindow();
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
#ifdef ESP_PLATFORM
  measurementWaitUs(100000);
#else
  delay(100);
#endif
  if (!writeRegister(0x0A, 0x06) || !readRegisters(0x0A, &value, 1) || value != 6)
    return false;
  initialized_ = true;
  lastPollMs_ = millis();
#ifdef ESP_PLATFORM
  measurementWaitUs(25000);
#else
  delay(25);
#endif
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
  // The LSM6DSOX wraps 0x7E back to the tag at 0x78 (ST AN5272 §4.7.1).
  // Read complete words and drain only the original snapshot.
  for (uint16_t remaining = count; remaining;) {
    const uint8_t words = min(remaining, uint16_t(kFifoBurstWords));
    uint8_t bytes[kFifoBurstWords * kFifoWordBytes];
    if (!readRegisters(0x78, bytes, words * kFifoWordBytes)) {
      // A short burst may already have consumed words from the FIFO.
      failed_ = true; initialized_ = false; discardWindow(); return false;
    }
    const uint32_t readAt = millis();
    for (uint8_t i = 0; i < words; ++i) {
      const uint8_t* data = bytes + i * kFifoWordBytes;
      const uint8_t tag = data[0] >> 3;
      if (tag != 1 && tag != 2) continue;
      const int16_t rx = static_cast<int16_t>(uint16_t(data[1]) | (uint16_t(data[2]) << 8));
      const int16_t ry = static_cast<int16_t>(uint16_t(data[3]) | (uint16_t(data[4]) << 8));
      const int16_t rz = static_cast<int16_t>(uint16_t(data[5]) | (uint16_t(data[6]) << 8));
      if (tag == 2) {
        auto& rawSums = rawSums_[0];
        rawSums[0] += rx; rawSums[1] += ry; rawSums[2] += rz;
        // Exact integer squared-norm avoids all software floating-point math
        // and sqrtf at 104 Hz on RV32IMC; scale directly into Q16 for motion
        // feedback and defer peak acceleration sqrtf until window readout.
        const uint32_t rawSq = static_cast<uint32_t>(int32_t(rx) * rx) +
                               static_cast<uint32_t>(int32_t(ry) * ry) +
                               static_cast<uint32_t>(int32_t(rz) * rz);
        if (rawSq > accelPeakRawSq_) accelPeakRawSq_ = rawSq;
        const uint32_t magRawQ8 = lil::timing::isqrt64(static_cast<uint64_t>(rawSq) << 16);
        // kAccelerationScale * 65536 = 7.995392 -> 7.995392 * 256 = 2046.820352.
        const int32_t magQ16 = static_cast<int32_t>((static_cast<uint64_t>(magRawQ8) * 2047ULL + 32768ULL) >> 16);
        feedback_.observeQ16(magQ16);
        ++accelCount_; lastAccelMs_ = readAt;
      } else {
        auto& rawSums = rawSums_[1];
        rawSums[0] += rx; rawSums[1] += ry; rawSums[2] += rz;
        // Exact integer squared-norm lower-bound pruning (<= 3 * 2^30 < UINT32_MAX):
        // skip soft-float scaling whenever this sample is strictly below the
        // current peak beyond single-precision rounding margin (~0.0002%),
        // preserving exact bit-for-bit IEEE-754 peakAngularRateDps.
        const uint32_t rawSq = static_cast<uint32_t>(int32_t(rx) * rx) +
                               static_cast<uint32_t>(int32_t(ry) * ry) +
                               static_cast<uint32_t>(int32_t(rz) * rz);
        const uint32_t margin = (gyroPeakRawSq_ >> 18) + 16U;
        if (gyroCount_ == 0 || rawSq >= gyroPeakRawSq_ ||
            gyroPeakRawSq_ - rawSq <= margin) {
          if (rawSq > gyroPeakRawSq_) gyroPeakRawSq_ = rawSq;
          const int16_t axes[3] = {rx, ry, rz};
          float magnitudeSquared = 0;
          for (uint8_t axis = 0; axis < 3; ++axis) {
            const float scaled = axes[axis] * kGyroscopeScale;
            magnitudeSquared += scaled * scaled;
          }
          gyroPeakSquared_ = fmaxf(gyroPeakSquared_, magnitudeSquared);
        }
        ++gyroCount_; lastGyroMs_ = readAt;
      }
    }
    remaining -= words;
  }
  return true;
}
EnvironmentalReading Lsm6dsoxDriver::read(bool pollFirst) {
  if (pollFirst) poll();
  EnvironmentalReading result{};
  result.sensorType = lil::protocol::EnvironmentalSensorType::kLsm6dsox;
  const uint32_t now = millis();
  const uint32_t age = max(now - lastAccelMs_, now - lastGyroMs_);
  result.valid = initialized_ && !failed_ && accelCount_ && gyroCount_ && age < 200;
  result.motion = motion_;
  result.motion.peakAccelerationG =
      accelCount_ ? sqrtf(static_cast<float>(accelPeakRawSq_)) * kAccelerationScale : 0.0F;
  result.motion.peakAngularRateDps = gyroCount_ ? sqrtf(gyroPeakSquared_) : 0;
  result.motionFeedback = feedback_.value();
  for (uint8_t axis = 0; axis < 3; ++axis) {
    // Scale the exact sum at readout. The C3 has no floating-point unit;
    // summing scaled values would use a software add for every FIFO axis.
    result.motion.accelerationG[axis] =
        accelCount_ ? float(rawSums_[0][axis]) / accelCount_ * kAccelerationScale : 0.0F;
    result.motion.angularRateDps[axis] =
        gyroCount_ ? float(rawSums_[1][axis]) / gyroCount_ * kGyroscopeScale : 0.0F;
    rawSums_[0][axis] = rawSums_[1][axis] = 0;
  }
  result.live.flags = lil::protocol::kLiveTimingKnown | (result.valid ? lil::protocol::kLiveSampleFresh : 0);
  result.live.flags |= lil::protocol::kMotionFeedbackPresent;
  if (gap_ || overflow_ || failed_) result.live.flags |= lil::protocol::kLiveGap;
  result.live.samplePeriodUs = 1000000UL / lil::timing::kImuHz;
  result.live.windowMs = min(uint32_t(65534), max(accelCount_, gyroCount_) * 1000UL / lil::timing::kImuHz);
  result.live.acquisitionAgeMs = min(uint32_t(65534), age);
  result.motion.sampleCount = static_cast<uint16_t>(min(accelCount_, uint32_t(UINT16_MAX)));
  result.motion.fifoOverrun = overflow_;
  if (result.valid) result.capabilities = lil::protocol::kMotion;
  motion_.peakAccelerationG = motion_.peakAngularRateDps = 0;
  accelPeakRawSq_ = 0;
  gyroPeakRawSq_ = 0;
  gyroPeakSquared_ = 0;
  accelCount_ = gyroCount_ = 0;
  gap_ = overflow_ = failed_ = false;
  return result;
}
}  // namespace sensor

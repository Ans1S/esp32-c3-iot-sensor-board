#include "bme280_driver.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "measurement_budget.h"
#include <sensor_log.h>

namespace sensor {
namespace {
uint16_t little16(const uint8_t* b) { return uint16_t(b[0]) | (uint16_t(b[1]) << 8); }
int16_t signed12(uint16_t value) {
  return static_cast<int16_t>(value & 0x800 ? int32_t(value) - 4096 : value);
}
// Exact IEEE-754 power-of-two scaling via exponent adjustment avoids software
// double multiplications (__muldf3) on FPU-less RV32IMC while preserving every
// output bit of Bosch's appendix 8.1 compensation equations.
inline double scalePow2(double value, int expDelta) {
  uint64_t bits;
  memcpy(&bits, &value, sizeof(bits));
  const int exponent = static_cast<int>((bits >> 52) & 0x7FFU);
  if (exponent != 0 && exponent != 0x7FF) {
    const int nextExponent = exponent + expDelta;
    if (nextExponent > 0 && nextExponent < 0x7FF) {
      bits = (bits & ~(uint64_t(0x7FF) << 52)) |
             (static_cast<uint64_t>(nextExponent) << 52);
      memcpy(&value, &bits, sizeof(bits));
      return value;
    }
  }
  return ldexp(value, expDelta);
}
}

bool Bme280Driver::readBytes(uint8_t reg, uint8_t* bytes, uint8_t count) {
  if (measurementBudgetExpired()) return false;
  Wire.beginTransmission(address_); Wire.write(reg);
  const uint8_t error = Wire.endTransmission(false);
  if (error != 0) {
    SENSOR_LOG_PRINTF("[BME280] Register select failed: address=0x%02X reg=0x%02X error=%u\n",
                      address_, reg, error);
    return false;
  }
  const uint8_t received = Wire.requestFrom(address_, count);
  if (received != count) {
    SENSOR_LOG_PRINTF("[BME280] Short read: address=0x%02X reg=0x%02X expected=%u received=%u\n",
                      address_, reg, count, received);
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) bytes[i] = Wire.read();
  return true;
}


bool Bme280Driver::write8(uint8_t reg, uint8_t value) {
  if (measurementBudgetExpired()) return false;
  Wire.beginTransmission(address_); Wire.write(reg); Wire.write(value);
  const uint8_t error = Wire.endTransmission();
  if (error != 0) {
    SENSOR_LOG_PRINTF("[BME280] Write failed: address=0x%02X reg=0x%02X error=%u\n",
                      address_, reg, error);
  }
  return error == 0;
}

bool Bme280Driver::waitReady(uint32_t timeoutMs, uint8_t* control) {
  const uint32_t started = millis();
  do {
    uint8_t status[2];
    if (!readBytes(0xF3, status, control ? 2 : 1)) return false;
    if ((status[0] & 0x09) == 0) {
      if (control) *control = status[1];
      return true;
    }
    measurementWaitUs(1000);
  } while (millis() - started < timeoutMs && !measurementBudgetExpired());
  SENSOR_LOG_PRINTLN("[BME280] Status did not become ready before the deadline");
  return false;
}

bool Bme280Driver::begin(uint8_t address) {
  initialized_ = false; isBmp280_ = false; address_ = address;
  uint8_t id;
  if (!readBytes(0xD0, &id, 1) || (id != 0x60 && id != 0x58)) return false;
  isBmp280_ = (id == 0x58);
  SENSOR_LOG_PRINTLN("Soft reset: write 0xB6; read calibration after ready");
  if (!write8(0xE0, 0xB6)) return false;
  measurementWaitUs(3000); // Reset/NVM copy must finish before reading trims.
  if (!waitReady(100)) return false;

  uint8_t trim[26]{}, humidity[7]{}, check[26]{};
  const uint8_t trimSize = isBmp280_ ? 24 : 26;
  // I2C has no CRC. Check transfer success and consistency of NVM blocks.
  if (!readBytes(0x88, trim, trimSize) || !readBytes(0x88, check, trimSize) ||
      memcmp(trim, check, trimSize) != 0) {
    SENSOR_LOG_PRINTLN("[BME280] Calibration transfer failed or repeated reads disagree");
    return false;
  }
  if (!isBmp280_ &&
      (!readBytes(0xE1, humidity, sizeof(humidity)) ||
       !readBytes(0xE1, check, sizeof(humidity)) ||
       memcmp(humidity, check, sizeof(humidity)) != 0)) {
    SENSOR_LOG_PRINTLN("[BME280] Humidity calibration transfer failed or repeated reads disagree");
    return false;
  }
  t1_ = little16(trim); t2_ = static_cast<int16_t>(little16(trim + 2));
  t3_ = static_cast<int16_t>(little16(trim + 4)); p1_ = little16(trim + 6);
  if (t1_ == 0 || t1_ == 0xFFFF || p1_ == 0 || p1_ == 0xFFFF) {
    SENSOR_LOG_PRINTF("[BME280] Invalid calibration: T1=%u P1=%u\n", t1_, p1_);
    return false;
  }
  // A module returned one repeated 16-bit word across both complete trim
  // blocks, identically on every read. Such data can yield in-range pressure.
  repeatedCalibration_ = memcmp(trim, trim + 2, trimSize - 2) == 0 &&
                         (isBmp280_ || memcmp(humidity, trim, sizeof(humidity)) == 0);
  for (unsigned i = 0; i < 8; ++i) p_[i] = static_cast<int16_t>(little16(trim + 8 + 2*i));
  if (!isBmp280_) {
    h1_ = trim[25]; h2_ = static_cast<int16_t>(little16(humidity)); h3_ = humidity[2];
    h4_ = signed12((uint16_t(humidity[3]) << 4) | (humidity[4] & 15));
    h5_ = signed12((uint16_t(humidity[5]) << 4) | (humidity[4] >> 4));
    h6_ = static_cast<int8_t>(humidity[6]);
  }

  // Configure while asleep; ctrl_hum only takes effect after ctrl_meas (BME280).
  // Do not start a redundant conversion during initialization.
  const uint8_t config[][2] = {{0xF2, 1}, {0xF5, 0}, {0xF4, 0x24}};
  for (size_t idx = isBmp280_ ? 1 : 0; idx < 3; ++idx) {
    const auto& entry = config[idx];
    uint8_t actual;
    if (!write8(entry[0], entry[1]) || !readBytes(entry[0], &actual, 1))
      return false;
    if (actual != entry[1]) {
      SENSOR_LOG_PRINTF("[BME280] Configuration mismatch: reg=0x%02X expected=0x%02X actual=0x%02X\n",
                        entry[0], entry[1], actual);
      return false;
    }
  }
  initialized_ = true;
  return true;
}

EnvironmentalReading Bme280Driver::read() {
  EnvironmentalReading reading{};
  reading.sensorType = isBmp280_
      ? lil::protocol::EnvironmentalSensorType::kBmp280
      : lil::protocol::EnvironmentalSensorType::kBme280;
  if (!initialized_ || !write8(0xF4, 0x25)) return reading;
  // BST-BME280-DS002 §9.1 / BST-BMP280-DS001 §3.8.1: x1 max conversion <= 9.3 ms.
  // Polling immediately can see measuring=0 before the conversion starts.
  measurementWaitUs(10000);
  uint8_t control, raw[8]{};
  // Read the adjacent status and control registers together. Keep the raw
  // channels in their own burst after verifying that the conversion ended.
  if (!waitReady(50, &control)) return reading;
  if (control != 0x24) {
    SENSOR_LOG_PRINTF("[BME280] Forced conversion control mismatch: expected=0x24 actual=0x%02X\n", control);
    return reading;
  }
  const uint8_t rawCount = isBmp280_ ? 6 : 8;
  if (!readBytes(0xF7, raw, rawCount)) return reading;
  const uint32_t adcP = (uint32_t(raw[0]) << 12) | (uint32_t(raw[1]) << 4) | (raw[2] >> 4);
  const uint32_t adcT = (uint32_t(raw[3]) << 12) | (uint32_t(raw[4]) << 4) | (raw[5] >> 4);
  const uint16_t adcH = isBmp280_ ? 0 : ((uint16_t(raw[6]) << 8) | raw[7]);
  if (repeatedCalibration_) {
    SENSOR_LOG_PRINTLN("[BME280] Rejected repeated calibration word across register blocks");
    return reading;
  }
  if (adcP == 0x80000 || adcT == 0x80000 || (!isBmp280_ && adcH == 0x8000)) {
    SENSOR_LOG_PRINTLN("[BME280] Conversion returned a skipped/uninitialized channel");
    return reading;
  }

  // Bosch double-precision compensation, datasheet appendix 8.1 (BME280/BMP280).
  const int32_t temperatureDelta = int32_t(adcT) - int32_t(t1_) * 16;
  double v1 = scalePow2(double(int64_t(temperatureDelta) * t2_), -14);
  double v2 = scalePow2(double(temperatureDelta), -17);
  v2 = v2 * v2 * t3_;
  const double fine = v1 + v2;
  reading.temperatureC = fine / 5120.0;
  v1 = scalePow2(fine, -1) - 64000.0;
  v2 = scalePow2(v1 * v1 * p_[4], -15) + scalePow2(v1 * p_[3], 1);
  v2 = scalePow2(v2, -2) + double(int32_t(p_[2]) * 65536);
  v1 = scalePow2(scalePow2(p_[1] * v1 * v1, -19) + p_[0] * v1, -19);
  v1 = (1.0 + scalePow2(v1, -15)) * p1_;
  if (v1 == 0) {
    SENSOR_LOG_PRINTLN("Pressure divisor is zero");
    return reading;
  }
  double pressure = (double(1048576U - adcP) - scalePow2(v2, -12)) * 6250.0 / v1;
  v1 = scalePow2(p_[7] * pressure * pressure, -31);
  v2 = scalePow2(pressure * p_[6], -15);
  reading.pressureHpa = (pressure + scalePow2(v1 + v2 + p_[5], -4)) / 100.0;
  if (!isBmp280_) {
    double h = fine - 76800.0;
    h = (adcH - (double(int32_t(h4_) * 64) + scalePow2(h5_ * h, -14))) *
        (scalePow2(double(h2_), -16) * (1.0 + scalePow2(h6_ * h, -26) * (1.0 + scalePow2(h3_ * h, -26))));
    h *= 1.0 - scalePow2(h1_ * h, -19);
    if (!isfinite(h)) return reading;
    reading.humidityPercent = constrain(h, 0.0, 100.0);
  }
  reading.valid = isfinite(reading.temperatureC) && reading.temperatureC >= -40 &&
      reading.temperatureC <= 85 && isfinite(reading.pressureHpa) &&
      reading.pressureHpa >= 300 && reading.pressureHpa <= 1100;
  if (reading.valid) {
    reading.capabilities = lil::protocol::kTemperature | lil::protocol::kPressure |
                           (isBmp280_ ? uint16_t(0) : static_cast<uint16_t>(lil::protocol::kHumidity));
  }
  return reading;
}
}  // namespace sensor

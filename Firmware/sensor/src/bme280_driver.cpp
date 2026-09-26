#include "bme280_driver.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "measurement_budget.h"

namespace sensor {
namespace {
uint16_t little16(const uint8_t* b) { return uint16_t(b[0]) | (uint16_t(b[1]) << 8); }
int16_t signed12(uint16_t value) {
  return static_cast<int16_t>(value & 0x800 ? int32_t(value) - 4096 : value);
}
}

bool Bme280Driver::readBytes(uint8_t reg, uint8_t* bytes, uint8_t count) {
  if (measurementBudgetExpired()) return false;
  Wire.beginTransmission(address_); Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(address_, count) != count)
    return false;
  for (uint8_t i = 0; i < count; ++i) bytes[i] = Wire.read();
  return true;
}

bool Bme280Driver::write8(uint8_t reg, uint8_t value) {
  if (measurementBudgetExpired()) return false;
  Wire.beginTransmission(address_); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool Bme280Driver::waitReady(uint32_t timeoutMs) {
  const uint32_t started = millis();
  do {
    uint8_t status;
    if (!readBytes(0xF3, &status, 1)) return false;
    if ((status & 0x09) == 0) return true;
    measurementWaitUs(1000);
  } while (millis() - started < timeoutMs && !measurementBudgetExpired());
  return false;
}

bool Bme280Driver::begin(uint8_t address) {
  initialized_ = false; address_ = address;
  uint8_t id;
  if (!readBytes(0xD0, &id, 1) || id != 0x60 || !write8(0xE0, 0xB6)) return false;
  measurementWaitUs(3000); // Reset/NVM copy must finish before reading trims.
  if (!waitReady(100)) return false;

  uint8_t trim[26], humidity[7], check[26];
  // I2C has no CRC. Check transfer success and consistency of both NVM blocks.
  if (!readBytes(0x88, trim, sizeof(trim)) || !readBytes(0x88, check, sizeof(trim)) ||
      memcmp(trim, check, sizeof(trim)) != 0 ||
      !readBytes(0xE1, humidity, sizeof(humidity)) ||
      !readBytes(0xE1, check, sizeof(humidity)) ||
      memcmp(humidity, check, sizeof(humidity)) != 0) return false;
  t1_ = little16(trim); t2_ = static_cast<int16_t>(little16(trim + 2));
  t3_ = static_cast<int16_t>(little16(trim + 4)); p1_ = little16(trim + 6);
  if (t1_ == 0 || t1_ == 0xFFFF || p1_ == 0 || p1_ == 0xFFFF) return false;
  for (unsigned i = 0; i < 8; ++i) p_[i] = static_cast<int16_t>(little16(trim + 8 + 2*i));
  h1_ = trim[25]; h2_ = static_cast<int16_t>(little16(humidity)); h3_ = humidity[2];
  h4_ = signed12((uint16_t(humidity[3]) << 4) | (humidity[4] & 15));
  h5_ = signed12((uint16_t(humidity[5]) << 4) | (humidity[4] >> 4));
  h6_ = static_cast<int8_t>(humidity[6]);

  // Configure while asleep; ctrl_hum only takes effect after ctrl_meas.
  // Do not start a redundant conversion during initialization.
  const uint8_t config[][2] = {{0xF2, 1}, {0xF5, 0}, {0xF4, 0x24}};
  for (const auto& entry : config) {
    uint8_t actual;
    if (!write8(entry[0], entry[1]) || !readBytes(entry[0], &actual, 1) || actual != entry[1])
      return false;
  }
  initialized_ = true;
  return true;
}

EnvironmentalReading Bme280Driver::read() {
  EnvironmentalReading reading{};
  reading.sensorType = lil::protocol::EnvironmentalSensorType::kBme280;
  if (!initialized_ || !write8(0xF4, 0x25)) return reading;
  // BST-BME280-DS002 section 9.1: x1 T/P/H maximum conversion time is 9.3 ms.
  // Polling immediately can see measuring=0 before the conversion starts.
  measurementWaitUs(10000);
  uint8_t control, raw[8];
  if (!waitReady(50) || !readBytes(0xF4, &control, 1) || control != 0x24 ||
      !readBytes(0xF7, raw, sizeof(raw))) return reading;
  const uint32_t adcP = (uint32_t(raw[0]) << 12) | (uint32_t(raw[1]) << 4) | (raw[2] >> 4);
  const uint32_t adcT = (uint32_t(raw[3]) << 12) | (uint32_t(raw[4]) << 4) | (raw[5] >> 4);
  const uint16_t adcH = (uint16_t(raw[6]) << 8) | raw[7];
  if (adcP == 0x80000 || adcT == 0x80000 || adcH == 0x8000) return reading;

  // Bosch double-precision compensation, datasheet appendix 8.1. A single
  // burst supplies all channels and the same t_fine compensates P and H.
  double v1 = (adcT / 16384.0 - t1_ / 1024.0) * t2_;
  double v2 = adcT / 131072.0 - t1_ / 8192.0;
  v2 = v2 * v2 * t3_;
  const double fine = v1 + v2;
  reading.temperatureC = fine / 5120.0;
  v1 = fine / 2.0 - 64000.0;
  v2 = v1 * v1 * p_[4] / 32768.0 + v1 * p_[3] * 2.0;
  v2 = v2 / 4.0 + p_[2] * 65536.0;
  v1 = (p_[1] * v1 * v1 / 524288.0 + p_[0] * v1) / 524288.0;
  v1 = (1.0 + v1 / 32768.0) * p1_;
  if (v1 == 0) return reading;
  double pressure = (1048576.0 - adcP - v2 / 4096.0) * 6250.0 / v1;
  v1 = p_[7] * pressure * pressure / 2147483648.0;
  v2 = pressure * p_[6] / 32768.0;
  reading.pressureHpa = (pressure + (v1 + v2 + p_[5]) / 16.0) / 100.0;
  double h = fine - 76800.0;
  h = (adcH - (h4_ * 64.0 + h5_ * h / 16384.0)) *
      (h2_ / 65536.0 * (1.0 + h6_ * h / 67108864.0 * (1.0 + h3_ * h / 67108864.0)));
  h *= 1.0 - h1_ * h / 524288.0;
  if (!isfinite(h)) return reading;
  reading.humidityPercent = constrain(h, 0.0, 100.0);
  reading.valid = isfinite(reading.temperatureC) && reading.temperatureC >= -40 &&
      reading.temperatureC <= 85 && isfinite(reading.pressureHpa) &&
      reading.pressureHpa >= 300 && reading.pressureHpa <= 1100;
  if (reading.valid) reading.capabilities = lil::protocol::kTemperature |
      lil::protocol::kHumidity | lil::protocol::kPressure;
  return reading;
}
}  // namespace sensor

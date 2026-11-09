#include <cassert>
#include <Arduino.h>
#include <Wire.h>
#include <cstring>
#include "bme280_driver.h"
#ifndef EXPECT_BME280_MEASUREMENT_READS
#define EXPECT_BME280_MEASUREMENT_READS 2
#endif
void testRadioTick() { ++testMillis; }
bool expired = false;
namespace sensor {
bool measurementBudgetExpired() { return expired; }
void measurementWaitUs(uint32_t us) { testMillis += (us + 999) / 1000; }
}
namespace {
// Immutable pre-optimization Bosch formula. Read the fixture's real trim
// bytes so comparison exercises the production driver's parsing and math.
sensor::EnvironmentalReading original(uint32_t adcT, uint32_t adcP, uint16_t adcH) {
  sensor::EnvironmentalReading reading{};
  reading.sensorType = lil::protocol::EnvironmentalSensorType::kBme280;
  if (adcT == 0x80000 || adcP == 0x80000 || adcH == 0x8000) return reading;
  const auto word = [](unsigned reg) { return uint16_t(Wire.registers[reg] | uint16_t(Wire.registers[reg + 1]) << 8); };
  const auto signed12 = [](uint16_t value) { return int16_t(value & 0x800 ? int32_t(value) - 4096 : value); };
  const uint16_t t1 = word(0x88), p1 = word(0x8E);
  const int16_t t2 = int16_t(word(0x8A)), t3 = int16_t(word(0x8C));
  int16_t p[8];
  for (size_t i = 0; i < 8; ++i) p[i] = int16_t(word(0x90 + i * 2));
  const uint8_t h1 = Wire.registers[0xA1], h3 = Wire.registers[0xE3];
  const int16_t h2 = int16_t(word(0xE1));
  const int16_t h4 = signed12(uint16_t(Wire.registers[0xE4]) << 4 | (Wire.registers[0xE5] & 15));
  const int16_t h5 = signed12(uint16_t(Wire.registers[0xE6]) << 4 | (Wire.registers[0xE5] >> 4));
  const int8_t h6 = int8_t(Wire.registers[0xE7]);
  double v1 = (adcT / 16384.0 - t1 / 1024.0) * t2;
  double v2 = adcT / 131072.0 - t1 / 8192.0;
  v2 = v2 * v2 * t3;
  const double fine = v1 + v2;
  reading.temperatureC = fine / 5120.0;
  v1 = fine / 2.0 - 64000.0;
  v2 = v1 * v1 * p[4] / 32768.0 + v1 * p[3] * 2.0;
  v2 = v2 / 4.0 + p[2] * 65536.0;
  v1 = (p[1] * v1 * v1 / 524288.0 + p[0] * v1) / 524288.0;
  v1 = (1.0 + v1 / 32768.0) * p1;
  if (v1 == 0) return reading;
  double pressure = (1048576.0 - adcP - v2 / 4096.0) * 6250.0 / v1;
  v1 = p[7] * pressure * pressure / 2147483648.0;
  v2 = pressure * p[6] / 32768.0;
  reading.pressureHpa = (pressure + (v1 + v2 + p[5]) / 16.0) / 100.0;
  double h = fine - 76800.0;
  h = (adcH - (h4 * 64.0 + h5 * h / 16384.0)) *
      (h2 / 65536.0 * (1.0 + h6 * h / 67108864.0 * (1.0 + h3 * h / 67108864.0)));
  h *= 1.0 - h1 * h / 524288.0;
  if (!isfinite(h)) return reading;
  reading.humidityPercent = constrain(h, 0.0, 100.0);
  reading.valid = isfinite(reading.temperatureC) && reading.temperatureC >= -40 &&
      reading.temperatureC <= 85 && isfinite(reading.pressureHpa) && reading.pressureHpa >= 300 && reading.pressureHpa <= 1100;
  if (reading.valid) reading.capabilities = lil::protocol::kTemperature | lil::protocol::kHumidity | lil::protocol::kPressure;
  return reading;
}
void compensationEquivalence() {
  sensor::Bme280Driver driver;
  Wire = BmeWire{};
  assert(driver.begin(0x76));
  size_t cases = 0;
  const auto check = [&](uint32_t t, uint32_t p, uint16_t h) {
    Wire.raw(t, p, h);
    const auto expected = original(t, p, h), actual = driver.read();
    assert(!memcmp(&expected.temperatureC, &actual.temperatureC, sizeof(float)));
    assert(!memcmp(&expected.pressureHpa, &actual.pressureHpa, sizeof(float)));
    assert(!memcmp(&expected.humidityPercent, &actual.humidityPercent, sizeof(float)));
    assert(expected.valid == actual.valid && expected.capabilities == actual.capabilities);
    ++cases;
  };
  for (uint32_t t = 0; t < (1U << 20); ++t) check(t, 415148, 35000);
  uint32_t random = 0x280C3U;
  const auto next = [&]() { random = random * 1664525U + 1013904223U; return random; };
  for (size_t calibration = 0; calibration < 32; ++calibration) {
    Wire = BmeWire{};
    Wire.word(0x88, calibration == 0 ? 1 : calibration == 1 ? 65534 : uint16_t(1 + next() % 65534));
    for (unsigned reg : {0x8AU, 0x8CU, 0x90U, 0x92U, 0x94U, 0x96U, 0x98U, 0x9AU, 0x9CU, 0x9EU, 0xE1U})
      Wire.word(reg, calibration < 2 ? (calibration ? 32767 : 32768) : uint16_t(next()));
    Wire.humidity(calibration == 0 ? -2048 : calibration == 1 ? 2047 : int16_t(next() % 4096) - 2048,
                  int16_t(next() % 4096) - 2048);
    assert(driver.begin(0x76));
    for (size_t i = 0; i < 512; ++i) check(next() & 0xFFFFF, next() & 0xFFFFF, uint16_t(next()));
  }
  printf("BME280 equivalence: %zu complete outputs match original float bits/validity, including all temperature ADC codes and trim boundaries\n", cases);
}
}
int main() {
  sensor::Bme280Driver driver;
  assert(!driver.read().valid);
  assert(driver.begin(0x76));
  assert(Wire.forcedWrites == 0); // Initialization must stay asleep.
  const unsigned previousReads = Wire.readRequests;
  auto r = driver.read();
  assert(r.valid && Wire.forcedWrites == 1 && Wire.dataBursts == 1);
  const unsigned measurementReads = Wire.readRequests - previousReads;
  printf("BME280 ready conversion: %u I2C read requests\n", measurementReads);
  assert(measurementReads == EXPECT_BME280_MEASUREMENT_READS);
  // Bosch reference T/P trimming and ADC example, double-precision results.
  assert(fabs(r.temperatureC - 25.0824779) < 0.0001);
  assert(fabs(r.pressureHpa - 1006.5326678) < 0.001);
  assert(fabs(r.humidityPercent - 78.4552267) < 0.0001);
  Wire = BmeWire{}; Wire.humidity(-30, -50); Wire.raw(519888,415148,1000);
  assert(driver.begin(0x76));
  assert(fabs(driver.read().humidityPercent - 17.3483688) < 0.0001);
  for (int reg : {0xD0,0xF3,0x88,0xE1,0xF2,0xF4,0xF5}) {
    Wire = BmeWire{}; Wire.failRead = reg; assert(!driver.begin(0x76));
  }
  for (int reg : {0xE0,0xF2,0xF4,0xF5}) {
    Wire = BmeWire{}; Wire.failWrite = reg; assert(!driver.begin(0x76));
  }
  Wire = BmeWire{}; Wire.ignoreWrite = 0xF2; assert(!driver.begin(0x76));
  Wire = BmeWire{}; Wire.inconsistentTrim = true; assert(!driver.begin(0x76));
  for (uint16_t trim : {0,65535}) {
    Wire = BmeWire{}; Wire.word(0x8E,trim); assert(!driver.begin(0x76));
  }
  Wire = BmeWire{}; Wire.stuck = true;
  auto start = millis(); assert(!driver.begin(0x76) && millis()-start <= 103);
  for (int reg : {0xF3,0xF4,0xF7}) {
    Wire = BmeWire{}; assert(driver.begin(0x76));
    Wire.failRead = reg; assert(!driver.read().valid);
  }
  Wire = BmeWire{}; assert(driver.begin(0x76));
  Wire.conversionDurationMs = 15;
  auto delayed = millis();
  assert(driver.read().valid && millis() - delayed == 15);
  Wire = BmeWire{}; assert(driver.begin(0x76));
  Wire.finishedControl = 0x20;
  assert(!driver.read().valid && Wire.dataBursts == 0);
  Wire = BmeWire{}; assert(driver.begin(0x76));
  Wire.failWrite = 0xF4; assert(!driver.read().valid);
  Wire.failWrite = -1; Wire.stuck = true; start = millis();
  assert(!driver.read().valid && millis()-start <= 60);
  Wire = BmeWire{}; assert(driver.begin(0x76));
  Wire.raw(0x80000,415148,35000); assert(!driver.read().valid);
  expired = true; assert(!driver.begin(0x76)); expired = false;
  Wire = BmeWire{}; Wire.present = 0x77; assert(driver.begin(0x77));
  assert(driver.read().valid);
  compensationEquivalence();
  puts("BME280: Bosch reference compensation, burst coherence, no init conversion, NVM/config checks, bus faults and deadlines passed");
}

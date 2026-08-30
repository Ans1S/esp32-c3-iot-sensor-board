#include "bme280_driver.h"

#include <Wire.h>

namespace sensor {

namespace {
constexpr uint8_t kAddresses[] = {0x76, 0x77};
}

bool Bme280Driver::begin() {
  initialized_ = false;
  for (const uint8_t address : kAddresses) {
    if (sensor_.begin(address, &Wire)) {
      initialized_ = true;
      break;
    }
  }
  if (!initialized_) {
    return false;
  }

  // Forced mode keeps the sensor asleep except for one short measurement.
  // x1 oversampling is the best energy/noise compromise for slow room data.
  sensor_.setSampling(Adafruit_BME280::MODE_FORCED,
                      Adafruit_BME280::SAMPLING_X1,
                      Adafruit_BME280::SAMPLING_X1,
                      Adafruit_BME280::SAMPLING_X1,
                      Adafruit_BME280::FILTER_OFF);
  return true;
}

EnvironmentalReading Bme280Driver::read() {
  EnvironmentalReading reading{};
  reading.sensorType = lil::protocol::EnvironmentalSensorType::kBme280;
  if (!initialized_ || !sensor_.takeForcedMeasurement()) {
    return reading;
  }

  reading.temperatureC = sensor_.readTemperature();
  reading.humidityPercent = sensor_.readHumidity();
  reading.pressureHpa = sensor_.readPressure() / 100.0F;
  reading.valid = isfinite(reading.temperatureC) &&
                  reading.temperatureC >= -40.0F &&
                  reading.temperatureC <= 85.0F &&
                  isfinite(reading.humidityPercent) &&
                  reading.humidityPercent >= 0.0F &&
                  reading.humidityPercent <= 100.0F &&
                  isfinite(reading.pressureHpa) &&
                  reading.pressureHpa >= 300.0F &&
                  reading.pressureHpa <= 1100.0F;
  if (reading.valid) {
    reading.capabilities = lil::protocol::kTemperature |
                           lil::protocol::kHumidity |
                           lil::protocol::kPressure;
  }
  return reading;
}

}  // namespace sensor

#include "adc_reader.h"

#include <algorithm>

#include "hardware_profile.h"

namespace sensor {

namespace {
constexpr size_t kSampleCount = 16;
}

void AdcReader::begin(PowerController& power) {
  power_ = &power;
  analogReadResolution(12);
  analogSetPinAttenuation(static_cast<uint8_t>(kHardware.adcPin), ADC_11db);
}

BatteryReading AdcReader::readBattery(float calibrationFactor) {
  BatteryReading reading{};
  power_->adcPower(true);
  delay(kHardware.adcSettleMs);

  uint16_t samples[kSampleCount];
  for (size_t i = 0; i < kSampleCount; ++i) {
    samples[i] = analogReadMilliVolts(static_cast<uint8_t>(kHardware.adcPin));
    delayMicroseconds(150);
  }
  power_->adcPower(false);

  std::sort(samples, samples + kSampleCount);
  uint32_t sum = 0;
  for (size_t i = 2; i < kSampleCount - 2; ++i) {
    sum += samples[i];
  }
  const float adcMillivolts =
      static_cast<float>(sum) / static_cast<float>(kSampleCount - 4);
  const uint32_t batteryMillivolts =
      lroundf(adcMillivolts * kHardware.batteryDividerScale *
              constrain(calibrationFactor, 0.7F, 1.3F));

  reading.valid = batteryMillivolts >= 2000 && batteryMillivolts <= 5000;
  reading.millivolts = static_cast<uint16_t>(
      batteryMillivolts > UINT16_MAX ? UINT16_MAX : batteryMillivolts);
  return reading;
}

}  // namespace sensor

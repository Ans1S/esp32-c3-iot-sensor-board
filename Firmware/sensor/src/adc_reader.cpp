#include "adc_reader.h"

#include "hardware_profile.h"
#include "low_power_wait.h"
#include "power_policy.h"
#include <string.h>

namespace sensor {

namespace {
constexpr size_t kSampleCount = 16;
}

void AdcReader::begin(PowerController& power) {
  power_ = &power;
  analogReadResolution(12);
  analogSetPinAttenuation(static_cast<uint8_t>(kHardware.adcPin), ADC_11db);
}

void AdcReader::startBatteryMeasurement() {
  if (power_ == nullptr || measurementPending_) {
    return;
  }
  power_->adcPower(true);
  settleStartedUs_ = micros();
  measurementPending_ = true;
}

BatteryReading AdcReader::finishBatteryMeasurement(float calibrationFactor) {
  BatteryReading reading{};
  if (power_ == nullptr) {
    return reading;
  }
  if (!measurementPending_) {
    startBatteryMeasurement();
  }

  const uint32_t requiredSettleUs =
      static_cast<uint32_t>(kHardware.adcSettleMs) * 1000UL;
  const uint32_t elapsedSettleUs = micros() - settleStartedUs_;
  if (elapsedSettleUs < requiredSettleUs) {
    const uint32_t remainingUs = requiredSettleUs - elapsedSettleUs;
    lowPowerSensorWaitUs(remainingUs);
  }

  uint32_t sum = 0;
  uint16_t lowest = UINT16_MAX, secondLowest = UINT16_MAX;
  uint16_t highest = 0, secondHighest = 0;
  for (size_t i = 0; i < kSampleCount; ++i) {
    const uint16_t sample = analogReadMilliVolts(static_cast<uint8_t>(kHardware.adcPin));
    sum += sample;
    if (sample < lowest) {
      secondLowest = lowest;
      lowest = sample;
    } else if (sample < secondLowest) {
      secondLowest = sample;
    }
    if (sample > highest) {
      secondHighest = highest;
      highest = sample;
    } else if (sample > secondHighest) {
      secondHighest = sample;
    }
    if (i + 1 < kSampleCount) delayMicroseconds(150);
  }
  power_->adcPower(false);
  measurementPending_ = false;

  // The same two readings from each end of a sorted window are discarded,
  // including repeated extrema, without sorting or buffering the samples.
  sum -= uint32_t(lowest) + secondLowest + highest + secondHighest;
  uint32_t factorBits;
  static_assert(sizeof(factorBits) == sizeof(calibrationFactor));
  memcpy(&factorBits, &calibrationFactor, sizeof(factorBits));
  const uint32_t gain = lil::power::calibrationQ20(factorBits);
  if (gain == 0) return reading;
  const uint32_t batteryMillivolts = lil::power::batteryMillivolts(
      sum, gain, kHardware.batteryDividerNumerator,
      kHardware.batteryDividerDenominator);

  reading.valid = batteryMillivolts >= 2000 && batteryMillivolts <= 5000;
  reading.millivolts = static_cast<uint16_t>(
      batteryMillivolts > UINT16_MAX ? UINT16_MAX : batteryMillivolts);
  return reading;
}

BatteryReading AdcReader::readBattery(float calibrationFactor) {
  startBatteryMeasurement();
  return finishBatteryMeasurement(calibrationFactor);
}

}  // namespace sensor

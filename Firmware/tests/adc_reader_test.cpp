#include <algorithm>
#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>

#include "adc_reader.h"
#include "low_power_wait.h"
#include "power_policy.h"

namespace {
std::array<uint16_t, 16> samples;
size_t sampleIndex;
uint32_t nowUs, startedUs, sampleDelayUs, settleUs;
bool enabled;
}

void analogReadResolution(int bits) { assert(bits == 12); }
void analogSetPinAttenuation(uint8_t pin, int attenuation) {
  assert(pin == sensor::kHardware.adcPin && attenuation == ADC_11db);
}
uint32_t analogReadMilliVolts(uint8_t pin) {
  assert(enabled && pin == sensor::kHardware.adcPin && sampleIndex < samples.size());
  assert(nowUs - startedUs == sensor::kHardware.adcSettleMs * 1000U + sampleIndex * 150U);
  return samples[sampleIndex++];
}
uint32_t micros() { return nowUs; }
void delayMicroseconds(uint32_t period) { sampleDelayUs += period; nowUs += period; }
void sensor::PowerController::adcPower(bool on) {
  assert(on != enabled);
  enabled = on;
  if (on) startedUs = nowUs;
}
void sensor::lowPowerSensorWaitUs(uint32_t period) { settleUs += period; nowUs += period; }

int main() {
  sensor::AdcReader reader;
  sensor::PowerController power;
  assert(!reader.readBattery(1).valid);
  reader.begin(power);
  uint32_t random = 0xb082d2U;
  for (size_t trial = 0; trial < 10000; ++trial) {
    for (auto& sample : samples) {
      random = random * 1664525U + 1013904223U;
      sample = trial == 0 ? 0 : trial == 1 ? UINT16_MAX : trial == 2 ? 2200 :
          trial < 100 ? 2100 + (random >> 30) : 1000 + (random >> 20);
    }
    auto sorted = samples;
    std::sort(sorted.begin(), sorted.end());
    uint32_t sum = 0;
    for (size_t i = 2; i < 14; ++i) sum += sorted[i];
    const float factor = trial % 3 == 0 ? 1.0F : trial % 3 == 1 ? 1.013F : 0.992F;
    uint32_t bits;
    memcpy(&bits, &factor, sizeof(bits));
    const auto expected = lil::power::batteryMillivolts(sum, lil::power::calibrationQ20(bits),
        sensor::kHardware.batteryDividerNumerator, sensor::kHardware.batteryDividerDenominator);
    sampleIndex = 0;
    nowUs = UINT32_MAX - 20;
    sampleDelayUs = settleUs = 0;
    const auto reading = reader.readBattery(factor);
    assert(reading.millivolts == std::min(expected, uint32_t(UINT16_MAX)));
    assert(reading.valid == (expected >= 2000 && expected <= 5000));
    assert(!enabled && sampleIndex == 16 && settleUs == sensor::kHardware.adcSettleMs * 1000U);
    assert(sampleDelayUs == 2250);
  }
  printf("ADC V%d: 10000 matching windows, 16 reads, %u us between reads in total\n",
         PCB_VERSION, sampleDelayUs);
}

#pragma once
#include <stdint.h>

namespace lil::timing {
constexpr uint32_t kI2cHz = 400000;
constexpr uint16_t kImuHz = 104;
constexpr uint16_t kImuReportMs = 100;
constexpr uint16_t kImuShortSessionMs = 50;
constexpr uint32_t kShortSessionDurationMs = 5UL * 60UL * 1000UL;
constexpr uint32_t imuRecordingIntervalMs(uint32_t elapsedMs) {
  return elapsedMs < kShortSessionDurationMs ? kImuShortSessionMs : kImuReportMs;
}
constexpr uint16_t kTemperatureCycleMs = 1000;
constexpr uint16_t kTemperatureAverages = 8;
constexpr uint16_t kTemperatureActiveMs = 124; // 8 * 15.5 ms typical.
constexpr uint16_t kTemperatureActiveMaxMs = 140; // 8 * 17.5 ms.
constexpr uint16_t kOpticalAdcHz = 100;
constexpr uint16_t kOpticalAverages = 4;
constexpr uint16_t kOpticalFifoHz = kOpticalAdcHz / kOpticalAverages;
constexpr uint16_t kOpticalSampleMs = 1000 / kOpticalFifoHz;
constexpr uint16_t kOpticalReportMs = 200;
constexpr uint16_t kPulseWindowMs = 8000;
constexpr uint16_t kPulseUpdateMs = 1000;
constexpr uint16_t kPulseSamples = kPulseWindowMs / kOpticalSampleMs;
constexpr uint16_t kFifoCapacityMs = 32 * kOpticalSampleMs;
// Wire time lower bounds: address+register+address+data, each with ACK.
constexpr uint32_t i2cReadUs(uint16_t bytes) {
  return (uint32_t(bytes + 3) * 9 * 1000000 + kI2cHz - 1) / kI2cHz;
}
static_assert(kTemperatureActiveMaxMs < kTemperatureCycleMs);
static_assert(kOpticalReportMs < kFifoCapacityMs);
static_assert(i2cReadUs(7) * kImuHz * 2 < 100000); // <10% data bus duty.
static_assert(i2cReadUs(6) * kOpticalFifoHz < 10000); // <1% data bus duty.
}

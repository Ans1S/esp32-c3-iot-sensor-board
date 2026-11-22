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
  return static_cast<uint32_t>(
      (uint64_t(bytes + 3) * 9ULL * 1000000ULL + kI2cHz - 1) / kI2cHz);
}
static_assert(kTemperatureActiveMaxMs < kTemperatureCycleMs);
static_assert(kOpticalReportMs < kFifoCapacityMs);
static_assert(i2cReadUs(7) * kImuHz * 2 < 100000); // <10% data bus duty.
static_assert(i2cReadUs(6) * kOpticalFifoHz < 10000); // <1% data bus duty.

// Exact unsigned 64-bit microsecond-to-millisecond conversion (us / 1000)
// using 32x32->64-bit hardware multiplies so 32-bit MCUs avoid __udivdi3.
constexpr uint64_t usToMs(uint64_t us) {
  const uint64_t q = us >> 3;
  constexpr uint64_t kMagic = 0x20C49BA5E353F7CFULL;  // ceil(2^68 / 125)
  const uint64_t a0 = static_cast<uint32_t>(q);
  const uint64_t a1 = q >> 32;
  constexpr uint64_t b0 = static_cast<uint32_t>(kMagic);
  constexpr uint64_t b1 = kMagic >> 32;
  const uint64_t p00 = a0 * b0;
  const uint64_t p01 = a0 * b1;
  const uint64_t p10 = a1 * b0;
  const uint64_t p11 = a1 * b1;
  const uint64_t mid = (p00 >> 32) + static_cast<uint32_t>(p01) + static_cast<uint32_t>(p10);
  const uint64_t hi = p11 + (p01 >> 32) + (p10 >> 32) + (mid >> 32);
  return hi >> 4;
}

constexpr uint64_t signedUsToMs(int64_t us) {
  return us > 0 ? usToMs(static_cast<uint64_t>(us)) : 0ULL;
}

// Exact integer square roots (floor(sqrt(n))) using hardware clz + Newton
// iterations, avoiding software floating-point sqrtf/sqrt on FPU-less RV32IMC.
constexpr uint32_t isqrt32(uint32_t n) {
  if (n < 2U) return n;
  uint32_t x = 1U << ((32 - __builtin_clz(n)) >> 1);
  // Three Newton iterations converge for all uint32_t inputs.
  x = (x + n / x) >> 1;
  x = (x + n / x) >> 1;
  x = (x + n / x) >> 1;
  if (x > 65535U) x = 65535U;
  while (x * x > n) --x;
  while ((x + 1U) <= 65535U && (x + 1U) * (x + 1U) <= n) ++x;
  return x;
}

inline uint32_t isqrt64(uint64_t n) {
  if (n < 2ULL) return static_cast<uint32_t>(n);
  if (n <= UINT32_MAX) return isqrt32(static_cast<uint32_t>(n));
  uint64_t bit = 1ULL << ((63 - __builtin_clzll(n)) & ~1);
  uint64_t res = 0;
  while (bit != 0ULL) {
    if (n >= res + bit) {
      n -= res + bit;
      res = (res >> 1) + bit;
    } else {
      res >>= 1;
    }
    bit >>= 2;
  }
  return static_cast<uint32_t>(res);
}
}

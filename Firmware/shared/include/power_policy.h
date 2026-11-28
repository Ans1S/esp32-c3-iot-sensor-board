#pragma once

#include <stdint.h>

namespace lil::power {
// An unreadable battery must not authorize powering an external sensor. Once
// paused, require a fresh reading above the recovery threshold (hysteresis).
constexpr bool batteryProtectionRequired(bool paused, bool valid, uint16_t mv,
                                         uint16_t pauseMv, uint16_t marginMv) {
  return pauseMv != 0 &&
      (!valid || mv < uint32_t(pauseMv) + (paused ? marginMv : 0));
}

constexpr uint32_t kInitialDiscoveryMs = 600000;

constexpr uint32_t discoverySleepForAgeMs(uint64_t ageMs, uint32_t maximumSeconds = 300) {
  if (ageMs < kInitialDiscoveryMs) return maximumSeconds < 10 ? maximumSeconds : 10;
  uint32_t seconds = maximumSeconds < 300 ? maximumSeconds : 300;
  // For the production 300-second ceiling the compiler eliminates this loop.
  constexpr uint32_t hourMs = 3600000;
  for (uint64_t next = kInitialDiscoveryMs + hourMs; seconds < maximumSeconds && ageMs >= next; next += hourMs) {
    seconds = seconds > maximumSeconds / 2 ? maximumSeconds : seconds * 2;
  }
  return seconds;
}

constexpr uint32_t discoverySleepSeconds(uint64_t ageSeconds, uint32_t maximumSeconds = 300) {
  return discoverySleepForAgeMs(ageSeconds > UINT64_MAX / 1000 ? UINT64_MAX : ageSeconds * 1000, maximumSeconds);
}

constexpr bool reportDue(bool attempted, uint64_t nowMs, uint64_t lastMs,
                         uint32_t intervalSeconds) {
  return !attempted || nowMs < lastMs ||
         nowMs - lastMs >= uint64_t(intervalSeconds) * 1000;
}

constexpr uint32_t scheduledSleepSeconds(bool currentSchedule, uint64_t nowMs,
                                         uint64_t nextMs, uint32_t fallback) {
  if (!currentSchedule || nextMs <= nowMs) return fallback;
  const uint64_t seconds = (nextMs - nowMs + 999) / 1000;
  return seconds < fallback ? static_cast<uint32_t>(seconds) : fallback;
}

// Positive IEEE-754 calibration factor, quantized once to Q20. Restricting the
// accepted exponent also rejects NaN/Infinity before any integer conversion.
inline uint32_t calibrationQ20(uint32_t bits) {
  const uint32_t exponent = (bits >> 23) & 255;
  if ((bits >> 31) != 0 || exponent < 126 || exponent > 127) return 0;
  const uint32_t mantissa = (bits & 0x7fffff) | 0x800000;
  const unsigned shift = 130 - exponent;
  const uint32_t fixed = (mantissa + (1U << (shift - 1))) >> shift;
  return fixed >= 734003 && fixed <= 1363149 ? fixed : 0;
}

constexpr uint32_t batteryMillivolts(uint32_t sum, uint32_t gainQ20,
                                    uint32_t dividerNumerator,
                                    uint32_t dividerDenominator) {
  const uint64_t denominator = uint64_t(12) * dividerDenominator * (1U << 20);
  return static_cast<uint32_t>((uint64_t(sum) * gainQ20 * dividerNumerator +
                                denominator / 2) / denominator);
}
}  // namespace lil::power

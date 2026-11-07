#pragma once

#include <stddef.h>
#include <stdint.h>
#include "sensor_timing.h"

namespace sensor::pulse {
constexpr size_t kMeanRadius = 12;
constexpr size_t kMeanSamples = 2 * kMeanRadius + 1;
constexpr size_t kAnalyzedSamples = lil::timing::kPulseSamples - 2 * kMeanRadius;
static_assert(kMeanSamples * uint64_t(0x3FFFF) <= UINT32_MAX);

// Samples are the MAX30102's unsigned 18-bit FIFO values. Every 25-value sum
// is exact in uint32_t and double, so sliding the sum preserves the original
// centered mean, float residuals and energy accumulation order bit for bit.
// The sample accessor also permits read counting in the host regression.
template <typename Samples>
double detrend(const Samples& samples, size_t head, float* ac) {
  constexpr size_t count = lil::timing::kPulseSamples;
  size_t first = head;
  size_t next = head;
  uint32_t sum = 0;
  for (size_t i = 0; i < kMeanSamples; ++i) {
    sum += samples[next];
    if (++next == count) next = 0;
  }
  size_t center = (head + kMeanRadius) % count;
  double energy = 0;
  for (size_t i = 0; i < kAnalyzedSamples; ++i) {
    ac[i] = samples[center] - static_cast<double>(sum) / kMeanSamples;
    energy += ac[i] * ac[i];
    if (i + 1 < kAnalyzedSamples) {
      sum -= samples[first];
      sum += samples[next];
      if (++first == count) first = 0;
      if (++next == count) next = 0;
      if (++center == count) center = 0;
    }
  }
  return energy;
}
}  // namespace sensor::pulse

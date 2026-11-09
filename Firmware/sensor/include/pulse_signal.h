#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include "sensor_timing.h"

namespace sensor::pulse {
constexpr size_t kMeanRadius = 12;
constexpr size_t kMeanSamples = 2 * kMeanRadius + 1;
constexpr size_t kAnalyzedSamples = lil::timing::kPulseSamples - 2 * kMeanRadius;
constexpr int kMinimumLag = 6, kMaximumLag = 51;
static_assert(kAnalyzedSamples > kMaximumLag);
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

inline void correlate(const float* ac, float* correlations) {
  constexpr int count = kAnalyzedSamples;
  double xx = 0, yy = 0;
  for (int i = 0; i < count - kMaximumLag; ++i) {
    xx += ac[i] * ac[i];
    yy += ac[i + kMaximumLag] * ac[i + kMaximumLag];
  }
  // A shorter lag adds one square to the end of the first range and one
  // to the beginning of the second. All additions are nonnegative, avoiding
  // cancellation on low-energy windows. The cross-product order is unchanged.
  for (int lag = kMaximumLag; lag >= kMinimumLag; --lag) {
    double xy = 0;
    for (int i = 0; i < count - lag; ++i) xy += ac[i] * ac[i + lag];
    correlations[lag] = xx > 0 && yy > 0 ? xy / sqrt(xx * yy) : 0;
    if (lag != kMinimumLag) {
      xx += ac[count - lag] * ac[count - lag];
      yy += ac[lag - 1] * ac[lag - 1];
    }
  }
}
}  // namespace sensor::pulse

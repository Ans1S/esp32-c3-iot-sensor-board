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

// Exact scaled-integer detrending and autocorrelation for FPU-less RV32IMC.
// ac25[i] = 25 * samples[center] - sum(25 samples) is exact in int32_t
// (|ac25| <= 25 * 262143 < 6.6e6), and every sum of up to 176 squared/cross
// products (<= 7.6e15) is exact in int64_t using hardware RV32IM mul/mulh.
template <typename Samples>
int64_t detrendInt25Energy(const Samples& samples, size_t head, int32_t* ac25) {
  constexpr size_t count = lil::timing::kPulseSamples;
  size_t first = head;
  size_t next = head;
  uint32_t sum = 0;
  for (size_t i = 0; i < kMeanSamples; ++i) {
    sum += samples[next];
    if (++next == count) next = 0;
  }
  size_t center = (head + kMeanRadius) % count;
  int64_t energy625 = 0;
  for (size_t i = 0; i < kAnalyzedSamples; ++i) {
    const int32_t residual =
        static_cast<int32_t>(samples[center] * kMeanSamples) -
        static_cast<int32_t>(sum);
    ac25[i] = residual;
    energy625 += static_cast<int64_t>(residual) * residual;
    if (i + 1 < kAnalyzedSamples) {
      sum -= samples[first];
      sum += samples[next];
      if (++first == count) first = 0;
      if (++next == count) next = 0;
      if (++center == count) center = 0;
    }
  }
  return energy625;
}

template <typename Samples>
double detrendInt25(const Samples& samples, size_t head, int32_t* ac25) {
  return static_cast<double>(detrendInt25Energy(samples, head, ac25)) /
         static_cast<double>(kMeanSamples * kMeanSamples);
}

#if defined(__GNUC__) && !defined(__clang__)
[[gnu::optimize("O2")]]
#endif
inline void correlateInt25(const int32_t* ac25, float* correlations) {
  constexpr int count = kAnalyzedSamples;
  int64_t xx = 0, yy = 0;
  for (int i = 0; i < count - kMaximumLag; ++i) {
    const int32_t a = ac25[i];
    const int32_t b = ac25[i + kMaximumLag];
    xx += static_cast<int64_t>(a) * a;
    yy += static_cast<int64_t>(b) * b;
  }
  for (int lag = kMaximumLag; lag >= kMinimumLag; --lag) {
    int64_t xy = 0;
    const int32_t* p = ac25;
    const int32_t* q = ac25 + lag;
    const int32_t* end = ac25 + (count - lag);
    while (p < end) {
      xy += static_cast<int64_t>(*p++) * (*q++);
    }
    if (xx > 0 && yy > 0) {
      correlations[lag] = static_cast<float>(
          static_cast<double>(xy) /
          sqrt(static_cast<double>(xx) * static_cast<double>(yy)));
    } else {
      correlations[lag] = 0.0F;
    }
    if (lag != kMinimumLag) {
      const int32_t a = ac25[count - lag];
      const int32_t b = ac25[lag - 1];
      xx += static_cast<int64_t>(a) * a;
      yy += static_cast<int64_t>(b) * b;
    }
  }
}
}  // namespace sensor::pulse

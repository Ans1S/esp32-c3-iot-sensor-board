#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif
#include "pulse_signal.h"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {
constexpr size_t samples = lil::timing::kPulseSamples;
constexpr size_t analyzed = sensor::pulse::kAnalyzedSamples;
struct CountedSamples {
  uint32_t values[samples]{};
  mutable size_t reads = 0;
  uint32_t operator[](size_t index) const { ++reads; return values[index]; }
};

// Numerical reference: the production centered-mean loop at ff7afa3.
template <typename Input>
double reference(const Input& signal, size_t head, float* ac) {
  double energy = 0;
  for (int i = 12; i < 188; ++i) {
    double mean = 0;
    for (int j = -12; j <= 12; ++j) mean += signal[(head + i + j) % 200];
    ac[i - 12] = signal[(head + i) % 200] - mean / 25;
    energy += ac[i - 12] * ac[i - 12];
  }
  return energy;
}
[[gnu::noinline]] double original(const uint32_t* signal, size_t head, float* ac) {
  return reference(signal, head, ac);
}
[[gnu::noinline]] double optimized(const uint32_t* signal, size_t head, float* ac) {
  return sensor::pulse::detrend(signal, head, ac);
}
void verify() {
  CountedSamples input;
  uint32_t random = 0x743921;
  size_t cases = 0, beforeReads = 0, afterReads = 0;
  for (size_t pattern = 0; pattern < 96; ++pattern) {
    for (size_t i = 0; i < samples; ++i) {
      random = random * 1664525U + 1013904223U;
      input.values[i] = pattern == 0 ? 0 : pattern == 1 ? 0x3FFFF :
          pattern == 2 ? (i & 1 ? 0x3FFFF : 0) :
          pattern == 3 ? i * 1000 : pattern == 4 ? (i == 199 ? 0x3FFFF : 0) :
          pattern < 14 ? static_cast<uint32_t>(100000 + 4000 *
              sin(i * 2 * 3.141592653589793 * (35 + (pattern - 5) * 20) / 1500)) :
          random & 0x3FFFF;
    }
    for (size_t head = 0; head < samples; ++head) {
      float expected[analyzed], actual[analyzed];
      input.reads = 0;
      const double before = reference(input, head, expected);
      beforeReads = input.reads;
      input.reads = 0;
      const double after = sensor::pulse::detrend(input, head, actual);
      afterReads = input.reads;
      assert(!memcmp(expected, actual, sizeof(actual)));
      assert(!memcmp(&before, &after, sizeof(before)));
      assert(beforeReads == 4576 && afterReads == 551);
      ++cases;
    }
  }
  printf("Pulse detrending: %zu bit-identical windows, %zu -> %zu sample reads per window\n",
         cases, beforeReads, afterReads);
}
void benchmark() {
  constexpr size_t runs = 9, iterations = 20000;
  double timings[2][runs]{}, sums[2][runs]{};
  for (size_t run = 0; run < runs; ++run) {
    for (size_t order = 0; order < 2; ++order) {
      const size_t version = (run + order) % 2;
      uint32_t signal[samples];
      for (size_t i = 0; i < samples; ++i) signal[i] = 100000 + i * 101;
      float ac[analyzed];
      double checksum = 0;
      const auto start = std::chrono::steady_clock::now();
      for (size_t i = 0; i < iterations; ++i) {
        signal[i % samples] ^= 1U;
        checksum += version ? optimized(signal, i % samples, ac) : original(signal, i % samples, ac);
      }
      timings[version][run] = std::chrono::duration<double, std::micro>(
          std::chrono::steady_clock::now() - start).count() / iterations;
      sums[version][run] = checksum;
    }
    assert(sums[0][run] == sums[1][run]);
  }
  for (size_t version = 0; version < 2; ++version) {
    std::sort(timings[version], timings[version] + runs);
    printf("%s host detrending: runs=%zu iterations=%zu median=%.6f us/window range=%.6f..%.6f us/window checksum=%.17g\n",
           version ? "After" : "Before", runs, iterations, timings[version][runs / 2],
           timings[version][0], timings[version][runs - 1], sums[version][0]);
  }
}
}  // namespace
int main(int argc, char** argv) {
  verify();
  if (argc == 2 && !strcmp(argv[1], "--benchmark")) benchmark();
}

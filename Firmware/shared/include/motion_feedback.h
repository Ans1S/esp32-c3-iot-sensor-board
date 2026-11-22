#pragma once
#include <math.h>
#include "lil_protocol.h"

namespace lil {
// Heuristic for a fixed, body-mounted sensor. Process every 104 Hz acceleration
// sample, never the downsampled chart means. Three rhythmic peaks confirm a bout.
// Fixed-point Q16 math avoids software floating-point emulation at 104 Hz on RV32IMC.
class MotionFeedbackTracker {
 public:
  void restartFilter() {
    gravityQ16_ = kOneQ16;
    filteredQ16_ = 0;
    envelopeQ16_ = 0;
    armed_ = true;
    sinceLastPeak_ = 255;
    lastInterval_ = 0;
    streak_ = 0;
    confirmed_ = false;
  }
  void observeQ16(int32_t magnitudeQ16) {
    if (sinceLastPeak_ < 65535) ++sinceLastPeak_;
    // 0.01 ~= 655 / 65536, 0.25 = >> 2, 0.05 ~= 3277 / 65536.
    gravityQ16_ += static_cast<int32_t>(
        (int64_t(magnitudeQ16 - gravityQ16_) * 655) >> 16);
    const int32_t highPass = magnitudeQ16 - gravityQ16_;
    filteredQ16_ += (highPass - filteredQ16_) >> 2;
    const int32_t absFiltered = filteredQ16_ < 0 ? -filteredQ16_ : filteredQ16_;
    envelopeQ16_ += static_cast<int32_t>(
        (int64_t(absFiltered - envelopeQ16_) * 3277) >> 16);
    if (envelopeQ16_ > kEnvelopeThreshQ16 &&
        activeTicks_ < uint32_t(UINT16_MAX) * timing::kImuHz) {
      ++activeTicks_;
    }
    if (filteredQ16_ < kRearmThreshQ16) armed_ = true;
    if (!armed_ || filteredQ16_ < kPeakThreshQ16 || sinceLastPeak_ < 26) return;
    armed_ = false;
    const uint32_t interval = sinceLastPeak_;
    sinceLastPeak_ = 0;
    if (!streak_ || interval > 156 || (lastInterval_ && streak_ > 1 &&
        (interval * 2 < lastInterval_ || interval > lastInterval_ * 2))) streak_ = 1;
    else if (streak_ < 3) ++streak_;
    lastInterval_ = interval;
    if (streak_ == 3) {
      const uint16_t add = confirmed_ ? 1 : 3;
      steps_ = steps_ > UINT16_MAX - add ? UINT16_MAX : steps_ + add;
      confirmed_ = true;
    } else confirmed_ = false;
  }
  void observe(float magnitude) {
    if (!isfinite(magnitude)) return;
    const float clamped = magnitude < -16.0F ? -16.0F : (magnitude > 16.0F ? 16.0F : magnitude);
    observeQ16(static_cast<int32_t>(clamped * 65536.0F));
  }
  protocol::MotionFeedback value() const {
    return {steps_, static_cast<uint16_t>(activeTicks_ / timing::kImuHz)};
  }
 private:
  static constexpr int32_t kOneQ16 = 65536;
  static constexpr int32_t kEnvelopeThreshQ16 = 3277;  // 0.05 g
  static constexpr int32_t kRearmThreshQ16 = 1966;     // 0.03 g
  static constexpr int32_t kPeakThreshQ16 = 7864;      // 0.12 g
  int32_t gravityQ16_ = kOneQ16, filteredQ16_ = 0, envelopeQ16_ = 0;
  uint32_t activeTicks_ = 0, lastInterval_ = 0;
  uint16_t sinceLastPeak_ = 255;
  uint16_t steps_ = 0;
  uint8_t streak_ = 0;
  bool armed_ = true, confirmed_ = false;
};
}

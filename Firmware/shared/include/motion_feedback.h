#pragma once
#include <math.h>
#include "lil_protocol.h"

namespace lil {
// Heuristic for a fixed, body-mounted sensor. Process every 104 Hz acceleration
// sample, never the downsampled chart means. Three rhythmic peaks confirm a bout.
class MotionFeedbackTracker {
 public:
  void restartFilter() { gravity_ = 1; filtered_ = envelope_ = 0; armed_ = true; lastPeak_ = ticks_; lastInterval_ = 0; streak_ = 0; confirmed_ = false; }
  void observe(float magnitude) {
    ++ticks_;
    gravity_ += .01F * (magnitude - gravity_);
    filtered_ += .25F * (magnitude - gravity_ - filtered_);
    envelope_ += .05F * (fabsf(filtered_) - envelope_);
    if (envelope_ > .05F && activeTicks_ < uint32_t(UINT16_MAX) * timing::kImuHz) ++activeTicks_;
    if (filtered_ < .03F) armed_ = true;
    if (!armed_ || filtered_ < .12F || ticks_ - lastPeak_ < 26) return;
    armed_ = false;
    const uint32_t interval = ticks_ - lastPeak_;
    lastPeak_ = ticks_;
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
  protocol::MotionFeedback value() const {
    return {steps_, static_cast<uint16_t>(activeTicks_ / timing::kImuHz)};
  }
 private:
  float gravity_ = 1, filtered_ = 0, envelope_ = 0;
  uint32_t ticks_ = 0, activeTicks_ = 0, lastPeak_ = 0, lastInterval_ = 0;
  uint16_t steps_ = 0;
  uint8_t streak_ = 0;
  bool armed_ = true, confirmed_ = false;
};
}

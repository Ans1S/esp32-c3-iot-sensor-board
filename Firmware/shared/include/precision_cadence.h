#pragma once
#include "lil_protocol.h"

namespace lil::timing {
// An idle retry has no deadline. Signed comparisons with a long-expired
// deadline become negative after 2^31 ms and can stall work for weeks.
class RetryBackoff {
 public:
  void clear() { waiting_ = false; }
  void wait(uint32_t now, uint32_t durationMs) {
    startedMs_ = now;
    durationMs_ = durationMs;
    waiting_ = durationMs != 0;
  }
  bool ready(uint32_t now) {
    if (waiting_ && uint32_t(now - startedMs_) >= durationMs_) clear();
    return !waiting_;
  }
 private:
  uint32_t startedMs_ = 0, durationMs_ = 0;
  bool waiting_ = false;
};

constexpr bool supportsNormalPrecisionMeasurements(
    protocol::EnvironmentalSensorType type) {
  return type == protocol::EnvironmentalSensorType::kLsm6dsox ||
      type == protocol::EnvironmentalSensorType::kTmp117;
}

// The configured interval schedules independent fresh acquisition windows.
// Recording uses its own faster profile and never inherits this interval.
constexpr uint32_t normalPrecisionIntervalMs(uint32_t seconds) {
  return (seconds < 1 ? 1 : seconds > 86400 ? 86400 : seconds) * 1000UL;
}

class PrecisionCadence {
 public:
  void reset(uint32_t now) { next_ = now; }
  bool due(uint32_t now) const { return static_cast<int32_t>(now - next_) >= 0; }
  void started(uint32_t now, uint32_t intervalMs) { next_ = now + intervalMs; }
 private:
  uint32_t next_ = 0;
};
}  // namespace lil::timing

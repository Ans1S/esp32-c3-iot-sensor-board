#pragma once
#include <stdint.h>

namespace station {
// Use the same 64-bit monotonic clock as authenticated sessions. An expired
// login block must not become a future deadline after weeks without a login.
class LoginThrottle {
 public:
  static constexpr uint8_t kMaxAttempts = 5;
  static constexpr uint64_t kBlockMs = 30000;
  static constexpr uint64_t kAttemptWindowMs = 5ULL * 60ULL * 1000ULL;
  bool blocked(uint64_t now, uint32_t& retryAfterSeconds) {
    if (blockedUntilMs_ > now) {
      retryAfterSeconds = static_cast<uint32_t>((blockedUntilMs_ - now + 999) / 1000);
      return true;
    }
    if (blockedUntilMs_ || now - lastAttemptMs_ > kAttemptWindowMs) {
      blockedUntilMs_ = 0;
      failures_ = 0;
    }
    retryAfterSeconds = 0;
    return false;
  }
  bool recordFailure(uint64_t now, uint32_t& retryAfterSeconds) {
    if (now - lastAttemptMs_ > kAttemptWindowMs) failures_ = 0;
    lastAttemptMs_ = now;
    if (++failures_ < kMaxAttempts) {
      retryAfterSeconds = 0;
      return false;
    }
    failures_ = 0;
    blockedUntilMs_ = now + kBlockMs;
    retryAfterSeconds = static_cast<uint32_t>(kBlockMs / 1000);
    return true;
  }
 private:
  uint64_t lastAttemptMs_ = 0, blockedUntilMs_ = 0;
  uint8_t failures_ = 0;
};
}  // namespace station

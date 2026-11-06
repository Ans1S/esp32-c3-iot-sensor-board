#pragma once
#include <atomic>
#include <stdint.h>

namespace station {
// The gateway task owns startedMs_; the Wi-Fi callback only completes a send.
class RadioSendGuard {
 public:
  void start(uint32_t now) {
    startedMs_ = now;
    pending_.store(true, std::memory_order_release);
  }
  void complete() { pending_.store(false, std::memory_order_release); }
  bool pending() const { return pending_.load(std::memory_order_acquire); }
  bool stalled(uint32_t now, uint32_t timeoutMs) const {
    return pending() && uint32_t(now - startedMs_) >= timeoutMs;
  }
 private:
  uint32_t startedMs_ = 0;
  std::atomic<bool> pending_{false};
};
}

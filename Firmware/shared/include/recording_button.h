#pragma once
#include <stdint.h>

namespace lil::recording {
class Button {
 public:
  void begin(bool pressed, uint32_t now) {
    raw_ = stable_ = pressed; armed_ = !pressed; changedMs_ = now;
  }
  bool poll(bool pressed, uint32_t now) {
    if (pressed != raw_) { raw_ = pressed; changedMs_ = now; }
    if (stable_ == raw_ || uint32_t(now - changedMs_) < 35) return false;
    stable_ = raw_;
    if (!stable_) { armed_ = true; return false; }
    if (!armed_) return false;
    armed_ = false; return true;
  }
 private:
  bool raw_ = false, stable_ = false, armed_ = false;
  uint32_t changedMs_ = 0;
};
}  // namespace lil::recording

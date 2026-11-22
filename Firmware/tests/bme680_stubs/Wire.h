#pragma once
#include <cstddef>
#include <cstdint>
class TwoWire {
 public:
  bool repeatedStart = false;
  void (*afterRegisterSelect)() = nullptr;
  void beginTransmission(uint8_t) {}
  void write(uint8_t) {}
  int endTransmission(bool stop = true) {
    repeatedStart = !stop;
    if (!stop && afterRegisterSelect) afterRegisterSelect();
    return 0;
  }
  size_t requestFrom(uint8_t, size_t count) { repeatedStart = false; return count; }
  int available() { return 1; }
  int read() { return 0; }
};
inline TwoWire Wire;

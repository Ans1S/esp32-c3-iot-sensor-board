#pragma once
#include <cstddef>
#include <cstdint>
class TwoWire {
 public:
  void beginTransmission(uint8_t) {}
  void write(uint8_t) {}
  int endTransmission(bool = true) { return 0; }
  size_t requestFrom(uint8_t, size_t count) { return count; }
  int available() { return 1; }
  int read() { return 0; }
};
inline TwoWire Wire;

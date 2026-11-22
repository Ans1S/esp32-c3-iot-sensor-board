#pragma once
#include <stddef.h>
#include <stdint.h>

struct TwoWire {
  bool begin(int = -1, int = -1, uint32_t = 100000);
  void end();
  void setClock(uint32_t);
  void setTimeOut(uint16_t);
  void beginTransmission(uint8_t);
  size_t write(uint8_t);
  uint8_t endTransmission(bool = true);
  uint8_t requestFrom(uint8_t, uint8_t);
  size_t requestFrom(uint8_t, size_t);
  int available();
  int read();
};

inline TwoWire Wire;

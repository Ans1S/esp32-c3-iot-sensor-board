#pragma once
#include <array>
#include <deque>
#include <vector>
#include <cstdint>
struct PrecisionWire {
  uint8_t address = 0, reg = 0, present = 0x48;
  bool fail = false, shortRead = false, stuckReset = false;
  uint8_t partialFifoBytes = 0;
  uint16_t words[256]{};
  uint8_t registers[256]{};
  std::vector<uint8_t> tx;
  std::deque<uint8_t> rx;
  std::deque<std::array<uint8_t,6>> fifo;
  PrecisionWire() { words[15] = 0x117; registers[255] = 0x15; }
  void beginTransmission(uint8_t a) { address = a; tx.clear(); }
  void write(uint8_t b) { tx.push_back(b); }
  int endTransmission(bool = true) {
    if (fail || address != present) return 2;
    reg = tx[0];
    if (tx.size() == 3) words[reg] = (tx[1] << 8) | tx[2];
    if (tx.size() == 2) {
      registers[reg] = reg == 9 && tx[1] == 0x40 && !stuckReset ? 0 : tx[1];
      if (reg == 6) fifo.clear();
    }
    return 0;
  }
  int requestFrom(uint8_t, uint8_t n) {
    rx.clear(); if (shortRead) return 0;
    if (present >= 0x48 && present <= 0x4B) {
      rx.push_back(words[reg] >> 8); rx.push_back(words[reg] & 255);
      if (reg == 1 || reg == 0) words[1] &= ~0x2000;
    } else if (reg == 4 && n == 3) {
      rx.push_back(fifo.size() & 31); rx.push_back(registers[5]); rx.push_back(0);
    } else if (reg == 7 && !fifo.empty()) {
      const auto& frame = fifo.front();
      for (uint8_t i = 0; i < (partialFifoBytes ? partialFifoBytes : 6); ++i)
        rx.push_back(frame[i]);
      fifo.pop_front();
    } else {
      for (int i=0;i<n;++i) rx.push_back(registers[reg+i]);
      if (reg == 0) registers[0] = 0;
    }
    return rx.size();
  }
  int read() { int b=rx.front(); rx.pop_front(); return b; }
  void sample(uint32_t value) {
    std::array<uint8_t,6> b{};
    for (int i=0;i<2;++i) { b[i*3]=value>>16; b[i*3+1]=value>>8; b[i*3+2]=value; }
    fifo.push_back(b);
    if (fifo.size() == 32) registers[0] |= 0x80;
  }
};
inline PrecisionWire Wire;

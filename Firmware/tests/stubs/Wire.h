#pragma once
#include <array>
#include <deque>
#include <vector>
#include <cstdint>
struct WireStub {
  uint8_t registers[256]{};
  uint8_t address = 0, reg = 0;
  bool fail = false, stuckReset = false, shortRead = false, overrun = false;
  std::vector<uint8_t> tx;
  std::deque<uint8_t> rx;
  std::deque<std::array<uint8_t, 7>> fifo;
  void beginTransmission(uint8_t a) { address = a; tx.clear(); }
  void write(uint8_t value) { tx.push_back(value); }
  uint8_t endTransmission(bool = true) {
    if (fail || address != 0x6A) return 2;
    reg = tx[0];
    if (tx.size() == 2) registers[reg] = reg == 0x12 && tx[1] == 1 && !stuckReset ? 0 : tx[1];
    return 0;
  }
  uint8_t requestFrom(uint8_t, uint8_t n) {
    rx.clear();
    if (shortRead) return 0;
    if (reg == 0x3A) {
      rx.push_back(fifo.size() & 255);
      rx.push_back((fifo.size() >> 8) | (overrun ? 0x40 : 0));
    } else if (reg == 0x78 && !fifo.empty()) {
      for (auto b : fifo.front()) rx.push_back(b);
      fifo.pop_front();
    } else for (uint8_t i = 0; i < n; ++i) rx.push_back(registers[reg + i]);
    return rx.size();
  }
  int read() { auto value = rx.front(); rx.pop_front(); return value; }
  void sample(uint8_t tag, int16_t x, int16_t y, int16_t z) {
    std::array<uint8_t, 7> data{}; data[0] = tag << 3;
    int16_t axes[] = {x, y, z};
    for (int a = 0; a < 3; ++a) {
      data[a * 2 + 1] = axes[a] & 255;
      data[a * 2 + 2] = uint16_t(axes[a]) >> 8;
    }
    fifo.push_back(data);
  }
};
inline WireStub Wire;

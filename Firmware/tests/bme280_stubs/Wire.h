#pragma once
#include <Arduino.h>
#include <cassert>
#include <deque>
#include <vector>
struct BmeWire {
  uint8_t registers[256]{}, reg = 0, address = 0, present = 0x76;
  int failRead = -1, failWrite = -1, ignoreWrite = -1;
  bool stuck = false, inconsistentTrim = false;
  unsigned trimReads = 0, dataBursts = 0, forcedWrites = 0;
  uint32_t conversionStart = 0;
  std::vector<uint8_t> tx;
  std::deque<uint8_t> rx;
  void word(uint8_t r, uint16_t v) { registers[r] = v; registers[r+1] = v >> 8; }
  void raw(uint32_t t, uint32_t p, uint16_t h) {
    registers[0xF7] = p >> 12; registers[0xF8] = p >> 4; registers[0xF9] = p << 4;
    registers[0xFA] = t >> 12; registers[0xFB] = t >> 4; registers[0xFC] = t << 4;
    registers[0xFD] = h >> 8; registers[0xFE] = h;
  }
  void humidity(int16_t h4, int16_t h5) {
    registers[0xE4] = uint16_t(h4) >> 4;
    registers[0xE5] = (h4 & 15) | ((h5 & 15) << 4);
    registers[0xE6] = uint16_t(h5) >> 4;
  }
  BmeWire() {
    registers[0xD0] = 0x60;
    const int trims[] = {27504,26435,-1000,36477,-10685,3024,2855,140,-7,15500,-14600,6000};
    for (unsigned i=0;i<12;++i) word(0x88+2*i,trims[i]);
    registers[0xA1] = 75; word(0xE1,362); humidity(325,50); registers[0xE7] = 30;
    raw(519888,415148,35000);
  }
  void beginTransmission(uint8_t a) { address = a; tx.clear(); }
  void write(uint8_t b) { tx.push_back(b); }
  int endTransmission(bool = true) {
    if (address != present) return 2;
    reg = tx[0];
    if (tx.size() == 2) {
      if (reg == failWrite) return 3;
      if (reg != ignoreWrite) registers[reg] = tx[1];
      if (reg == 0xF4 && tx[1] == 0x25) { ++forcedWrites; conversionStart = millis(); }
    }
    return 0;
  }
  int requestFrom(uint8_t, uint8_t n) {
    rx.clear();
    if (reg == failRead) return 0;
    if (!stuck && registers[0xF4] == 0x25 && millis()-conversionStart >= 10) registers[0xF4] = 0x24;
    if (reg == 0xF3) registers[0xF3] = stuck ? 9 : 0;
    for (int i=0;i<n;++i) rx.push_back(registers[reg+i]);
    if (reg == 0x88 && ++trimReads == 2 && inconsistentTrim) rx.front() ^= 1;
    if (reg == 0xF7) { assert(n == 8); ++dataBursts; }
    return n;
  }
  int read() { const int b=rx.front(); rx.pop_front(); return b; }
};
inline BmeWire Wire;

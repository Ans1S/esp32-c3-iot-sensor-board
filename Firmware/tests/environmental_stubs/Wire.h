#pragma once
#include <cstdint>
#include <map>
#include <vector>
class TwoWire {
 public:
  std::map<uint8_t,uint8_t> devices;
  std::map<uint8_t,unsigned> readyOnBeginCount;
  std::map<uint8_t,int> variants;  // A negative value simulates a missing byte.
  std::vector<uint32_t> clocks;
  uint8_t address = 0, reg = 0;
  bool beginFails = false, failNextBegin = false;
  unsigned ends = 0;
  bool hasDevice(uint8_t selected) const {
    const auto ready = readyOnBeginCount.find(selected);
    return devices.count(selected) &&
        (ready == readyOnBeginCount.end() || clocks.size() >= ready->second);
  }
  uint32_t activeClock = 0;
  bool begin(int, int, uint32_t clock) { clocks.push_back(clock); activeClock = clock; if (failNextBegin) { failNextBegin = false; return false; } return !beginFails; }
  void setClock(uint32_t clock) { activeClock = clock; }
  void end() { ++ends; }
  void setTimeOut(uint16_t) {}
  void beginTransmission(uint8_t selected) { address = selected; }
  void write(uint8_t selected) { reg = selected; }
  int endTransmission(bool = true) { return hasDevice(address) ? 0 : 2; }
  size_t requestFrom(uint8_t, size_t count) {
    return hasDevice(address) && !(reg == 0xF0 && variants[address] < 0) ? count : 0;
  }
  int available() { return 1; }
  int read() { return reg == 0xF0 ? variants[address] : devices.at(address); }
};
inline TwoWire Wire;

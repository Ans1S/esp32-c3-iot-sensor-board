#pragma once
#include <cstdint>
#include <map>
#include <vector>
class TwoWire {
 public:
  std::map<uint8_t,uint8_t> devices;
  std::map<uint8_t,unsigned> readyOnBeginCount;
  std::vector<uint32_t> clocks;
  uint8_t address = 0, reg = 0;
  bool beginFails = false;
  unsigned ends = 0;
  bool hasDevice(uint8_t selected) const {
    const auto ready = readyOnBeginCount.find(selected);
    return devices.count(selected) &&
        (ready == readyOnBeginCount.end() || clocks.size() >= ready->second);
  }
  bool begin(int, int, uint32_t clock) { clocks.push_back(clock); return !beginFails; }
  void end() { ++ends; }
  void setTimeOut(uint16_t) {}
  void beginTransmission(uint8_t selected) { address = selected; }
  void write(uint8_t selected) { reg = selected; }
  int endTransmission(bool = true) { return hasDevice(address) ? 0 : 2; }
  size_t requestFrom(uint8_t, size_t count) {
    return hasDevice(address) ? count : 0;
  }
  int available() { return 1; }
  int read() { return devices.at(address); }
};
inline TwoWire Wire;

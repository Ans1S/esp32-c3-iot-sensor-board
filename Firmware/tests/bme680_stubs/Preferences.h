#pragma once
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>
class Preferences {
 public:
  inline static std::map<std::string, std::vector<uint8_t>> values;
  inline static unsigned stateWrites = 0, calibrationWrites = 0;
  inline static bool failWrite = false;
  inline static bool failClear = false;
  bool begin(const char*, bool) { return true; }
  size_t getBytesLength(const char* key) { return values[key].size(); }
  size_t getBytes(const char* key, void* data, size_t count) {
    const auto& value = values[key];
    if (value.size() != count) return 0;
    std::memcpy(data, value.data(), count); return count;
  }
  size_t putBytes(const char* key, const void* data, size_t count) {
    if (failWrite) return 0;
    const auto* bytes = static_cast<const uint8_t*>(data);
    values[key] = std::vector<uint8_t>(bytes, bytes + count);
    if (std::string(key) == "bsec") ++stateWrites;
    else if (std::string(key) == "cal") ++calibrationWrites;
    return count;
  }
  bool clear() { if (failClear) return false; values.clear(); return true; }
};

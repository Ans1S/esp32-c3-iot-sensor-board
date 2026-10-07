#pragma once
#include "../stubs/Arduino.h"
#include <map>
#include <vector>

inline std::map<std::string, std::vector<uint8_t>> sensorConfigPreferences;
inline std::string sensorConfigReadFailure, sensorConfigWriteFailure;
inline bool sensorConfigOpen = true, sensorConfigClear = true;
inline unsigned sensorConfigWrites = 0, sensorConfigReads = 0;
class Preferences {
 public:
  bool begin(const char*, bool) { return sensorConfigOpen; }
  size_t getBytesLength(const char* key) const {
    const auto found = sensorConfigPreferences.find(key);
    return found == sensorConfigPreferences.end() ? 0 : found->second.size();
  }
  size_t getBytes(const char* key, void* output, size_t length) const {
    ++sensorConfigReads;
    const auto found = sensorConfigPreferences.find(key);
    if (found == sensorConfigPreferences.end() || length < found->second.size()) return 0;
    const size_t count = found->second.size() - (sensorConfigReadFailure == key ? 1 : 0);
    memcpy(output, found->second.data(), count); return count;
  }
  size_t putBytes(const char* key, const void* input, size_t length) {
    if (sensorConfigWriteFailure == key) return 0;
    const auto* bytes = static_cast<const uint8_t*>(input);
    sensorConfigPreferences[key] = std::vector<uint8_t>(bytes, bytes + length);
    if (!strcmp(key, "config")) ++sensorConfigWrites;
    return length;
  }
  bool clear() {
    if (!sensorConfigClear) return false;
    sensorConfigPreferences.clear(); return true;
  }
};

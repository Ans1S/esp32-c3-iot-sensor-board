#pragma once
#include <map>
#include <string>
#include <vector>
#include <cstring>
inline std::map<std::string, std::vector<uint8_t>> stationOtaPreferences;
inline bool failNextOtaPreferenceWrite = false;
class Preferences {
 public:
  bool begin(const char*, bool) { return true; }
  size_t getBytesLength(const char* key) { return stationOtaPreferences[key].size(); }
  size_t getBytes(const char* key, void* output, size_t size) {
    if (getBytesLength(key) != size) return 0;
    memcpy(output, stationOtaPreferences[key].data(), size); return size;
  }
  size_t putBytes(const char* key, const void* bytes, size_t size) {
    if (failNextOtaPreferenceWrite) { failNextOtaPreferenceWrite = false; return 0; }
    const auto* data = static_cast<const uint8_t*>(bytes);
    stationOtaPreferences[key] = {data, data + size}; return size;
  }
};

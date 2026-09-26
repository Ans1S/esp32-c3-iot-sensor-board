#pragma once
#include <map>
#include <string>
#include <vector>
#include <cstring>
#include <cstdint>
inline std::map<std::string,std::vector<uint8_t>> recordingPreferences;
class Preferences {
 public:
  bool begin(const char*, bool) { return true; }
  template<class T> T get(const char* key, T fallback) const {
    auto found = recordingPreferences.find(key);
    if (found == recordingPreferences.end() || found->second.size() != sizeof(T)) return fallback;
    T value; memcpy(&value, found->second.data(), sizeof(value)); return value;
  }
  template<class T> size_t put(const char* key, T value) {
    auto& bytes = recordingPreferences[key]; bytes.resize(sizeof(T));
    memcpy(bytes.data(), &value, sizeof(T)); return sizeof(T);
  }
  uint64_t getULong64(const char* key, uint64_t fallback) const { return get(key,fallback); }
  uint32_t getUInt(const char* key, uint32_t fallback) const { return get(key,fallback); }
  uint8_t getUChar(const char* key, uint8_t fallback) const { return get(key,fallback); }
  size_t putULong64(const char* key, uint64_t value) { return put(key,value); }
  size_t putUInt(const char* key, uint32_t value) { return put(key,value); }
  size_t putUChar(const char* key, uint8_t value) { return put(key,value); }
};

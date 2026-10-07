#pragma once
#include <cstring>
#include <map>
#include <string>
#include <vector>
inline std::map<std::string, std::vector<uint8_t>> testPreferences;
inline bool testPreferencesBegin = true;
inline bool testPreferencesShortRead = false;
class Preferences {
 public:
  bool begin(const char*, bool) { return testPreferencesBegin; }
  bool isKey(const char* key) const { return testPreferences.count(key); }
  size_t getBytesLength(const char* key) const {
    const auto found = testPreferences.find(key);
    return found == testPreferences.end() ? 0 : found->second.size();
  }
  size_t getBytes(const char* key, void* output, size_t length) const {
    const auto found = testPreferences.find(key);
    if (found == testPreferences.end()) return 0;
    const size_t count = std::min(length, found->second.size()) - (testPreferencesShortRead && length ? 1 : 0);
    std::memcpy(output, found->second.data(), count); return count;
  }
  size_t putBytes(const char* key, const void* input, size_t length) {
    const auto* bytes = static_cast<const uint8_t*>(input);
    testPreferences[key] = std::vector<uint8_t>(bytes, bytes + length); return length;
  }
  bool remove(const char* key) { return testPreferences.erase(key); }
  bool clear() { testPreferences.clear(); return true; }
};

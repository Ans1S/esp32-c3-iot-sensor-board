#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <type_traits>
using std::isfinite;
template <typename T, typename U> auto min(T a, U b) { return a < b ? a : b; }
template <typename T, typename U> auto max(T a, U b) { return a > b ? a : b; }
template <typename T, typename U, typename V> auto constrain(T a, U b, V c) {
  return a < b ? b : a > c ? c : a;
}
class String {
 public:
  String(const char* value = "") : value_(value) {}
  String(std::string value) : value_(value) {}
  String substring(size_t start, size_t end = std::string::npos) const {
    return value_.substr(start, end == std::string::npos ? end : end - start);
  }
  void toCharArray(char* out, size_t capacity) const {
    std::snprintf(out, capacity, "%s", value_.c_str());
  }
  size_t length() const { return value_.size(); }
  const char* c_str() const { return value_.c_str(); }
 private:
  std::string value_;
};
struct SerialStub { template<typename... Args> void printf(const char*, Args...) {} };
inline SerialStub Serial;
inline uint32_t testMillis = 100;
inline uint32_t millis() { return testMillis; }
void testRadioTick();
inline void delay(uint32_t ms) { while (ms--) testRadioTick(); }
#define RTC_DATA_ATTR

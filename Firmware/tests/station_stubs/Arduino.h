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
inline size_t strlcpy(char* destination, const char* source, size_t capacity) {
  const size_t length = std::strlen(source);
  if (capacity) { const size_t copied = std::min(length, capacity - 1);
    std::memcpy(destination, source, copied); destination[copied] = 0; }
  return length;
}
class String {
 public:
  String(const char* value = "") : value_(value) {}
  String(std::string value) : value_(value) {}
  template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
  String(T value) : value_(std::to_string(value)) {}
  String substring(size_t start, size_t end = std::string::npos) const {
    return value_.substr(start, end == std::string::npos ? end : end - start);
  }
  void toCharArray(char* out, size_t capacity) const { std::snprintf(out, capacity, "%s", value_.c_str()); }
  size_t length() const { return value_.size(); }
  const char* c_str() const { return value_.c_str(); }
  operator const char*() const { return value_.c_str(); }
  friend String operator+(const String& left, const String& right) {
    return left.value_ + right.value_;
  }
  friend String operator+(const char* left, const String& right) { return String(left) + right; }
  friend String operator+(const String& left, const char* right) { return left + String(right); }
 private:
  std::string value_;
};
struct SerialStub { template<typename... Args> void printf(const char*, Args...) {} };
inline SerialStub Serial;
inline uint32_t testMillis = 100;
inline uint32_t millis() { return testMillis; }
inline void delay(uint32_t ms) { testMillis += ms; }
#define RTC_DATA_ATTR

#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

namespace station::webinput {

// Validate before narrowing HTTP form values to protocol fields. Arduino's
// toInt() accepts prefixes and cannot distinguish a missing value from zero.
inline bool unsignedDecimal(const char* text, uint32_t& output,
                            uint32_t minimum = 0,
                            uint32_t maximum = UINT32_MAX) {
  if (!text || !*text || minimum > maximum) return false;
  uint32_t value = 0;
  for (const char* cursor = text; *cursor; ++cursor) {
    if (*cursor < '0' || *cursor > '9') return false;
    const uint32_t digit = static_cast<uint32_t>(*cursor - '0');
    if (value > maximum / 10 ||
        (value == maximum / 10 && digit > maximum % 10)) return false;
    value = value * 10 + digit;
  }
  if (value < minimum) return false;
  output = value;
  return true;
}

inline bool finiteDecimal(const char* text, float& output,
                          float minimum, float maximum) {
  if (!text || !*text) return false;
  // Accept the decimal syntax emitted by number inputs, including exponents,
  // while rejecting whitespace, partial values, hexadecimal, NaN and infinity.
  const char* cursor = text;
  if (*cursor == '-' || *cursor == '+') ++cursor;
  bool digits = false;
  while (*cursor >= '0' && *cursor <= '9') { digits = true; ++cursor; }
  if (*cursor == '.') {
    ++cursor;
    while (*cursor >= '0' && *cursor <= '9') { digits = true; ++cursor; }
  }
  if (!digits) return false;
  if (*cursor == 'e' || *cursor == 'E') {
    ++cursor;
    if (*cursor == '-' || *cursor == '+') ++cursor;
    const char* exponent = cursor;
    while (*cursor >= '0' && *cursor <= '9') ++cursor;
    if (cursor == exponent) return false;
  }
  if (*cursor) return false;
  const float value = strtof(text, nullptr);
  if (!isfinite(value) || value < minimum || value > maximum) return false;
  output = value;
  return true;
}

}  // namespace station::webinput

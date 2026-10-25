#include "web_input.h"
#include "login_throttle.h"

#include <assert.h>
#include <stdio.h>

int main() {
  using station::webinput::unsignedDecimal;
  using station::webinput::finiteDecimal;
  uint32_t value = 42;
  assert(unsignedDecimal("0", value) && value == 0);
  assert(unsignedDecimal("4294967295", value) && value == UINT32_MAX);
  assert(unsignedDecimal("00008", value, 1, 8) && value == 8);
  const char* invalid[] = {nullptr, "", "-1", "+1", "1.0", "1foo", " 1",
                           "1 ", "4294967296", "999999999999999999999"};
  for (const auto* text : invalid) {
    value = 42;
    assert(!unsignedDecimal(text, value) && value == 42);
  }
  assert(!unsignedDecimal("0", value, 1, 8));
  assert(!unsignedDecimal("257", value, 0, 8));
  assert(!unsignedDecimal("259", value, 0, 255));
  assert(!unsignedDecimal("1", value, 2, 1));

  float number = 42;
  assert(finiteDecimal("-0.466", number, -10, 10) && number == -0.466F);
  assert(finiteDecimal("1e-1", number, -10, 10) && number == 0.1F);
  assert(finiteDecimal(".5", number, -10, 10) && number == 0.5F);
  const char* invalidFloats[] = {nullptr, "", "0.466bad", "NaN", "inf",
                                 "0x1p0", " 1", "1 ", "+", ".", "1e",
                                 "1e+", "1e999", "11"};
  for (const auto* text : invalidFloats) {
    number = 42;
    assert(!finiteDecimal(text, number, -10, 10) && number == 42);
  }
  station::LoginThrottle throttle;
  uint32_t retryAfter = 42;
  assert(!throttle.blocked(100, retryAfter) && retryAfter == 0);
  for (unsigned i = 0; i < 4; ++i) assert(!throttle.recordFailure(100+i, retryAfter));
  assert(throttle.recordFailure(104, retryAfter) && retryAfter == 30);
  assert(throttle.blocked(105, retryAfter) && retryAfter == 30);
  assert(!throttle.blocked(30104, retryAfter) && retryAfter == 0);
  for (unsigned i = 0; i < 5; ++i) throttle.recordFailure(40000+i, retryAfter);
  assert(!throttle.blocked(0x80010000ULL, retryAfter) && retryAfter == 0);
  // A current block still applies across the 32-bit millis boundary.
  for (unsigned i = 0; i < 5; ++i) throttle.recordFailure(0xfffffff0ULL+i, retryAfter);
  assert(throttle.blocked(0x100000010ULL, retryAfter));
  assert(!throttle.blocked(0x100010000ULL, retryAfter));
  // Many wraps and long quiet periods cannot resurrect a previous block.
  assert(!throttle.blocked(0x800000000ULL, retryAfter));
  puts("HTTP inputs and login throttling: bounded decimals, monotonic cooldowns and long uptime passed");
}

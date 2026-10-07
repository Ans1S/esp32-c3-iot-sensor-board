#include "web_input.h"

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
  puts("HTTP numeric inputs: complete decimals, bounds and narrowing passed");
}

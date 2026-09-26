#pragma once
#include <cstdint>
inline uint32_t esp_random() { static uint32_t value = 12345; value = value*1664525+1013904223; return value; }

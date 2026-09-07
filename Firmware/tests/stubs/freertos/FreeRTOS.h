#pragma once
#include <stdint.h>
constexpr uint32_t portMAX_DELAY = UINT32_MAX;
using TickType_t = uint32_t;
constexpr bool pdTRUE = true;
inline TickType_t pdMS_TO_TICKS(uint32_t value) { return value; }

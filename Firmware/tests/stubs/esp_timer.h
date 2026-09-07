#pragma once
#include <stdint.h>
extern int64_t testTimeUs;
inline int64_t esp_timer_get_time() { return testTimeUs; }

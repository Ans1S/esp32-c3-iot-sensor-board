#pragma once
#include <Arduino.h>
inline int64_t esp_timer_get_time() { return int64_t(testMillis)*1000; }

#pragma once
#include <Arduino.h>
inline uint64_t recordingClockExtraMs = 0;
inline int64_t esp_timer_get_time() { return int64_t(uint64_t(testMillis)+recordingClockExtraMs)*1000; }

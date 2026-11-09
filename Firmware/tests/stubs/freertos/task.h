#pragma once

#include "Arduino.h"
#include "freertos/FreeRTOS.h"

inline TickType_t xTaskGetTickCount() { return millis(); }

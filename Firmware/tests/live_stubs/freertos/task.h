#pragma once
#include "freertos/FreeRTOS.h"
using TaskHandle_t = void*;
inline void (*testAcquisitionEntry)(void*) = nullptr;
inline void* testAcquisitionContext = nullptr;
constexpr bool pdPASS = true;
inline bool xTaskCreate(void (*entry)(void*), const char*, uint32_t, void* context,
    uint32_t, TaskHandle_t* task) {
  testAcquisitionEntry = entry; testAcquisitionContext = context;
  *task = context; return pdPASS;
}

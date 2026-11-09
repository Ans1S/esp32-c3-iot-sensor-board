#pragma once
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
using TaskHandle_t = void*;
constexpr bool pdPASS = true;
inline bool xTaskCreate(void (*)(void*), const char*, uint32_t, void* context,
                        uint32_t, TaskHandle_t* task) {
  *task = context;
  return pdPASS;
}
inline TickType_t xTaskGetTickCount() { return millis(); }
inline void vTaskDelay(TickType_t ticks) { delay(ticks); }
void xTaskNotifyGive(TaskHandle_t task);
uint32_t ulTaskNotifyTake(bool clear, TickType_t ticks);

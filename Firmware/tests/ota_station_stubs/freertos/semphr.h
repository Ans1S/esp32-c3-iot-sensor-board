#pragma once
#include <cassert>
#include <freertos/FreeRTOS.h>
struct TestOtaMutex { bool held = false; };
using SemaphoreHandle_t = TestOtaMutex*;
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return new TestOtaMutex; }
inline bool xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t wait) {
  assert(mutex);
  if (mutex->held && wait == 0) return false;
  assert(!mutex->held); mutex->held = true; return true;
}
inline bool xSemaphoreGive(SemaphoreHandle_t mutex) { assert(mutex && mutex->held); mutex->held = false; return true; }

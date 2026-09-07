#pragma once
#include <assert.h>
// Deterministic re-entrant storage hooks stand in for a competing radio task.
// Taking a held mutex fails immediately instead of hanging a regression run.
struct TestMutex { bool held = false; };
using SemaphoreHandle_t = TestMutex*;
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return new TestMutex; }
inline bool xSemaphoreTake(SemaphoreHandle_t mutex, unsigned) {
  assert(mutex != nullptr && !mutex->held);
  mutex->held = true;
  return true;
}
inline bool xSemaphoreGive(SemaphoreHandle_t mutex) {
  assert(mutex != nullptr && mutex->held);
  mutex->held = false;
  return true;
}

#pragma once
#include <deque>
#include <vector>
#include <cstring>
#include "freertos/FreeRTOS.h"
void testRadioTick();
struct TestQueue { size_t capacity, itemSize; std::deque<std::vector<uint8_t>> data; };
using QueueHandle_t = TestQueue*;
inline QueueHandle_t xQueueCreate(size_t capacity, size_t itemSize) {
  return new TestQueue{capacity, itemSize, {}};
}
inline bool xQueueSend(QueueHandle_t queue, const void* item, TickType_t) {
  if (queue->data.size() == queue->capacity) return false;
  const auto* bytes = static_cast<const uint8_t*>(item);
  queue->data.emplace_back(bytes, bytes + queue->itemSize);
  return true;
}
inline bool xQueueReceive(QueueHandle_t queue, void* item, TickType_t wait) {
  while (queue->data.empty() && wait-- > 0) testRadioTick();
  if (queue->data.empty()) return false;
  memcpy(item, queue->data.front().data(), queue->itemSize);
  queue->data.pop_front();
  return true;
}
inline void vQueueDelete(QueueHandle_t queue) { delete queue; }

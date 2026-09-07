#pragma once
#include <cstdlib>
constexpr int MALLOC_CAP_SPIRAM = 1;
constexpr int MALLOC_CAP_8BIT = 2;
constexpr int MALLOC_CAP_INTERNAL = 4;
inline size_t testAllocations = 0;
inline void* heap_caps_calloc(size_t count, size_t size, int) {
  ++testAllocations;
  return calloc(count, size);
}
inline void heap_caps_free(void* pointer) { free(pointer); }
inline size_t heap_caps_get_free_size(int) { return 1024 * 1024; }

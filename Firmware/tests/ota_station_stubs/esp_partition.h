#pragma once
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <functional>
#include <vector>
using esp_partition_subtype_t = int;
constexpr int ESP_PARTITION_TYPE_DATA = 1, ESP_OK = 0;
struct esp_partition_t { uint32_t size = 0x180000; };
inline esp_partition_t stagingPartition;
inline std::vector<uint8_t> stagingFlash(0x180000, 0xff);
struct StagingRead { size_t offset, size; };
inline std::vector<StagingRead> stagingReads;
inline bool failLargeStagingReadOnce = false, failAllStagingReads = false;
inline std::function<void()> stagingReadHook;
inline const esp_partition_t* esp_partition_find_first(int, esp_partition_subtype_t, const char*) { return &stagingPartition; }
inline int esp_partition_read(const esp_partition_t* partition, size_t offset, void* output, size_t size) {
  assert(partition == &stagingPartition && offset + size <= stagingFlash.size());
  stagingReads.push_back({offset, size});
  if (stagingReadHook) { auto hook = stagingReadHook; stagingReadHook = {}; hook(); }
  if (failAllStagingReads) return -1;
  if (failLargeStagingReadOnce && size > 192) { failLargeStagingReadOnce = false; return -1; }
  memcpy(output, stagingFlash.data() + offset, size); return ESP_OK;
}
inline int esp_partition_write(const esp_partition_t* partition, size_t offset, const void* input, size_t size) {
  assert(partition == &stagingPartition && offset + size <= stagingFlash.size());
  memcpy(stagingFlash.data() + offset, input, size); return ESP_OK;
}
inline int esp_partition_erase_range(const esp_partition_t* partition, size_t offset, size_t size) {
  assert(partition == &stagingPartition && offset % 4096 == 0 && size % 4096 == 0 && offset + size <= stagingFlash.size());
  std::fill(stagingFlash.begin() + offset, stagingFlash.begin() + offset + size, 0xff); return ESP_OK;
}

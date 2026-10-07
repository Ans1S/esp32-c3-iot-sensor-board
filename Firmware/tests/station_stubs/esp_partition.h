#pragma once
#include <cassert>
#include <vector>
#include <cstring>
constexpr int ESP_OK = 0, ESP_FAIL = -1;
constexpr int ESP_PARTITION_TYPE_DATA = 1, ESP_PARTITION_SUBTYPE_DATA_SPIFFS = 0x82;
struct esp_partition_t { size_t size = 8192; };
inline esp_partition_t testFilesystemPartition;
inline std::vector<uint8_t> testFilesystemBytes(8192, 0xFF);
inline bool testPartitionPresent = true, testPartitionReadFails = false;
inline const esp_partition_t* esp_partition_find_first(int type, int subtype, const char* label) {
  assert(type == ESP_PARTITION_TYPE_DATA && subtype == ESP_PARTITION_SUBTYPE_DATA_SPIFFS && !strcmp(label, "spiffs"));
  return testPartitionPresent ? &testFilesystemPartition : nullptr;
}
inline int esp_partition_read(const esp_partition_t*, size_t offset, void* output, size_t size) {
  if (testPartitionReadFails || offset + size > testFilesystemBytes.size()) return ESP_FAIL;
  memcpy(output, testFilesystemBytes.data() + offset, size); return ESP_OK;
}

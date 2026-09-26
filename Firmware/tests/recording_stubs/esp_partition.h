#pragma once
#include <algorithm>
#include <vector>
#include <cstring>
#include <cassert>
using esp_partition_subtype_t = int;
constexpr int ESP_OK = 0, ESP_PARTITION_TYPE_DATA = 1;
struct esp_partition_t { size_t size = 0x160000; };
inline esp_partition_t recordingPartition;
inline std::vector<uint8_t> recordingFlash(0x160000,0xff);
inline bool recordingPartitionAvailable = true;
inline const esp_partition_t* esp_partition_find_first(int, int subtype, const char* label) {
  return recordingPartitionAvailable && subtype == 0x41 && !strcmp(label,"recordings") ? &recordingPartition : nullptr;
}
inline int esp_partition_read(const esp_partition_t*, size_t offset, void* data, size_t length) {
  if (offset + length > recordingFlash.size()) return -1;
  memcpy(data, recordingFlash.data()+offset, length); return 0;
}
inline int esp_partition_write(const esp_partition_t*, size_t offset, const void* data, size_t length) {
  if (offset + length > recordingFlash.size()) return -1;
  const auto* bytes = static_cast<const uint8_t*>(data);
  for (size_t i=0;i<length;++i) {
    assert((recordingFlash[offset+i]&bytes[i])==bytes[i]); recordingFlash[offset+i]&=bytes[i];
  }
  return 0;
}
inline int esp_partition_erase_range(const esp_partition_t*, size_t offset, size_t length) {
  if (offset+length > recordingFlash.size() || offset%4096 || length%4096) return -1;
  std::fill(recordingFlash.begin()+offset,recordingFlash.begin()+offset+length,0xff); return 0;
}

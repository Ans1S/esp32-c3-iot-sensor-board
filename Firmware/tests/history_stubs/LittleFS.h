#pragma once
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include "filesystem_guard.h"
struct HistoryFileSystem {
  size_t extraUsed = 0;
  mutable uint32_t cachedEpoch = 0;
  mutable size_t cachedDirBytes = 0;
  size_t totalBytes() const { return 0x1a0000; }
  size_t usedBytes() const {
    const uint32_t epoch = station::filesystemSpaceEpoch();
    if (epoch == 0 || epoch != cachedEpoch) {
      size_t bytes = 0;
      for (const auto& entry : std::filesystem::directory_iterator(NORMAL_HISTORY_MOUNT_PATH)) {
        if (entry.is_regular_file()) bytes += ((entry.file_size() + 4095) / 4096) * 4096;
      }
      cachedDirBytes = bytes;
      cachedEpoch = epoch;
    }
    return extraUsed + cachedDirBytes;
  }
};
inline HistoryFileSystem LittleFS;

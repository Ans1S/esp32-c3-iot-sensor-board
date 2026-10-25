#pragma once
#include <cstddef>
#include <filesystem>
struct HistoryFileSystem {
  size_t extraUsed = 0;
  size_t totalBytes() const { return 0x1a0000; }
  size_t usedBytes() const {
    size_t bytes = extraUsed;
    for (const auto& entry : std::filesystem::directory_iterator(NORMAL_HISTORY_MOUNT_PATH)) {
      if (entry.is_regular_file()) bytes += ((entry.file_size() + 4095) / 4096) * 4096;
    }
    return bytes;
  }
};
inline HistoryFileSystem LittleFS;

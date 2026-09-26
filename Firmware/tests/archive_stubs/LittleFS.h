#pragma once
#include <cstddef>
struct ArchiveFileSystem {
  size_t used = 0;
  size_t totalBytes() const { return 0x1a0000; }
  size_t usedBytes() const { return used; }
};
inline ArchiveFileSystem LittleFS;

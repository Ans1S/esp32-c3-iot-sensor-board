#pragma once

#include <cstddef>
#include <cstdio>

// Count production file operations and inject one-shot readback faults.
inline size_t testFileSeekCount = 0;
inline size_t testFileReadCount = 0;
inline size_t testFileSeekFailAt = 0;
inline size_t testFileReadFailAt = 0;
inline size_t testFileReadCorruptAt = 0;
inline int testFileSeek(FILE* file, long offset, int origin) {
  ++testFileSeekCount;
  if (testFileSeekFailAt == testFileSeekCount) return -1;
  return std::fseek(file, offset, origin);
}
inline size_t testFileRead(void* data, size_t size, size_t count, FILE* file) {
  ++testFileReadCount;
  if (testFileReadFailAt == testFileReadCount) return 0;
  const size_t read = std::fread(data, size, count, file);
  if (read && size && testFileReadCorruptAt == testFileReadCount)
    static_cast<unsigned char*>(data)[0] ^= 1;
  return read;
}

#define fseek(...) testFileSeek(__VA_ARGS__)
#define fread(...) testFileRead(__VA_ARGS__)

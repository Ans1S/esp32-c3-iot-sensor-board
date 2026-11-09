#pragma once

#include <cerrno>
#include <cstddef>
#include <cstdio>

// Count production file operations and inject one-shot readback faults.
inline size_t testFileSeekCount = 0;
inline size_t testFileReadCount = 0;
inline size_t testFileSeekFailAt = 0;
inline size_t testFileReadFailAt = 0;
inline size_t testFileReadCorruptAt = 0;
inline size_t testFileReadCorruptOffset = 0;
inline bool testFileFailNextOpen = false;
inline FILE* testFileOpen(const char* path, const char* mode) {
  if (testFileFailNextOpen) {
    testFileFailNextOpen = false;
    errno = EIO;
    return nullptr;
  }
  return std::fopen(path, mode);
}
inline int testFileSeek(FILE* file, long offset, int origin) {
  ++testFileSeekCount;
  if (testFileSeekFailAt == testFileSeekCount) return -1;
  return std::fseek(file, offset, origin);
}
inline size_t testFileRead(void* data, size_t size, size_t count, FILE* file) {
  ++testFileReadCount;
  if (testFileReadFailAt == testFileReadCount) return 0;
  const size_t read = std::fread(data, size, count, file);
  if (testFileReadCorruptAt == testFileReadCount && testFileReadCorruptOffset < read * size)
    static_cast<unsigned char*>(data)[testFileReadCorruptOffset] ^= 1;
  return read;
}

#define fseek(...) testFileSeek(__VA_ARGS__)
#define fread(...) testFileRead(__VA_ARGS__)
#define fopen(...) testFileOpen(__VA_ARGS__)

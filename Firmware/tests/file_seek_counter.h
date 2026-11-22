#pragma once

#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <string>

// Count production file operations and inject one-shot readback faults.
inline size_t testFileSeekCount = 0;
inline size_t testFileReadCount = 0;
inline size_t testFileSeekFailAt = 0;
inline size_t testFileReadFailAt = 0;
inline size_t testFileReadCorruptAt = 0;
inline size_t testFileReadCorruptOffset = 0;
inline bool testFileFailNextOpen = false;
#ifdef _WIN32
inline FILE* testCachedRwFile = nullptr;
inline std::string testCachedRwPath;
inline std::string testLastRwPath;
inline void testCloseCachedRwFile() {
  if (testCachedRwFile) {
    std::fclose(testCachedRwFile);
    testCachedRwFile = nullptr;
    testCachedRwPath.clear();
  }
}
#endif
inline FILE* testFileOpen(const char* path, const char* mode) {
  if (testFileFailNextOpen) {
    testFileFailNextOpen = false;
    errno = EIO;
    return nullptr;
  }
#ifdef _WIN32
  if (path && mode && std::strcmp(mode, "r+b") == 0) {
    if (testCachedRwFile && testCachedRwPath == path) {
      std::rewind(testCachedRwFile);
      std::clearerr(testCachedRwFile);
      return testCachedRwFile;
    }
    testCloseCachedRwFile();
    FILE* f = std::fopen(path, mode);
    if (f && testLastRwPath == path) {
      testCachedRwFile = f;
      testCachedRwPath = path;
    } else if (f) {
      testLastRwPath = path;
    }
    return f;
  }
  testCloseCachedRwFile();
  testLastRwPath.clear();
#endif
  return std::fopen(path, mode);
}
#ifdef _WIN32
inline int testFileClose(FILE* file) {
  if (file && file == testCachedRwFile) {
    const int rc = std::fflush(file);
    if (std::ftell(file) >= 4064L) {
      testCloseCachedRwFile();
    }
    return rc;
  }
  return std::fclose(file);
}
#endif
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
#ifdef _WIN32
#define fclose(...) testFileClose(__VA_ARGS__)
#endif

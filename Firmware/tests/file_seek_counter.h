#pragma once

#include <cstddef>
#include <cstdio>

// Count positioning calls made by the production file readers in host tests.
inline size_t testFileSeekCount = 0;
inline int testFileSeek(FILE* file, long offset, int origin) {
  ++testFileSeekCount;
  return std::fseek(file, offset, origin);
}

#define fseek(...) testFileSeek(__VA_ARGS__)

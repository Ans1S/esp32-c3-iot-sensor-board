#pragma once
#include <cassert>
#include "Arduino.h"
constexpr const char* FILE_READ = "r";
constexpr const char* FILE_WRITE = "w";
struct File {
  explicit operator bool() const { return false; }
  size_t size() const { return 0; }
  bool seek(size_t) { return false; }
  size_t read(uint8_t*, size_t) { return 0; }
  size_t write(const uint8_t*, size_t) { return 0; }
  void close() {}
  void flush() {}
};
struct TestLittleFS {
  bool mountWorks = true, formatWorks = true;
  unsigned mounts = 0, formats = 0;
  bool begin(bool formatOnFail) {
    assert(!formatOnFail); ++mounts; return mountWorks;
  }
  bool format() { ++formats; if (formatWorks) mountWorks = true; return formatWorks; }
  File open(const String&, const char*) { return {}; }
  bool exists(const String&) const { return false; }
  bool remove(const String&) { return true; }
  bool rename(const String&, const String&) { return true; }
};
inline TestLittleFS LittleFS;

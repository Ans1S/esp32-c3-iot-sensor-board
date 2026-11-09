#pragma once

#include <Arduino.h>
#include <algorithm>
#include <cstring>

namespace station {

enum class HttpBodyStatus { Ok, TooLarge, Timeout, Truncated, Invalid, NoMemory };

// HTTPClient handles TLS and the response headers. The body is read directly
// so its size and total duration are bounded even on a slow, open connection.
template <typename Client, typename Text>
class HttpResponseBody {
 public:
  HttpResponseBody(Client& client, Text& output, size_t limit, uint32_t timeoutMs)
      : client_(client), output_(output), limit_(limit), timeoutMs_(timeoutMs), started_(millis()) {}

  HttpBodyStatus read(int contentLength, bool chunked) {
    output_ = "";
    if (chunked) {
      readChunks();
    } else if (contentLength >= 0) {
      const size_t count = static_cast<size_t>(contentLength);
      if (reserve(count)) readExact(count);
    } else if (contentLength == -1) {
      while (fill() && append(size_ - offset_)) {}
    } else {
      status_ = HttpBodyStatus::Invalid;
    }
    if (status_ != HttpBodyStatus::Ok) output_ = "";
    return status_;
  }

 private:
  bool fill() {
    while (uint32_t(millis() - started_) < timeoutMs_) {
      if (offset_ < size_) return true;
      const int available = client_.available();
      if (available > 0) {
        const int count = client_.read(buffer_, std::min(size_t(available), sizeof(buffer_)));
        if (count > 0) {
          offset_ = 0;
          size_ = static_cast<size_t>(count);
          return true;
        }
      }
      if (!client_.connected()) return false;
      delay(1);
    }
    status_ = HttpBodyStatus::Timeout;
    return false;
  }

  bool reserve(size_t size) {
    if (size > limit_) status_ = HttpBodyStatus::TooLarge;
    else if (size > reserved_) {
      if (!output_.reserve(size)) status_ = HttpBodyStatus::NoMemory;
      else reserved_ = size;
    }
    return status_ == HttpBodyStatus::Ok;
  }

  bool append(size_t count) {
    if (count > limit_ - output_.length()) {
      status_ = HttpBodyStatus::TooLarge;
      return false;
    }
    const size_t needed = output_.length() + count;
    if (needed > reserved_) {
      const size_t grown = reserved_ > limit_ / 2 ? limit_ : reserved_ * 2;
      if (!reserve(std::max(needed, std::max(grown, std::min(limit_, sizeof(buffer_)))))) return false;
    }
    if (!output_.concat(reinterpret_cast<const char*>(buffer_ + offset_), count)) {
      status_ = HttpBodyStatus::NoMemory;
      return false;
    }
    offset_ += count;
    return true;
  }

  bool requireData() {
    if (fill()) return true;
    if (status_ == HttpBodyStatus::Ok) status_ = HttpBodyStatus::Truncated;
    return false;
  }

  bool readExact(size_t remaining) {
    if (remaining > limit_ - output_.length()) {
      status_ = HttpBodyStatus::TooLarge;
      return false;
    }
    while (remaining != 0) {
      if (!requireData()) return false;
      const size_t count = std::min(remaining, size_ - offset_);
      if (!append(count)) return false;
      remaining -= count;
    }
    return true;
  }

  bool expectLf() {
    if (!requireData()) return false;
    if (buffer_[offset_++] == '\n') return true;
    status_ = HttpBodyStatus::Invalid;
    return false;
  }

  bool readLine(char (&line)[128]) {
    size_t length = 0;
    while (requireData()) {
      const uint8_t byte = buffer_[offset_++];
      if (byte == '\r') {
        line[length] = '\0';
        return expectLf();
      }
      if (byte == '\n' || byte == 0 || length == sizeof(line) - 1) {
        status_ = HttpBodyStatus::Invalid;
        return false;
      }
      line[length++] = static_cast<char>(byte);
    }
    return false;
  }

  bool readChunkSize(const char* line, size_t& size) {
    const char* cursor = line;
    for (;; ++cursor) {
      const unsigned char c = *cursor;
      const unsigned char lower = c | 0x20;
      size_t digit;
      if (c >= '0' && c <= '9') digit = c - '0';
      else if (lower >= 'a' && lower <= 'f') digit = lower - 'a' + 10;
      else break;
      if (digit > limit_ || size > (limit_ - digit) / 16) {
        status_ = HttpBodyStatus::TooLarge;
        return false;
      }
      size = size * 16 + digit;
    }
    const bool hasDigits = cursor != line;
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (hasDigits && (*cursor == '\0' || *cursor == ';')) return true;
    status_ = HttpBodyStatus::Invalid;
    return false;
  }

  void readChunks() {
    char line[128];
    for (;;) {
      size_t size = 0;
      if (!readLine(line) || !readChunkSize(line, size)) return;
      if (size == 0) break;
      if (!readExact(size) || !requireData()) return;
      if (buffer_[offset_++] != '\r') {
        status_ = HttpBodyStatus::Invalid;
        return;
      }
      if (!expectLf()) return;
    }
    size_t trailers = 0;
    while (readLine(line) && line[0] != '\0') {
      trailers += strlen(line) + 2;
      if (trailers > 1024) {
        status_ = HttpBodyStatus::Invalid;
        return;
      }
    }
  }

  Client& client_;
  Text& output_;
  const size_t limit_;
  const uint32_t timeoutMs_, started_;
  size_t offset_ = 0, size_ = 0, reserved_ = 0;
  HttpBodyStatus status_ = HttpBodyStatus::Ok;
  uint8_t buffer_[512];
};

}  // namespace station

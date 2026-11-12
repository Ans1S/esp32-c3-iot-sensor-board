#pragma once

#include <ctype.h>

namespace station {

// Prefer gzip only when the client explicitly advertises it. A malformed
// weight is treated as zero, so it cannot accidentally enable compression.
inline bool acceptsDashboardGzip(const char* header) {
  const char* p = header;
  while (*p) {
    while (*p == ' ' || *p == '\t' || *p == ',') ++p;
    const char* start = p;
    while (*p && *p != ',' && *p != ';' && *p != ' ' && *p != '\t') ++p;
    const bool gzip = p - start == 4 && tolower(static_cast<unsigned char>(start[0])) == 'g' &&
                      tolower(static_cast<unsigned char>(start[1])) == 'z' &&
                      tolower(static_cast<unsigned char>(start[2])) == 'i' &&
                      tolower(static_cast<unsigned char>(start[3])) == 'p';
    bool acceptable = true;
    while (*p && *p != ',') {
      while (*p == ' ' || *p == '\t') ++p;
      if (!*p || *p == ',') break;
      if (*p != ';') { acceptable = false; while (*p && *p != ',') ++p; break; }
      ++p;
      while (*p == ' ' || *p == '\t') ++p;
      if ((*p != 'q' && *p != 'Q') || p[1] != '=') {
        acceptable = false;
        while (*p && *p != ',') ++p;
        break;
      }
      p += 2;
      const char* number = p;
      if (*p != '0' && *p != '1') { acceptable = false; while (*p && *p != ',') ++p; break; }
      const bool one = *p++ == '1';
      bool positive = one;
      if (*p == '.') {
        ++p;
        unsigned digits = 0;
        while (*p >= '0' && *p <= '9') {
          if (*p != '0') positive = true;
          if (one && *p != '0') acceptable = false;
          ++p; ++digits;
        }
        if (digits > 3) acceptable = false;
      }
      if (p == number || (*p && *p != ',' && *p != ' ' && *p != '\t')) acceptable = false;
      acceptable = acceptable && positive;
    }
    if (gzip && acceptable) return true;
    if (*p == ',') ++p;
  }
  return false;
}

}  // namespace station

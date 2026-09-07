#pragma once

#include <Arduino.h>

#ifndef SENSOR_DIAGNOSTIC_LOGGING
#define SENSOR_DIAGNOSTIC_LOGGING 0
#endif

namespace sensor {

inline void beginDiagnosticLogging() {
#if SENSOR_DIAGNOSTIC_LOGGING
  Serial.begin(115200);
#endif
}

inline void flushDiagnosticLogging() {
#if SENSOR_DIAGNOSTIC_LOGGING
  Serial.flush();
#endif
}

}  // namespace sensor

#if SENSOR_DIAGNOSTIC_LOGGING
#define SENSOR_LOG_PRINTLN(...)  \
  do {                           \
    Serial.println(__VA_ARGS__); \
  } while (false)
#define SENSOR_LOG_PRINTF(...)  \
  do {                          \
    Serial.printf(__VA_ARGS__); \
  } while (false)
#else
#define SENSOR_LOG_PRINTLN(...) \
  do {                          \
  } while (false)
#define SENSOR_LOG_PRINTF(...) \
  do {                         \
  } while (false)
#endif

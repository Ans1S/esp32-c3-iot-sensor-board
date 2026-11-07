#pragma once
#include <Arduino.h>
// JSON is outside this transfer fixture. Real serializer tests and target
// builds use the installed ArduinoJson library instead of these inert proxies.
struct JsonValue {
  template <typename T> JsonValue& operator=(const T&) { return *this; }
  template <typename T> T to() { return {}; }
};
struct JsonObject { JsonValue operator[](const char*) { return {}; } };
struct JsonArray { template <typename T> T add() { return {}; } };

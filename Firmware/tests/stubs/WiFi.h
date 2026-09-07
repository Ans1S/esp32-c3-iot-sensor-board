#pragma once
constexpr int WIFI_OFF = 0;
constexpr int WIFI_STA = 1;
struct TestWifi {
  int currentMode = WIFI_OFF;
  unsigned shutdowns = 0;
  void persistent(bool) {}
  void disconnect(bool, bool) {}
  void setSleep(bool) {}
  void mode(int mode) { currentMode = mode; if (mode == WIFI_OFF) ++shutdowns; }
  int getMode() const { return currentMode; }
};
inline TestWifi WiFi;

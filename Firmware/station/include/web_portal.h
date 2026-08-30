#pragma once

#include <WebServer.h>

#include "app_config.h"
#include "config_store.h"
#include "espnow_gateway.h"
#include "sensor_registry.h"
#include "thingspeak_service.h"
#include "wifi_service.h"

namespace station {

class WebPortal {
 public:
  bool begin(StationConfig& config, ConfigStore& store, WifiService& wifi,
             SensorRegistry& registry, ThingSpeakService& thingSpeak,
             EspNowGateway& gateway);
 void loop();

 private:
  bool authenticationEnabled() const;
  bool isAuthenticated() const;
  bool requireAuthentication(bool redirectToLogin = false);
  bool verifyAdminPassword(const String& password) const;
  void setAdminPassword(const String& password);
  void refreshAuthToken();
  void handleLogin();
  void handleLogout();
  bool verifyCsrf();
  void sendJsonStatus();
  void sendJsonHistory();
  void sendJsonConfig();
  void sendWifiScan();
  void saveSetup();
  void saveSetupSensor();
  void saveStation();
  void saveSensor();
  void saveChannelProfile();
  void deleteChannelProfile();
  void createThingSpeakChannel();
  void listThingSpeakChannels();
  void createManagedThingSpeakChannel();
  void updateManagedThingSpeakChannel();
  void clearManagedThingSpeakChannel();
  void deleteManagedThingSpeakChannel();
  void resetSensor();
  void resetSensorIaqCalibration();
  void deleteSensor();
  void factoryReset();
  void sendJsonResult(bool success, const String& message,
                      bool restartRequired = false,
                      bool authenticationChanged = false);
  void scheduleRestart();

  WebServer server_{80};
  StationConfig* config_ = nullptr;
  ConfigStore* store_ = nullptr;
  WifiService* wifi_ = nullptr;
  SensorRegistry* registry_ = nullptr;
  ThingSpeakService* thingSpeak_ = nullptr;
  EspNowGateway* gateway_ = nullptr;
  uint32_t restartAtMs_ = 0;
  uint32_t failedLoginAttempts_ = 0;
  uint32_t loginBlockedUntilMs_ = 0;
  char csrfToken_[17]{};
  char authToken_[33]{};
};

}  // namespace station

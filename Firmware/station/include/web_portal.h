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
  struct AuthSession {
    char token[33]{};
    uint64_t createdAtMs = 0;
    bool active = false;
  };

  struct LoginThrottle {
    uint32_t lastAttemptMs = 0;
    uint32_t blockedUntilMs = 0;
    uint8_t failures = 0;
  };

  bool authenticationEnabled() const;
  bool isAuthenticated();
  bool requireAuthentication(bool redirectToLogin = false);
  bool verifyAdminPassword(const String& password) const;
  void setAdminPassword(const String& password);
  void refreshCsrfToken();
  const char* createAuthSession();
  void clearAuthSessions();
  void revokeCurrentSession();
  String currentSessionToken() const;
  bool loginBlocked(uint32_t& retryAfterSeconds);
  bool recordFailedLogin(uint32_t& retryAfterSeconds);
  void clearLoginFailures();
  bool validRequestOrigin() const;
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
  AuthSession authSessions_[4]{};
  LoginThrottle loginThrottle_{};
  char csrfToken_[33]{};
};

}  // namespace station

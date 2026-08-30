#pragma once

#include <DNSServer.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#include "app_config.h"

namespace station {

struct WifiNetwork {
  String ssid;
  int32_t rssi = 0;
  uint8_t channel = 0;
  bool secure = false;
};

class WifiService {
 public:
  bool begin(const StationConfig& config);
  void loop();
  uint8_t channel() const;
  bool stationConnected() const;
  bool setupPortalActive() const;
  bool consumeSetupPortalCompletion();
  String stationSsid() const;
  String stationIp() const;
  String accessPointIp() const;
  String accessPointSsid() const;
  const uint8_t* stationMac() const;
  bool requestNetworkScan();
  bool networkScanInProgress() const;
  size_t cachedNetworks(WifiNetwork* output, size_t capacity) const;

 private:
  void startStationConnection();
  void startMdnsIfNeeded();
  void stopSetupPortal();
  bool startSetupPortalAccessPoint();
  bool startRecoveryPortal();
  void collectNetworkScan();

  static constexpr size_t kNetworkCacheSize = 20;

  StationConfig config_{};
  DNSServer dnsServer_;
  uint8_t stationMac_[6]{};
  String apSsid_;
  uint32_t lastReconnectAttemptMs_ = 0;
  uint32_t lastMdnsAttemptMs_ = 0;
  uint32_t disconnectedSinceMs_ = 0;
  wl_status_t lastReportedStatus_ = WL_NO_SHIELD;
  bool mdnsStarted_ = false;
  bool setupPortalActive_ = false;
  bool setupPortalCompletionPending_ = false;
  bool networkScanInProgress_ = false;
  WifiNetwork networkCache_[kNetworkCacheSize]{};
  size_t networkCacheCount_ = 0;
};

}  // namespace station

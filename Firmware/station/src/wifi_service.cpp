#include "wifi_service.h"

#include <algorithm>
#include <esp_mac.h>
#include <esp_wifi.h>

namespace station {

namespace {
constexpr uint32_t kReconnectIntervalMs = 10000;
constexpr uint32_t kPortalReconnectIntervalMs = 30000;
constexpr uint32_t kRecoveryPortalDelayMs = 30000;
constexpr uint32_t kRecoveryRetryIntervalMs = 15000;
constexpr uint32_t kMdnsRetryIntervalMs = 5000;
constexpr uint16_t kDnsPort = 53;
constexpr uint32_t kPortalStartSettleMs = 150;
const IPAddress kPortalIp(192, 168, 4, 1);
const IPAddress kPortalGateway(192, 168, 4, 1);
const IPAddress kPortalSubnet(255, 255, 255, 0);
}

bool WifiService::begin(const StationConfig& config) {
  config_ = config;
  setupPortalActive_ = false;
  setupPortalCompletionPending_ = false;
  lastRecoveryAttemptMs_ = 0;
  hasRecoveryAttempt_ = false;
  hasMdnsAttempt_ = false;
  hasDisconnectedSince_ = false;
  lastStaChannel_ = 0;
  esp_read_mac(stationMac_, ESP_MAC_WIFI_STA);

  char suffix[7];
  snprintf(suffix, sizeof(suffix), "%02X%02X%02X", stationMac_[3],
           stationMac_[4], stationMac_[5]);
  apSsid_ = "W-Charger-" + String(suffix);

  // Do not reuse the Arduino Wi-Fi driver's credentials or its autonomous
  // reconnect task. The station owns reconnection below. In particular, do
  // not cycle through WIFI_OFF here: on the ESP32-S3 that stop/start sequence
  // can race the Arduino network-event task during PHY wake-up after a cold
  // boot (the resulting panic is reported as an MMU/cache error).
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  // Set hostname before enabling the STA interface so DHCP includes it.
  WiFi.setHostname(config_.hostname);
  const bool setupRequired = config_.wifiSsid[0] == '\0' ||
                             config_.setupPortalRequired;
  Serial.printf("[WiFi] Starting radio (%s) ...\n",
                setupRequired ? "Setup AP+STA" : "home Wi-Fi STA");
  if (!WiFi.mode(setupRequired ? WIFI_AP_STA : WIFI_STA)) {
    Serial.println("[WiFi] Radio mode could not be initialized");
    return false;
  }
  WiFi.setSleep(false);  // The USB-powered station prioritizes ESP-NOW latency.

  if (setupRequired) {
    if (!startSetupPortalAccessPoint()) {
      return false;
    }
  } else {
    // WIFI_STA above is the only active interface. Calling softAPdisconnect()
    // here would perform another asynchronous interface transition while the
    // PHY is still starting.
    dnsServer_.stop();
    disconnectedSinceMs_ = millis();
    hasDisconnectedSince_ = true;
  }
  startStationConnection();
  return true;
}

void WifiService::startStationConnection() {
  if (config_.wifiSsid[0] == '\0') {
    return;
  }
  Serial.printf("[WiFi] Connecting to '%s' ...\n", config_.wifiSsid);
  WiFi.begin(config_.wifiSsid, config_.wifiPassword);
  lastReconnectAttemptMs_ = millis();
}

void WifiService::startMdnsIfNeeded() {
  if (mdnsStarted_ || !stationConnected()) {
    return;
  }
  if (hasMdnsAttempt_ &&
      uint32_t(millis() - lastMdnsAttemptMs_) < kMdnsRetryIntervalMs) {
    return;
  }
  lastMdnsAttemptMs_ = millis();
  hasMdnsAttempt_ = true;
  if (MDNS.begin(config_.hostname)) {
    MDNS.addService("http", "tcp", 80);
    mdnsStarted_ = true;
    Serial.printf("[mDNS] http://%s.local/ active\n", config_.hostname);
  } else {
    Serial.println("[mDNS] Start failed; retrying in 5 seconds");
  }
}

void WifiService::loop() {
  collectNetworkScan();
  if (setupPortalActive_) {
    dnsServer_.processNextRequest();
  }
  const wl_status_t currentStatus = WiFi.status();
  if (currentStatus != lastReportedStatus_) {
    lastReportedStatus_ = currentStatus;
    if (currentStatus == WL_CONNECTED) {
      Serial.printf("[WiFi] Connected: %s, IP %s, channel %ld\n",
                    WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(),
                    static_cast<long>(WiFi.channel()));
    } else {
      Serial.printf("[WiFi] Status %d, not connected yet\n",
                    static_cast<int>(currentStatus));
    }
  }

  if (stationConnected()) {
    disconnectedSinceMs_ = 0;
    hasDisconnectedSince_ = false;
    lastRecoveryAttemptMs_ = 0;
    hasRecoveryAttempt_ = false;
    if (!networkScanInProgress_) {
      uint8_t primary = 0;
      wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
      if (esp_wifi_get_channel(&primary, &secondary) == ESP_OK &&
          primary >= 1 && primary <= 13) {
        lastStaChannel_ = primary;
      }
    }
    startMdnsIfNeeded();
  } else {
    if (!hasDisconnectedSince_) {
      disconnectedSinceMs_ = millis();
      hasDisconnectedSince_ = true;
    }
    if (mdnsStarted_) {
      MDNS.end();
      mdnsStarted_ = false;
      lastMdnsAttemptMs_ = 0;
      hasMdnsAttempt_ = false;
    }
  }

  if (setupPortalActive_ && stationConnected()) {
    stopSetupPortal();
  }

  const uint32_t reconnectIntervalMs =
      setupPortalActive_ ? kPortalReconnectIntervalMs : kReconnectIntervalMs;
  if (config_.wifiSsid[0] != '\0' && !stationConnected() &&
      !networkScanInProgress_ &&
      uint32_t(millis() - lastReconnectAttemptMs_) >= reconnectIntervalMs) {
    WiFi.disconnect(false, false);
    startStationConnection();
  }

  if (!setupPortalActive_ && config_.wifiSsid[0] != '\0' &&
      !stationConnected() && hasDisconnectedSince_ &&
      uint32_t(millis() - disconnectedSinceMs_) >= kRecoveryPortalDelayMs &&
      (!hasRecoveryAttempt_ ||
       uint32_t(millis() - lastRecoveryAttemptMs_) >= kRecoveryRetryIntervalMs)) {
    lastRecoveryAttemptMs_ = millis();
    hasRecoveryAttempt_ = true;
    startRecoveryPortal();
  }
}

bool WifiService::startRecoveryPortal() {
  if (!WiFi.mode(WIFI_AP_STA)) {
    Serial.println("[RECOVERY] Could not enable AP+STA mode");
    return false;
  }
  WiFi.setSleep(false);
  delay(kPortalStartSettleMs);
  return startSetupPortalAccessPoint();
}

bool WifiService::startSetupPortalAccessPoint() {
  // A fixed private address makes the fallback URL predictable even when the
  // operating system does not display its captive-portal dialog.
  if (!WiFi.softAPConfig(kPortalIp, kPortalGateway, kPortalSubnet)) {
    Serial.println("[PORTAL] Could not configure the access-point address");
    return false;
  }
  const char* apPassword = strlen(config_.accessPointPassword) >= 8
                               ? config_.accessPointPassword
                               : nullptr;
  if (!WiFi.softAP(apSsid_.c_str(), apPassword,
                   config_.fallbackWifiChannel, false, 4)) {
    Serial.println("[PORTAL] Could not start the setup access point");
    return false;
  }
  delay(kPortalStartSettleMs);
  const IPAddress portalIp = WiFi.softAPIP();
  if (portalIp != kPortalIp ||
      !dnsServer_.start(kDnsPort, "*", portalIp)) {
    Serial.println("[PORTAL] DNS server could not start");
    WiFi.softAPdisconnect(true);
    return false;
  }
  setupPortalActive_ = true;
  Serial.printf("[PORTAL] Setup Wi-Fi ready: %s, http://%s/\n",
                apSsid_.c_str(), portalIp.toString().c_str());
  return true;
}

void WifiService::stopSetupPortal() {
  if (!setupPortalActive_) {
    return;
  }
  dnsServer_.stop();
  // softAPdisconnect() already removes only the AP interface and leaves STA
  // enabled. A second WiFi.mode(WIFI_STA) transition is both redundant and
  // unsafe while ESP-NOW is active.
  if (!WiFi.softAPdisconnect(true)) {
    Serial.println("[PORTAL] SoftAP stop failed; retrying");
    return;
  }
  setupPortalActive_ = false;
  if (config_.setupPortalRequired) {
    setupPortalCompletionPending_ = true;
  }
}

uint8_t WifiService::channel() const {
  const uint8_t cached = lastStaChannel_.load(std::memory_order_relaxed);
  // Never advertise a transient off-channel frequency during an active Wi-Fi scan.
  if (networkScanInProgress_.load(std::memory_order_acquire)) {
    return cached >= 1 && cached <= 13 ? cached : config_.fallbackWifiChannel;
  }
  uint8_t primary = config_.fallbackWifiChannel;
  wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
  if (esp_wifi_get_channel(&primary, &secondary) != ESP_OK ||
      primary < 1 || primary > 13) {
    return cached >= 1 && cached <= 13 ? cached : config_.fallbackWifiChannel;
  }
  return primary;
}

bool WifiService::stationConnected() const {
  return WiFi.status() == WL_CONNECTED;
}

bool WifiService::setupPortalActive() const {
  return setupPortalActive_;
}

bool WifiService::consumeSetupPortalCompletion() {
  const bool completed = setupPortalCompletionPending_;
  setupPortalCompletionPending_ = false;
  return completed;
}

String WifiService::stationSsid() const {
  return stationConnected() ? WiFi.SSID() : String(config_.wifiSsid);
}

String WifiService::stationIp() const {
  return stationConnected() ? WiFi.localIP().toString() : String("-");
}

String WifiService::accessPointIp() const {
  return setupPortalActive_ ? WiFi.softAPIP().toString() : String("-");
}

String WifiService::accessPointSsid() const {
  return apSsid_;
}

const uint8_t* WifiService::stationMac() const {
  return stationMac_;
}

bool WifiService::requestNetworkScan() {
  if (networkScanInProgress_.load(std::memory_order_acquire)) {
    return true;
  }
  WiFi.scanDelete();
  // Publish scan-in-progress before esp_wifi_scan_start() begins channel
  // hopping on Core 0 so espnow-gateway never advertises a transient scan
  // channel as stable.
  networkScanInProgress_.store(true, std::memory_order_release);
  const int16_t result = WiFi.scanNetworks(true, true);
  if (result == WIFI_SCAN_RUNNING) {
    return true;
  }
  if (result >= 0) {
    collectNetworkScan();
    return true;
  }
  networkScanInProgress_.store(false, std::memory_order_release);
  return false;
}

bool WifiService::networkScanInProgress() const {
  return networkScanInProgress_;
}

void WifiService::collectNetworkScan() {
  const int16_t found = WiFi.scanComplete();
  if (found == WIFI_SCAN_RUNNING) {
    networkScanInProgress_ = true;
    return;
  }
  if (!networkScanInProgress_) {
    return;
  }
  networkScanInProgress_ = false;
  networkCacheCount_ = 0;
  if (found <= 0) {
    WiFi.scanDelete();
    return;
  }
  for (int16_t i = 0; i < found; ++i) {
    const String ssid = WiFi.SSID(i);
    if (ssid.isEmpty()) {
      continue;
    }
    const int32_t rssi = WiFi.RSSI(i);
    const uint8_t ch = static_cast<uint8_t>(WiFi.channel(i));
    const bool secure = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
    size_t existing = networkCacheCount_;
    for (size_t j = 0; j < networkCacheCount_; ++j) {
      if (networkCache_[j].ssid == ssid) {
        existing = j;
        break;
      }
    }
    if (existing < networkCacheCount_) {
      if (rssi > networkCache_[existing].rssi) {
        networkCache_[existing].rssi = rssi;
        networkCache_[existing].channel = ch;
        networkCache_[existing].secure = secure;
      }
      continue;
    }
    if (networkCacheCount_ < kNetworkCacheSize) {
      auto& network = networkCache_[networkCacheCount_++];
      network.ssid = ssid;
      network.rssi = rssi;
      network.channel = ch;
      network.secure = secure;
    } else {
      size_t weakest = 0;
      for (size_t j = 1; j < networkCacheCount_; ++j) {
        if (networkCache_[j].rssi < networkCache_[weakest].rssi) {
          weakest = j;
        }
      }
      if (rssi > networkCache_[weakest].rssi) {
        networkCache_[weakest].ssid = ssid;
        networkCache_[weakest].rssi = rssi;
        networkCache_[weakest].channel = ch;
        networkCache_[weakest].secure = secure;
      }
    }
  }
  std::sort(networkCache_, networkCache_ + networkCacheCount_,
            [](const WifiNetwork& a, const WifiNetwork& b) {
              return a.rssi > b.rssi;
            });
  WiFi.scanDelete();
}

size_t WifiService::cachedNetworks(WifiNetwork* output,
                                   size_t capacity) const {
  if (output == nullptr || capacity == 0) {
    return 0;
  }
  const size_t written = min(capacity, networkCacheCount_);
  for (size_t i = 0; i < written; ++i) {
    output[i] = networkCache_[i];
  }
  return written;
}

}  // namespace station

#include "wifi_service.h"

#include <esp_mac.h>
#include <esp_wifi.h>

namespace station {

namespace {
constexpr uint32_t kReconnectIntervalMs = 10000;
constexpr uint32_t kRecoveryPortalDelayMs = 30000;
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
  const bool setupRequired = config_.wifiSsid[0] == '\0' ||
                             config_.setupPortalRequired;
  Serial.printf("[WiFi] Starting radio (%s) ...\n",
                setupRequired ? "Setup AP+STA" : "home Wi-Fi STA");
  if (!WiFi.mode(setupRequired ? WIFI_AP_STA : WIFI_STA)) {
    Serial.println("[WiFi] Radio mode could not be initialized");
    return false;
  }
  WiFi.setHostname(config_.hostname);
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
  if (lastMdnsAttemptMs_ != 0 &&
      millis() - lastMdnsAttemptMs_ < kMdnsRetryIntervalMs) {
    return;
  }
  lastMdnsAttemptMs_ = millis();
  if (MDNS.begin(config_.hostname)) {
    MDNS.addService("http", "tcp", 80);
    mdnsStarted_ = true;
    Serial.printf("[mDNS] http://%s.local/ aktiv\n", config_.hostname);
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
    startMdnsIfNeeded();
  } else {
    if (disconnectedSinceMs_ == 0) {
      disconnectedSinceMs_ = millis();
    }
    if (mdnsStarted_) {
      MDNS.end();
      mdnsStarted_ = false;
      lastMdnsAttemptMs_ = 0;
    }
  }

  if (setupPortalActive_ && stationConnected()) {
    stopSetupPortal();
  }

  if (config_.wifiSsid[0] != '\0' && !stationConnected() &&
      millis() - lastReconnectAttemptMs_ >= kReconnectIntervalMs) {
    WiFi.disconnect(false, false);
    startStationConnection();
  }

  if (!setupPortalActive_ && config_.wifiSsid[0] != '\0' &&
      !stationConnected() && disconnectedSinceMs_ != 0 &&
      millis() - disconnectedSinceMs_ >= kRecoveryPortalDelayMs) {
    startRecoveryPortal();
  }
}

bool WifiService::startRecoveryPortal() {
  if (!WiFi.mode(WIFI_AP_STA)) {
    Serial.println("[RECOVERY] Could not enable AP+STA mode");
    return false;
  }
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
  WiFi.softAPdisconnect(true);
  setupPortalActive_ = false;
  setupPortalCompletionPending_ = true;
}

uint8_t WifiService::channel() const {
  uint8_t primary = config_.fallbackWifiChannel;
  wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
  if (esp_wifi_get_channel(&primary, &secondary) != ESP_OK) {
    return config_.fallbackWifiChannel;
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
  if (networkScanInProgress_) {
    return true;
  }
  WiFi.scanDelete();
  const int16_t result = WiFi.scanNetworks(true, true);
  networkScanInProgress_ = result == WIFI_SCAN_RUNNING;
  if (result >= 0) {
    networkScanInProgress_ = true;
    collectNetworkScan();
  }
  return networkScanInProgress_ || result >= 0;
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
  for (int16_t i = 0; i < found && networkCacheCount_ < kNetworkCacheSize;
       ++i) {
    const String ssid = WiFi.SSID(i);
    if (ssid.isEmpty()) {
      continue;
    }
    bool duplicate = false;
    for (size_t existing = 0; existing < networkCacheCount_; ++existing) {
      if (networkCache_[existing].ssid == ssid) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) {
      continue;
    }
    auto& network = networkCache_[networkCacheCount_++];
    network.ssid = ssid;
    network.rssi = WiFi.RSSI(i);
    network.channel = static_cast<uint8_t>(WiFi.channel(i));
    network.secure = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
  }
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

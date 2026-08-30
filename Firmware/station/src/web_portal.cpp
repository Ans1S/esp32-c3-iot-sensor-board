#include "web_portal.h"

#include <ArduinoJson.h>
#include <SHA2Builder.h>
#include <esp_random.h>

#include "web_pages.h"

namespace station {

namespace {
constexpr size_t kMaxScannedNetworks = 20;
constexpr char kSessionCookieName[] = "wch_session";
constexpr char kPasswordHashDomain[] = "W-Charger website password v1";
constexpr uint32_t kMaxLoginAttempts = 5;
constexpr uint32_t kLoginBlockMs = 30000;

void hashAdminPassword(const String& password, char output[65]) {
  SHA256Builder hash;
  hash.begin();
  const uint64_t deviceId = ESP.getEfuseMac();
  hash.add(reinterpret_cast<const uint8_t*>(kPasswordHashDomain),
           strlen(kPasswordHashDomain));
  hash.add(reinterpret_cast<const uint8_t*>(&deviceId), sizeof(deviceId));
  hash.add(reinterpret_cast<const uint8_t*>(password.c_str()),
           password.length());
  hash.calculate();
  hash.getChars(output);
}

bool constantTimeEquals(const char* first, const char* second,
                        size_t length) {
  uint8_t difference = 0;
  for (size_t i = 0; i < length; ++i) {
    difference |= static_cast<uint8_t>(first[i] ^ second[i]);
  }
  return difference == 0;
}

void appendFormValue(String& body, const char* name, const String& value) {
  if (!body.isEmpty()) {
    body += '&';
  }
  body += name;
  body += '=';
  body += ThingSpeakService::urlEncode(value);
}

void extractApiKeys(JsonObjectConst channel, String& readKey,
                    String& writeKey) {
  for (JsonObjectConst key : channel["api_keys"].as<JsonArrayConst>()) {
    if (key["write_flag"] | false) {
      writeKey = key["api_key"].as<String>();
    } else if (readKey.isEmpty()) {
      readKey = key["api_key"].as<String>();
    }
  }
}

String tagsAsText(JsonVariantConst value) {
  String output;
  for (JsonVariantConst tag : value.as<JsonArrayConst>()) {
    const String name = tag.is<JsonObjectConst>()
                            ? tag["name"].as<String>()
                            : tag.as<String>();
    if (name.isEmpty()) {
      continue;
    }
    if (!output.isEmpty()) {
      output += ", ";
    }
    output += name;
  }
  return output;
}

bool validOptionalDecimal(const String& text, double minimum,
                          double maximum) {
  if (text.isEmpty()) {
    return true;
  }
  char* end = nullptr;
  const double value = strtod(text.c_str(), &end);
  return end != text.c_str() && *end == '\0' && isfinite(value) &&
         value >= minimum && value <= maximum;
}

bool validManagedChannelRequest(WebServer& server) {
  if (server.arg("name").isEmpty() || server.arg("name").length() > 80 ||
      server.arg("description").length() > 255 ||
      server.arg("metadata").length() > 512 ||
      server.arg("tags").length() > 160 || server.arg("url").length() > 200 ||
      !validOptionalDecimal(server.arg("latitude"), -90.0, 90.0) ||
      !validOptionalDecimal(server.arg("longitude"), -180.0, 180.0) ||
      !validOptionalDecimal(server.arg("elevation"), -10000.0, 100000.0)) {
    return false;
  }
  for (uint8_t field = 1; field <= 8; ++field) {
    if (server.arg("field" + String(field)).length() > 80) {
      return false;
    }
  }
  return true;
}

String managedChannelSettings(WebServer& server) {
  String body;
  body.reserve(900);
  appendFormValue(body, "name", server.arg("name"));
  appendFormValue(body, "description", server.arg("description"));
  appendFormValue(body, "metadata", server.arg("metadata"));
  appendFormValue(body, "tags", server.arg("tags"));
  appendFormValue(body, "url", server.arg("url"));
  appendFormValue(body, "latitude", server.arg("latitude"));
  appendFormValue(body, "longitude", server.arg("longitude"));
  appendFormValue(body, "elevation", server.arg("elevation"));
  appendFormValue(body, "public_flag",
                  server.hasArg("publicFlag") ? "true" : "false");
  for (uint8_t field = 1; field <= 8; ++field) {
    const String key = "field" + String(field);
    appendFormValue(body, key.c_str(), server.arg(key));
  }
  return body;
}

bool validThingSpeakFields(const ThingSpeakFieldMapping& fields,
                           bool requireOne) {
  const uint8_t values[] = {fields.temperature, fields.humidity,
                            fields.pressure, fields.iaq, fields.battery,
                            fields.gasResistance};
  constexpr size_t count = sizeof(values) / sizeof(values[0]);
  bool hasField = false;
  for (size_t i = 0; i < count; ++i) {
    if (values[i] > 8) {
      return false;
    }
    if (values[i] == 0) {
      continue;
    }
    hasField = true;
    for (size_t j = i + 1; j < count; ++j) {
      if (values[i] == values[j]) {
        return false;
      }
    }
  }
  return hasField || !requireOne;
}

void sendNoCache(WebServer& server) {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("X-Content-Type-Options", "nosniff");
  server.sendHeader("X-Frame-Options", "DENY");
  server.sendHeader("Referrer-Policy", "no-referrer");
  server.sendHeader(
      "Content-Security-Policy",
      "default-src 'self'; style-src 'self' 'unsafe-inline'; script-src "
      "'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; "
      "frame-ancestors 'none'; base-uri 'none'; form-action 'self'");
}
}  // namespace

bool WebPortal::begin(StationConfig& config, ConfigStore& store,
                      WifiService& wifi, SensorRegistry& registry,
                      ThingSpeakService& thingSpeak, EspNowGateway& gateway) {
  config_ = &config;
  store_ = &store;
  wifi_ = &wifi;
  registry_ = &registry;
  thingSpeak_ = &thingSpeak;
  gateway_ = &gateway;
  snprintf(csrfToken_, sizeof(csrfToken_), "%08lX%08lX",
           static_cast<unsigned long>(esp_random()),
           static_cast<unsigned long>(esp_random()));
  refreshAuthToken();
  const char* collectedHeaders[] = {"Cookie"};
  server_.collectHeaders(collectedHeaders, 1);

  server_.on("/", HTTP_GET, [this]() {
    sendNoCache(server_);
    if (config_->wifiSsid[0] != '\0' && !requireAuthentication(true)) {
      return;
    }
    server_.send_P(200, "text/html; charset=utf-8",
                   config_->wifiSsid[0] == '\0' ? kSetupWizardPage
                                                : kDashboardPage);
  });
  server_.on("/setup", HTTP_GET, [this]() {
    sendNoCache(server_);
    if (config_->wifiSsid[0] != '\0') {
      server_.sendHeader("Location", "/dashboard", true);
      server_.send(302, "text/plain", "");
      return;
    }
    server_.send_P(200, "text/html; charset=utf-8", kSetupWizardPage);
  });
  server_.on("/dashboard", HTTP_GET, [this]() {
    sendNoCache(server_);
    if (!requireAuthentication(true)) {
      return;
    }
    server_.send_P(200, "text/html; charset=utf-8", kDashboardPage);
  });
  server_.on("/login", HTTP_GET, [this]() { handleLogin(); });
  server_.on("/login", HTTP_POST, [this]() { handleLogin(); });
  server_.on("/logout", HTTP_GET, [this]() { handleLogout(); });
  server_.on("/api/status", HTTP_GET, [this]() {
    if (requireAuthentication()) sendJsonStatus();
  });
  server_.on("/api/history", HTTP_GET, [this]() {
    if (requireAuthentication()) sendJsonHistory();
  });
  server_.on("/api/config", HTTP_GET, [this]() {
    if (requireAuthentication()) sendJsonConfig();
  });
  server_.on("/api/wifi-scan", HTTP_GET, [this]() {
    if (requireAuthentication()) sendWifiScan();
  });
  server_.on("/api/setup", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf()) saveSetup();
  });
  server_.on("/api/setup-sensor", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf()) saveSetupSensor();
  });
  server_.on("/api/station", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf()) saveStation();
  });
  server_.on("/api/sensor", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf()) saveSensor();
  });
  server_.on("/api/channel-profile", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf()) saveChannelProfile();
  });
  server_.on("/api/delete-channel-profile", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf()) deleteChannelProfile();
  });
  server_.on("/api/create-channel", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf()) createThingSpeakChannel();
  });
  server_.on("/api/thingspeak/channels", HTTP_GET, [this]() {
    if (requireAuthentication()) listThingSpeakChannels();
  });
  server_.on("/api/thingspeak/channel-create", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf())
      createManagedThingSpeakChannel();
  });
  server_.on("/api/thingspeak/channel-update", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf())
      updateManagedThingSpeakChannel();
  });
  server_.on("/api/thingspeak/channel-clear", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf())
      clearManagedThingSpeakChannel();
  });
  server_.on("/api/thingspeak/channel-delete", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf())
      deleteManagedThingSpeakChannel();
  });
  server_.on("/api/reset-sensor", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf()) resetSensor();
  });
  server_.on("/api/reset-sensor-iaq", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf()) resetSensorIaqCalibration();
  });
  server_.on("/api/delete-sensor", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf()) deleteSensor();
  });
  server_.on("/api/factory-reset", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf()) factoryReset();
  });
  server_.on("/favicon.ico", HTTP_GET,
             [this]() { server_.send(204, "text/plain", ""); });

  const auto captiveRedirect = [this]() {
    sendNoCache(server_);
    server_.sendHeader("Connection", "close");
    server_.sendHeader("Location", "/", true);
    server_.send(302, "text/plain", "");
  };
  server_.on("/generate_204", HTTP_ANY, captiveRedirect);
  server_.on("/gen_204", HTTP_ANY, captiveRedirect);
  server_.on("/hotspot-detect.html", HTTP_ANY, captiveRedirect);
  server_.on("/library/test/success.html", HTTP_ANY, captiveRedirect);
  server_.on("/fwlink", HTTP_ANY, captiveRedirect);
  server_.on("/connecttest.txt", HTTP_ANY, captiveRedirect);
  server_.on("/redirect", HTTP_ANY, captiveRedirect);
  server_.on("/canonical.html", HTTP_ANY, captiveRedirect);
  server_.on("/success.txt", HTTP_ANY, captiveRedirect);
  server_.on("/ncsi.txt", HTTP_ANY, captiveRedirect);
  // Common HTTP-only connectivity probes. HTTPS probes cannot be redirected
  // safely because the device does not own their TLS certificates; users can
  // always open http://192.168.4.1/ directly in that case.
  server_.on("/connectivity-check.html", HTTP_ANY, captiveRedirect);
  server_.on("/check_network_status.txt", HTTP_ANY, captiveRedirect);
  server_.on("/portal.html", HTTP_ANY, captiveRedirect);
  server_.on("/success.html", HTTP_ANY, captiveRedirect);
  server_.on("/wifi/wifidog", HTTP_ANY, captiveRedirect);
  server_.on("/kindle-wifi/wifistub.html", HTTP_ANY, captiveRedirect);
  server_.on("/.well-known/captive-portal", HTTP_ANY, captiveRedirect);
  server_.onNotFound(captiveRedirect);
  server_.begin();
  return true;
}

bool WebPortal::authenticationEnabled() const {
  return config_ != nullptr && config_->adminPassword[0] != '\0';
}

bool WebPortal::isAuthenticated() const {
  if (!authenticationEnabled()) {
    return true;
  }
  String cookies = server_.header("Cookie");
  const String expected = String(kSessionCookieName) + '=' + authToken_;
  int start = 0;
  while (start < static_cast<int>(cookies.length())) {
    int end = cookies.indexOf(';', start);
    if (end < 0) {
      end = cookies.length();
    }
    String cookie = cookies.substring(start, end);
    cookie.trim();
    if (cookie == expected) {
      return true;
    }
    start = end + 1;
  }
  return false;
}

bool WebPortal::requireAuthentication(bool redirectToLogin) {
  if (isAuthenticated()) {
    return true;
  }
  sendNoCache(server_);
  if (redirectToLogin) {
    server_.sendHeader("Location", "/login", true);
    server_.send(302, "text/plain", "");
  } else {
    JsonDocument document;
    document["success"] = false;
    document["message"] = "Authentication required";
    document["authenticationRequired"] = true;
    String output;
    serializeJson(document, output);
    server_.send(401, "application/json", output);
  }
  return false;
}

bool WebPortal::verifyAdminPassword(const String& password) const {
  if (!authenticationEnabled() || password.length() > 64) {
    return false;
  }
  char candidate[65]{};
  hashAdminPassword(password, candidate);
  return constantTimeEquals(candidate, config_->adminPassword, 64);
}

void WebPortal::setAdminPassword(const String& password) {
  if (password.isEmpty()) {
    config_->adminPassword[0] = '\0';
    return;
  }
  hashAdminPassword(password, config_->adminPassword);
}

void WebPortal::refreshAuthToken() {
  snprintf(authToken_, sizeof(authToken_), "%08lX%08lX%08lX%08lX",
           static_cast<unsigned long>(esp_random()),
           static_cast<unsigned long>(esp_random()),
           static_cast<unsigned long>(esp_random()),
           static_cast<unsigned long>(esp_random()));
}

void WebPortal::handleLogin() {
  sendNoCache(server_);
  if (!authenticationEnabled()) {
    server_.sendHeader("Location", "/dashboard", true);
    server_.send(302, "text/plain", "");
    return;
  }
  if (server_.method() == HTTP_GET) {
    if (isAuthenticated()) {
      server_.sendHeader("Location", "/dashboard", true);
      server_.send(302, "text/plain", "");
      return;
    }
    server_.send_P(200, "text/html; charset=utf-8", kLoginPage);
    return;
  }

  if (loginBlockedUntilMs_ != 0 &&
      static_cast<int32_t>(millis() - loginBlockedUntilMs_) < 0) {
    JsonDocument document;
    document["success"] = false;
    document["message"] = "Too many attempts. Try again in a few seconds.";
    String output;
    serializeJson(document, output);
    server_.send(429, "application/json", output);
    return;
  }
  loginBlockedUntilMs_ = 0;
  if (!verifyAdminPassword(server_.arg("password"))) {
    ++failedLoginAttempts_;
    if (failedLoginAttempts_ >= kMaxLoginAttempts) {
      failedLoginAttempts_ = 0;
      loginBlockedUntilMs_ = millis() + kLoginBlockMs;
    }
    JsonDocument document;
    document["success"] = false;
    document["message"] = "Incorrect password";
    String output;
    serializeJson(document, output);
    server_.send(401, "application/json", output);
    return;
  }

  failedLoginAttempts_ = 0;
  server_.sendHeader(
      "Set-Cookie",
      String(kSessionCookieName) + '=' + authToken_ +
          "; Path=/; HttpOnly; SameSite=Strict");
  server_.send(200, "application/json", "{\"success\":true}");
}

void WebPortal::handleLogout() {
  sendNoCache(server_);
  server_.sendHeader(
      "Set-Cookie",
      String(kSessionCookieName) +
          "=; Path=/; Max-Age=0; HttpOnly; SameSite=Strict");
  server_.sendHeader("Location", authenticationEnabled() ? "/login"
                                                         : "/dashboard",
                     true);
  server_.send(303, "text/plain", "");
}

bool WebPortal::verifyCsrf() {
  if (server_.arg("csrf") == csrfToken_) {
    return true;
  }
  sendJsonResult(false, "Invalid CSRF token");
  return false;
}

void WebPortal::loop() {
  server_.handleClient();
  if (restartAtMs_ != 0 && static_cast<int32_t>(millis() - restartAtMs_) >= 0) {
    ESP.restart();
  }
}

void WebPortal::sendJsonStatus() {
  JsonDocument document;
  document["uptimeMs"] = millis();
  document["freeHeap"] = ESP.getFreeHeap();
  JsonObject wifi = document["wifi"].to<JsonObject>();
  wifi["connected"] = wifi_->stationConnected();
  wifi["ssid"] = wifi_->stationSsid();
  wifi["ip"] = wifi_->stationIp();
  wifi["apActive"] = wifi_->setupPortalActive();
  wifi["apIp"] = wifi_->accessPointIp();
  wifi["apSsid"] = wifi_->accessPointSsid();
  wifi["channel"] = wifi_->channel();
  wifi["hostname"] = "w-charger";

  JsonObject espnow = document["espnow"].to<JsonObject>();
  espnow["received"] = gateway_->receivedPackets();
  espnow["invalid"] = gateway_->invalidPackets();
  espnow["dropped"] = gateway_->droppedPackets();
  JsonObject cloud = document["cloud"].to<JsonObject>();
  cloud["lastHttpStatus"] = thingSpeak_->lastHttpStatus();
  cloud["lastEntryId"] = thingSpeak_->lastEntryId();
  cloud["successfulUploads"] = thingSpeak_->successfulUploads();
  cloud["failedUploads"] = thingSpeak_->failedUploads();
  cloud["lastAttemptAgeMs"] =
      thingSpeak_->lastAttemptMs() == 0
          ? 0
          : millis() - thingSpeak_->lastAttemptMs();
  cloud["lastSuccessAgeMs"] =
      thingSpeak_->lastSuccessMs() == 0
          ? 0
          : millis() - thingSpeak_->lastSuccessMs();
  cloud["droppedJobs"] = thingSpeak_->droppedJobs();

  SensorView sensorViews[kMaxSensors];
  const size_t count = registry_->views(sensorViews, kMaxSensors);
  size_t provisionedCount = 0;
  size_t pendingCount = 0;
  size_t uploadEnabledCount = 0;
  JsonArray sensors = document["sensors"].to<JsonArray>();
  for (size_t i = 0; i < count; ++i) {
    const SensorView& view = sensorViews[i];
    const uint32_t ageMs = view.runtime.hasTelemetry
                               ? millis() - view.runtime.lastSeenMs
                               : 0;
    const uint32_t onlineLimitMs =
        max(120000UL, view.config.sleepSeconds * 3000UL);
    JsonObject item = sensors.add<JsonObject>();
    item["mac"] = SensorRegistry::formatMac(view.config.mac);
    item["name"] = view.config.name;
    item["sleepSeconds"] = view.config.sleepSeconds;
    item["revision"] = view.config.revision;
    item["provisioned"] = view.config.provisioned;
    item["profileSlot"] = view.config.thingSpeakProfileSlot;
    item["channelId"] = view.config.thingSpeakChannelId;
    item["uploadEnabled"] = view.config.cloudUploadEnabled;
    item["writeKeySet"] = view.config.thingSpeakWriteKey[0] != '\0';
    item["temperatureField"] = view.config.thingSpeakFields.temperature;
    item["humidityField"] = view.config.thingSpeakFields.humidity;
    item["pressureField"] = view.config.thingSpeakFields.pressure;
    item["iaqField"] = view.config.thingSpeakFields.iaq;
    item["batteryField"] = view.config.thingSpeakFields.battery;
    item["gasResistanceField"] =
        view.config.thingSpeakFields.gasResistance;
    item["configuredSensorType"] =
        static_cast<uint8_t>(view.config.environmentalSensorType);
    item["sensorType"] =
        static_cast<uint8_t>(view.runtime.telemetry.sensorType);
    item["temperatureOffsetC"] = view.config.temperatureOffsetC;
    item["batteryCalibrationFactor"] =
        view.config.batteryCalibrationFactor;
    item["seen"] = view.runtime.hasTelemetry;
    item["restored"] = view.runtime.hasPersistedTelemetry;
    item["online"] = view.runtime.hasTelemetry && ageMs <= onlineLimitMs;
    item["ageMs"] = ageMs;
    item["rssi"] = view.runtime.stationRssi;
    item["temperature"] = view.runtime.telemetry.temperatureC;
    item["humidity"] = view.runtime.telemetry.humidityPercent;
    item["pressure"] = view.runtime.telemetry.pressureHpa;
    item["iaq"] = view.runtime.telemetry.iaq;
    item["iaqAccuracy"] = view.runtime.telemetry.iaqAccuracy;
    item["iaqCalibrationPhase"] = static_cast<uint8_t>(
        view.runtime.telemetry.iaqCalibrationPhase);
    item["iaqCalibrationElapsedMinutes"] =
        view.config.bme680QuickStartComplete
            ? max<uint16_t>(10,
                            view.runtime.telemetry.iaqCalibrationElapsedMinutes)
            : view.runtime.telemetry.iaqCalibrationElapsedMinutes;
    item["iaqCalibrationRemainingMinutes"] =
        view.runtime.telemetry.iaqCalibrationRemainingMinutes;
    item["bme680QuickStartComplete"] =
        view.config.bme680QuickStartComplete;
    item["gasResistanceOhms"] =
        view.runtime.telemetry.gasResistanceOhms;
    item["capabilities"] = view.runtime.telemetry.capabilities;
    item["telemetryFlags"] = view.runtime.telemetry.flags;
    item["batteryMv"] = view.runtime.telemetry.batteryMillivolts;
    item["pcbVersion"] = view.runtime.telemetry.pcbVersion;
    item["appliedRevision"] = view.runtime.telemetry.appliedConfigRevision;
    item["historyRevision"] = registry_->historyRevision(view.config.mac);
    if (view.config.provisioned) {
      ++provisionedCount;
      if (view.config.cloudUploadEnabled) {
        ++uploadEnabledCount;
      }
    } else {
      ++pendingCount;
    }
  }
  cloud["configuredSensors"] = provisionedCount;
  cloud["pendingSensors"] = pendingCount;
  cloud["uploadEnabledSensors"] = uploadEnabledCount;

  String output;
  serializeJson(document, output);
  sendNoCache(server_);
  server_.send(200, "application/json", output);
}

void WebPortal::sendJsonHistory() {
  uint8_t mac[6]{};
  if (!SensorRegistry::parseMac(server_.arg("mac"), mac)) {
    sendJsonResult(false, "Invalid sensor MAC address");
    return;
  }
  SensorConfig config{};
  if (!registry_->findConfig(mac, config)) {
    sendJsonResult(false, "Sensor not found");
    return;
  }

  HistorySample samples[kHistoryBucketCount]{};
  const size_t count =
      registry_->history(mac, samples, kHistoryBucketCount);
  JsonDocument document;
  document["mac"] = SensorRegistry::formatMac(mac);
  document["bucketMinutes"] = kHistoryBucketSeconds / 60UL;
  JsonArray points = document["points"].to<JsonArray>();
  for (size_t i = 0; i < count; ++i) {
    const HistorySample& sample = samples[i];
    JsonObject point = points.add<JsonObject>();
    point["t"] = static_cast<uint64_t>(sample.timestamp) * 1000ULL;
    point["sensorType"] = sample.sensorType;
    point["pcbVersion"] = sample.pcbVersion;
    point["iaqAccuracy"] = sample.iaqAccuracy;
    if ((sample.capabilities & lil::protocol::kTemperature) != 0) {
      point["temperature"] = sample.temperature;
    }
    if ((sample.capabilities & lil::protocol::kHumidity) != 0) {
      point["humidity"] = sample.humidity;
    }
    if ((sample.capabilities & lil::protocol::kPressure) != 0) {
      point["pressure"] = sample.pressure;
    }
    if ((sample.capabilities & lil::protocol::kIaq) != 0 &&
        sample.iaqAccuracy > 0) {
      point["iaq"] = sample.iaq;
    }
    if ((sample.capabilities & lil::protocol::kGasResistance) != 0) {
      point["gas"] = sample.gasKohms;
    }
    if ((sample.capabilities & lil::protocol::kBattery) != 0) {
      point["battery"] = sample.batteryVolts;
    }
  }

  String output;
  serializeJson(document, output);
  sendNoCache(server_);
  server_.send(200, "application/json", output);
}

void WebPortal::sendJsonConfig() {
  JsonDocument document;
  document["hostname"] = "w-charger";
  document["wifiSsid"] = config_->wifiSsid;
  document["wifiPasswordSet"] = config_->wifiPassword[0] != '\0';
  document["adminPasswordSet"] = authenticationEnabled();
  document["userApiKey"] = config_->thingSpeakUserApiKey;
  document["defaultSleepSeconds"] = config_->defaultSleepSeconds;
  document["fallbackChannel"] = config_->fallbackWifiChannel;
  document["apActive"] = wifi_->setupPortalActive();
  document["apSsid"] = wifi_->accessPointSsid();
  document["apIp"] = wifi_->accessPointIp();
  document["accessPointPassword"] = config_->accessPointPassword;
  document["csrfToken"] = csrfToken_;
  JsonArray channels = document["thingSpeakChannels"].to<JsonArray>();
  for (size_t i = 0; i < kMaxThingSpeakChannels; ++i) {
    const auto& profile = config_->thingSpeakChannels[i];
    if (!profile.occupied) {
      continue;
    }
    JsonObject item = channels.add<JsonObject>();
    item["slot"] = i;
    item["name"] = profile.name;
    item["channelId"] = profile.channelId;
    item["readApiKey"] = profile.readApiKey;
    item["writeApiKey"] = profile.writeApiKey;
  }
  String output;
  serializeJson(document, output);
  sendNoCache(server_);
  server_.send(200, "application/json", output);
}

void WebPortal::sendWifiScan() {
  if (server_.arg("refresh") == "1") {
    wifi_->requestNetworkScan();
  }
  WifiNetwork networks[kMaxScannedNetworks];
  const size_t count =
      wifi_->cachedNetworks(networks, kMaxScannedNetworks);
  JsonDocument document;
  document["scanning"] = wifi_->networkScanInProgress();
  JsonArray items = document["networks"].to<JsonArray>();
  for (size_t i = 0; i < count; ++i) {
    JsonObject item = items.add<JsonObject>();
    item["ssid"] = networks[i].ssid;
    item["rssi"] = networks[i].rssi;
    item["channel"] = networks[i].channel;
    item["secure"] = networks[i].secure;
  }
  String output;
  serializeJson(document, output);
  server_.send(200, "application/json", output);
}

void WebPortal::saveSetup() {
  if (config_->wifiSsid[0] != '\0') {
    sendJsonResult(false,
                   "The station is already configured. Use Settings.");
    return;
  }
  const String oldSsid = config_->wifiSsid;
  const String ssid = server_.arg("ssid");
  const String password = server_.arg("wifiPassword");
  const String userApiKey = server_.arg("userApiKey");
  const String channelName = server_.arg("channelName");
  const uint32_t channelId = server_.arg("defaultChannelId").toInt();
  const String readApiKey = server_.arg("readApiKey");
  const String writeApiKey = server_.arg("writeApiKey");
  const String adminPassword = server_.arg("adminPassword");
  const String confirmAdminPassword = server_.arg("confirmAdminPassword");
  const uint32_t defaultSleep = server_.arg("defaultSleep").toInt();
  if ((!adminPassword.isEmpty() &&
       (adminPassword.length() < 8 || adminPassword.length() > 64)) ||
      adminPassword != confirmAdminPassword) {
    sendJsonResult(false,
                   "The website passwords must match and contain 8 to 64 characters.");
    return;
  }
  if (ssid.isEmpty() || ssid.length() > 32 || password.length() > 64 ||
      userApiKey.length() > 40 || channelName.length() > 24 ||
      readApiKey.length() > 32 ||
      writeApiKey.length() > 32 || defaultSleep < kMinSleepSeconds ||
      defaultSleep > kMaxSleepSeconds ||
      (channelId == 0 && (!readApiKey.isEmpty() || !writeApiKey.isEmpty()))) {
    sendJsonResult(false, "Check Wi-Fi and the measurement interval.");
    return;
  }

  copyText(config_->hostname, "w-charger");
  copyText(config_->wifiSsid, ssid);
  if (!password.isEmpty() || ssid != oldSsid) {
    copyText(config_->wifiPassword, password);
  }
  copyText(config_->thingSpeakUserApiKey, userApiKey);
  config_->thingSpeakDefaultChannelId = channelId;
  copyText(config_->thingSpeakReadApiKey, readApiKey);
  copyText(config_->thingSpeakWriteApiKey, writeApiKey);
  auto& profile = config_->thingSpeakChannels[0];
  profile = ThingSpeakChannelProfile{};
  if (channelId != 0) {
    profile.occupied = true;
    copyText(profile.name,
             channelName.isEmpty() ? String("Home Sensor") : channelName);
    profile.channelId = channelId;
    copyText(profile.readApiKey, readApiKey);
    copyText(profile.writeApiKey, writeApiKey);
  }
  setAdminPassword(adminPassword);
  config_->defaultSleepSeconds = defaultSleep;
  // Keep the setup portal alive through this reboot.  WifiService clears it
  // and persists completion only after the home WLAN really connects.
  config_->setupPortalRequired = true;
  const bool profileSynced = channelId != 0
                                 ? registry_->syncThingSpeakProfile(0, profile)
                                 : registry_->removeThingSpeakProfile(0);
  const bool saved = store_->saveStationConfig(*config_) && profileSynced;
  sendJsonResult(saved,
                 saved ? "Setup saved. W-Charger is restarting."
                       : "Setup could not be saved.",
                 saved);
}

void WebPortal::saveSetupSensor() {
  uint8_t mac[6];
  SensorView view{};
  const String name = server_.arg("name");
  const uint32_t sleepSeconds = server_.arg("sleepSeconds").toInt();
  const auto sensorType =
      static_cast<lil::protocol::EnvironmentalSensorType>(
          server_.arg("sensorType").toInt());
  const bool validSensorType =
      sensorType == lil::protocol::EnvironmentalSensorType::kAutoDetect ||
      sensorType == lil::protocol::EnvironmentalSensorType::kBme280 ||
      sensorType == lil::protocol::EnvironmentalSensorType::kBme680 ||
      sensorType == lil::protocol::EnvironmentalSensorType::kDisabled;
  if (!wifi_->setupPortalActive() ||
      !SensorRegistry::parseMac(server_.arg("mac"), mac) ||
      !registry_->findView(mac, view) || view.config.provisioned) {
    sendJsonResult(false,
                   "The sensor could not be configured. Check the selection and connection.");
    return;
  }
  const bool bme680 =
      sensorType == lil::protocol::EnvironmentalSensorType::kBme680 ||
      (sensorType == lil::protocol::EnvironmentalSensorType::kAutoDetect &&
       view.runtime.telemetry.sensorType ==
           lil::protocol::EnvironmentalSensorType::kBme680);
  if (name.isEmpty() || name.length() > 24 || !validSensorType ||
      sleepSeconds < kMinSleepSeconds || sleepSeconds > kMaxSleepSeconds ||
      (bme680 && (sleepSeconds < 300 || sleepSeconds % 300 != 0))) {
    sendJsonResult(false,
                   "Sensor name, sensor type or interval is invalid.");
    return;
  }

  ThingSpeakFieldMapping fields{};
  if (bme680) {
    fields.iaq = 4;
    fields.gasResistance = 5;
    fields.battery = 6;
  } else if (sensorType == lil::protocol::EnvironmentalSensorType::kBme280) {
    fields.battery = 4;
  }
  const bool saved = registry_->updateConfig(
      mac, name, sleepSeconds, 0, String(), false, fields, 0xFF, sensorType,
      view.config.temperatureOffsetC, view.config.batteryCalibrationFactor);
  sendJsonResult(saved,
                 saved ? "Sensor added. Cloud settings and calibration can be completed later in the dashboard."
                       : "Sensor could not be saved.");
}

void WebPortal::saveStation() {
  const String oldSsid = config_->wifiSsid;
  const String oldAdminPasswordHash = config_->adminPassword;
  const uint32_t oldDefaultSleep = config_->defaultSleepSeconds;
  const uint8_t oldFallbackChannel = config_->fallbackWifiChannel;
  const String ssid = server_.arg("ssid");
  const String wifiPassword = server_.arg("wifiPassword");
  const String apPassword = server_.arg("apPassword");
  const String userApiKey = server_.arg("userApiKey");
  const String newAdminPassword = server_.arg("newAdminPassword");
  const String confirmAdminPassword = server_.arg("confirmAdminPassword");
  const bool removeAdminPassword = server_.hasArg("removeAdminPassword");
  const uint32_t defaultSleep = server_.arg("defaultSleep").toInt();
  const uint8_t fallbackChannel = server_.arg("fallbackChannel").toInt();
  if (ssid.isEmpty() || ssid.length() > 32 || wifiPassword.length() > 64 ||
      userApiKey.length() > 40 || defaultSleep < kMinSleepSeconds ||
      defaultSleep > kMaxSleepSeconds || fallbackChannel < 1 ||
      fallbackChannel > 13 ||
      (!apPassword.isEmpty() &&
       (apPassword.length() < 8 || apPassword.length() > 63)) ||
      (!newAdminPassword.isEmpty() &&
       (newAdminPassword.length() < 8 || newAdminPassword.length() > 64)) ||
      newAdminPassword != confirmAdminPassword ||
      (removeAdminPassword && !newAdminPassword.isEmpty())) {
    sendJsonResult(false, "Invalid station configuration");
    return;
  }

  copyText(config_->hostname, "w-charger");
  copyText(config_->wifiSsid, ssid);
  if (!wifiPassword.isEmpty() || ssid != oldSsid) {
    copyText(config_->wifiPassword, wifiPassword);
  }
  if (!apPassword.isEmpty()) {
    copyText(config_->accessPointPassword, apPassword);
  }
  copyText(config_->thingSpeakUserApiKey, userApiKey);
  if (removeAdminPassword) {
    setAdminPassword(String());
  } else if (!newAdminPassword.isEmpty()) {
    setAdminPassword(newAdminPassword);
  }
  config_->defaultSleepSeconds = defaultSleep;
  config_->fallbackWifiChannel = fallbackChannel;
  const bool saved = store_->saveStationConfig(*config_);
  const bool authenticationChanged =
      saved && oldAdminPasswordHash != config_->adminPassword;
  const bool restartRequested = server_.arg("restartNow") == "1";
  const bool restartRequired =
      saved && (ssid != oldSsid || !wifiPassword.isEmpty() ||
                defaultSleep != oldDefaultSleep ||
                fallbackChannel != oldFallbackChannel || restartRequested);
  if (authenticationChanged) {
    refreshAuthToken();
    server_.sendHeader(
        "Set-Cookie",
        String(kSessionCookieName) +
            "=; Path=/; Max-Age=0; HttpOnly; SameSite=Strict");
  }
  sendJsonResult(saved,
                 saved ? (restartRequired
                              ? "Network or measurement settings saved. W-Charger is restarting."
                              : "Settings saved and applied.")
                       : "Save failed",
                 restartRequired, authenticationChanged);
}

void WebPortal::saveSensor() {
  uint8_t mac[6];
  const uint32_t sleepSeconds = server_.arg("sleepSeconds").toInt();
  uint32_t channelId = server_.arg("channelId").toInt();
  String writeKey = server_.arg("writeKey");
  int profileIndex = -1;
  if (server_.hasArg("profileSlot")) {
    profileIndex = server_.arg("profileSlot").toInt();
  }
  uint8_t profileSlot = 0xFF;
  if (profileIndex >= 0 &&
      profileIndex < static_cast<int>(kMaxThingSpeakChannels) &&
      config_->thingSpeakChannels[profileIndex].occupied) {
    const auto& profile = config_->thingSpeakChannels[profileIndex];
    profileSlot = static_cast<uint8_t>(profileIndex);
    channelId = profile.channelId;
    writeKey = profile.writeApiKey;
  }
  const bool uploadEnabled = server_.hasArg("uploadEnabled");
  ThingSpeakFieldMapping fields{};
  fields.temperature = server_.arg("temperatureField").toInt();
  fields.humidity = server_.arg("humidityField").toInt();
  fields.pressure = server_.arg("pressureField").toInt();
  fields.iaq = server_.arg("iaqField").toInt();
  fields.battery = server_.arg("batteryField").toInt();
  fields.gasResistance = server_.arg("gasResistanceField").toInt();
  const uint32_t sensorTypeValue = server_.arg("sensorType").toInt();
  const auto sensorType =
      static_cast<lil::protocol::EnvironmentalSensorType>(sensorTypeValue);
  const float temperatureOffsetC = server_.arg("temperatureOffsetC").toFloat();
  const bool validSensorType =
      sensorType == lil::protocol::EnvironmentalSensorType::kAutoDetect ||
      sensorType == lil::protocol::EnvironmentalSensorType::kBme280 ||
      sensorType == lil::protocol::EnvironmentalSensorType::kBme680 ||
      sensorType == lil::protocol::EnvironmentalSensorType::kDisabled;
  SensorView existingView{};
  if (!SensorRegistry::parseMac(server_.arg("mac"), mac) ||
      !registry_->findView(mac, existingView) ||
      sleepSeconds < kMinSleepSeconds || sleepSeconds > kMaxSleepSeconds ||
      writeKey.length() > 32 || !validSensorType ||
      !isfinite(temperatureOffsetC) || temperatureOffsetC < -10.0F ||
      temperatureOffsetC > 10.0F ||
      (sensorType == lil::protocol::EnvironmentalSensorType::kBme680 &&
       (sleepSeconds < 300 || sleepSeconds % 300 != 0))) {
    sendJsonResult(false, "Invalid sensor configuration");
    return;
  }
  const SensorConfig& existing = existingView.config;
  const bool bme680Effective =
      sensorType == lil::protocol::EnvironmentalSensorType::kBme680 ||
      (sensorType == lil::protocol::EnvironmentalSensorType::kAutoDetect &&
       existingView.runtime.telemetry.sensorType ==
           lil::protocol::EnvironmentalSensorType::kBme680);
  if (bme680Effective && (sleepSeconds < 300 || sleepSeconds % 300 != 0)) {
    sendJsonResult(
        false,
        "A detected BME680 requires a five-minute interval or a multiple of five minutes.");
    return;
  }
  float batteryCalibrationFactor = existing.batteryCalibrationFactor;
  if (server_.hasArg("resetBatteryCalibration")) {
    batteryCalibrationFactor = 1.0F;
  } else {
    const String batteryReference = server_.arg("batteryReferenceVoltage");
    if (!batteryReference.isEmpty()) {
      char* end = nullptr;
      const float referenceVoltage = strtof(batteryReference.c_str(), &end);
      const uint16_t measuredMillivolts =
          existingView.runtime.telemetry.batteryMillivolts;
      if (end == batteryReference.c_str() ||
          *end != '\0' || !isfinite(referenceVoltage) ||
          referenceVoltage < 2.0F || referenceVoltage > 5.0F ||
          !existingView.runtime.hasTelemetry || measuredMillivolts < 2000 ||
          (existingView.runtime.telemetry.flags &
           lil::protocol::kBatteryReadFailed) != 0) {
        sendJsonResult(
            false,
            "Battery calibration failed. Check the reference voltage and latest measurement.");
        return;
      }
      batteryCalibrationFactor *=
          referenceVoltage * 1000.0F / measuredMillivolts;
      if (!isfinite(batteryCalibrationFactor) ||
          batteryCalibrationFactor < 0.7F ||
          batteryCalibrationFactor > 1.3F) {
        sendJsonResult(
            false,
            "The calculated correction factor is implausible. Check the PCB version, voltage divider and multimeter value.");
        return;
      }
    }
  }
  if (!validThingSpeakFields(fields, uploadEnabled)) {
    sendJsonResult(
        false,
        "Do not assign ThingSpeak fields 1–8 more than once. At least "
        "one measurement must be enabled.");
    return;
  }
  const bool storedKeyUsable = profileSlot == 0xFF &&
                               channelId == existing.thingSpeakChannelId &&
                               existing.thingSpeakWriteKey[0] != '\0';
  if (writeKey.isEmpty() && storedKeyUsable) {
    writeKey = existing.thingSpeakWriteKey;
  }
  if (uploadEnabled &&
      (channelId == 0 || writeKey.isEmpty())) {
    sendJsonResult(false,
                   "Cloud upload requires a Channel ID and Write API Key.");
    return;
  }
  const bool saved = registry_->updateConfig(
      mac, server_.arg("name"), sleepSeconds, channelId, writeKey,
      uploadEnabled, fields, profileSlot, sensorType, temperatureOffsetC,
      batteryCalibrationFactor);
  sendJsonResult(saved,
                 saved ? "Sensor settings will be sent at the next check-in."
                       : "Sensor not found");
}

void WebPortal::saveChannelProfile() {
  int slot = server_.arg("slot").toInt();
  if (!server_.hasArg("slot") || slot < 0 ||
      slot >= static_cast<int>(kMaxThingSpeakChannels)) {
    slot = -1;
    for (size_t i = 0; i < kMaxThingSpeakChannels; ++i) {
      if (!config_->thingSpeakChannels[i].occupied) {
        slot = static_cast<int>(i);
        break;
      }
    }
  }
  const String name = server_.arg("name");
  const uint32_t channelId = server_.arg("channelId").toInt();
  const String readApiKey = server_.arg("readApiKey");
  const String writeApiKey = server_.arg("writeApiKey");
  if (slot < 0 || channelId == 0 || name.length() > 24 ||
      readApiKey.length() > 32 || writeApiKey.length() > 32) {
    sendJsonResult(false,
                   slot < 0 ? "A maximum of six channels is supported."
                            : "Check the Channel ID and API keys.");
    return;
  }
  auto& profile = config_->thingSpeakChannels[slot];
  profile = ThingSpeakChannelProfile{};
  profile.occupied = true;
  copyText(profile.name,
           name.isEmpty() ? "Channel " + String(channelId) : name);
  profile.channelId = channelId;
  copyText(profile.readApiKey, readApiKey);
  copyText(profile.writeApiKey, writeApiKey);
  if (slot == 0) {
    config_->thingSpeakDefaultChannelId = channelId;
    copyText(config_->thingSpeakReadApiKey, readApiKey);
    copyText(config_->thingSpeakWriteApiKey, writeApiKey);
  }
  const bool saved = store_->saveStationConfig(*config_) &&
                     registry_->syncThingSpeakProfile(slot, profile);
  sendJsonResult(saved, saved ? "ThingSpeak channel saved."
                               : "Channel could not be saved.");
}

void WebPortal::deleteChannelProfile() {
  const int slot = server_.arg("slot").toInt();
  if (slot < 0 || slot >= static_cast<int>(kMaxThingSpeakChannels) ||
      !config_->thingSpeakChannels[slot].occupied) {
    sendJsonResult(false, "Channel not found.");
    return;
  }
  config_->thingSpeakChannels[slot] = ThingSpeakChannelProfile{};
  if (slot == 0) {
    config_->thingSpeakDefaultChannelId = 0;
    config_->thingSpeakReadApiKey[0] = '\0';
    config_->thingSpeakWriteApiKey[0] = '\0';
  }
  const bool saved = store_->saveStationConfig(*config_) &&
                     registry_->removeThingSpeakProfile(slot);
  sendJsonResult(saved, saved ? "ThingSpeak channel removed."
                               : "Channel could not be removed.");
}

void WebPortal::createThingSpeakChannel() {
  uint8_t mac[6];
  SensorConfig sensor{};
  if (!SensorRegistry::parseMac(server_.arg("mac"), mac) ||
      !registry_->findConfig(mac, sensor)) {
    sendJsonResult(false, "Sensor not found");
    return;
  }
  const String requestedName = server_.arg("name");
  if (requestedName.length() > 24) {
    sendJsonResult(false, "The sensor name is too long.");
    return;
  }
  const String channelName = requestedName.isEmpty()
                                 ? String(sensor.name)
                                 : requestedName;
  int profileSlot = -1;
  for (size_t i = 0; i < kMaxThingSpeakChannels; ++i) {
    if (!config_->thingSpeakChannels[i].occupied) {
      profileSlot = static_cast<int>(i);
      break;
    }
  }
  if (profileSlot < 0) {
    sendJsonResult(false, "A maximum of six channels is supported.");
    return;
  }
  const ChannelCreationResult result = thingSpeak_->createChannel(
      config_->thingSpeakUserApiKey, channelName);
  if (!result.success) {
    sendJsonResult(false, result.error);
    return;
  }
  auto& profile = config_->thingSpeakChannels[profileSlot];
  profile.occupied = true;
  copyText(profile.name, channelName);
  profile.channelId = result.channelId;
  copyText(profile.readApiKey, result.readKey);
  copyText(profile.writeApiKey, result.writeKey);
  if (profileSlot == 0) {
    config_->thingSpeakDefaultChannelId = result.channelId;
    copyText(config_->thingSpeakReadApiKey, result.readKey);
    copyText(config_->thingSpeakWriteApiKey, result.writeKey);
  }
  if (!store_->saveStationConfig(*config_)) {
    sendJsonResult(false,
                   "Channel created, but it could not be saved locally.");
    return;
  }

  JsonDocument document;
  document["success"] = true;
  document["message"] = "ThingSpeak-Channel " + String(result.channelId) +
                        " was created. Assign its fields next.";
  document["restart"] = false;
  document["slot"] = profileSlot;
  document["channelId"] = result.channelId;
  String output;
  serializeJson(document, output);
  sendNoCache(server_);
  server_.send(200, "application/json", output);
}

void WebPortal::listThingSpeakChannels() {
  const ThingSpeakApiResult result =
      thingSpeak_->listChannels(config_->thingSpeakUserApiKey);
  if (!result.success) {
    sendJsonResult(false, result.error);
    return;
  }

  JsonDocument remote;
  if (deserializeJson(remote, result.response) ||
      !remote.is<JsonArray>()) {
    sendJsonResult(false, "ThingSpeak returned an invalid channel list.");
    return;
  }

  JsonDocument document;
  document["success"] = true;
  document["message"] = "ThingSpeak channels loaded.";
  JsonArray channels = document["channels"].to<JsonArray>();
  for (JsonObjectConst source : remote.as<JsonArrayConst>()) {
    const uint32_t channelId = source["id"] | 0U;
    if (channelId == 0) {
      continue;
    }
    String readKey;
    String writeKey;
    extractApiKeys(source, readKey, writeKey);
    int localSlot = -1;
    for (size_t slot = 0; slot < kMaxThingSpeakChannels; ++slot) {
      if (config_->thingSpeakChannels[slot].occupied &&
          config_->thingSpeakChannels[slot].channelId == channelId) {
        localSlot = static_cast<int>(slot);
        if (readKey.isEmpty()) {
          readKey = config_->thingSpeakChannels[slot].readApiKey;
        }
        if (writeKey.isEmpty()) {
          writeKey = config_->thingSpeakChannels[slot].writeApiKey;
        }
        break;
      }
    }
    JsonObject item = channels.add<JsonObject>();
    item["id"] = channelId;
    item["name"] = source["name"] | "Unnamed channel";
    item["description"] = source["description"] | "";
    item["metadata"] = source["metadata"] | "";
    item["tags"] = tagsAsText(source["tags"]);
    item["url"] = source["url"] | "";
    item["latitude"] = source["latitude"] | "";
    item["longitude"] = source["longitude"] | "";
    item["elevation"] = source["elevation"] | "";
    item["publicFlag"] = source["public_flag"] | false;
    item["lastEntryId"] = source["last_entry_id"] | 0U;
    item["createdAt"] = source["created_at"] | "";
    item["updatedAt"] = source["updated_at"] | "";
    item["readApiKey"] = readKey;
    item["writeApiKey"] = writeKey;
    item["localSlot"] = localSlot;
    item["channelUrl"] =
        "https://thingspeak.com/channels/" + String(channelId);
    item["privateUrl"] = "https://thingspeak.com/channels/" +
                         String(channelId) + "/private_show";
    item["feedUrl"] = "https://api.thingspeak.com/channels/" +
                      String(channelId) + "/feeds.json";
    JsonArray fields = item["fields"].to<JsonArray>();
    for (uint8_t field = 1; field <= 8; ++field) {
      fields.add(source["field" + String(field)] | "");
    }
  }
  String output;
  serializeJson(document, output);
  sendNoCache(server_);
  server_.send(200, "application/json", output);
}

void WebPortal::createManagedThingSpeakChannel() {
  if (!validManagedChannelRequest(server_)) {
    sendJsonResult(false, "Check the name and channel fields.");
    return;
  }
  const ThingSpeakApiResult result = thingSpeak_->createManagedChannel(
      config_->thingSpeakUserApiKey, managedChannelSettings(server_));
  if (!result.success) {
    sendJsonResult(false, result.error);
    return;
  }
  JsonDocument remote;
  if (deserializeJson(remote, result.response)) {
    sendJsonResult(false, "Channel created, but the response was invalid.");
    return;
  }
  const uint32_t channelId = remote["id"] | 0U;
  String readKey;
  String writeKey;
  extractApiKeys(remote.as<JsonObjectConst>(), readKey, writeKey);
  int localSlot = -1;
  for (size_t slot = 0; slot < kMaxThingSpeakChannels; ++slot) {
    if (!config_->thingSpeakChannels[slot].occupied) {
      localSlot = static_cast<int>(slot);
      break;
    }
  }
  if (localSlot >= 0 && channelId != 0) {
    auto& profile = config_->thingSpeakChannels[localSlot];
    profile.occupied = true;
    copyText(profile.name, server_.arg("name"));
    profile.channelId = channelId;
    copyText(profile.readApiKey, readKey);
    copyText(profile.writeApiKey, writeKey);
    if (localSlot == 0) {
      config_->thingSpeakDefaultChannelId = channelId;
      copyText(config_->thingSpeakReadApiKey, readKey);
      copyText(config_->thingSpeakWriteApiKey, writeKey);
    }
    if (!store_->saveStationConfig(*config_)) {
      sendJsonResult(false,
                     "Channel created, but the local profile could not be saved.");
      return;
    }
  }
  JsonDocument document;
  document["success"] = channelId != 0;
  document["restart"] = false;
  document["message"] = localSlot >= 0
                            ? "ThingSpeak channel created and imported locally."
                            : "ThingSpeak channel created. The local profile list is full.";
  document["channelId"] = channelId;
  document["slot"] = localSlot;
  String output;
  serializeJson(document, output);
  sendNoCache(server_);
  server_.send(channelId != 0 ? 200 : 400, "application/json", output);
}

void WebPortal::updateManagedThingSpeakChannel() {
  const uint32_t channelId = server_.arg("channelId").toInt();
  if (channelId == 0 || !validManagedChannelRequest(server_)) {
    sendJsonResult(false, "Check the Channel ID, name and fields.");
    return;
  }
  const ThingSpeakApiResult result = thingSpeak_->updateChannel(
      config_->thingSpeakUserApiKey, channelId,
      managedChannelSettings(server_));
  if (!result.success) {
    sendJsonResult(false, result.error);
    return;
  }
  JsonDocument remote;
  String readKey;
  String writeKey;
  if (!deserializeJson(remote, result.response)) {
    extractApiKeys(remote.as<JsonObjectConst>(), readKey, writeKey);
  }
  bool localChanged = false;
  bool localSaved = true;
  for (size_t slot = 0; slot < kMaxThingSpeakChannels; ++slot) {
    auto& profile = config_->thingSpeakChannels[slot];
    if (!profile.occupied || profile.channelId != channelId) {
      continue;
    }
    copyText(profile.name, server_.arg("name"));
    if (!readKey.isEmpty()) {
      copyText(profile.readApiKey, readKey);
    }
    if (!writeKey.isEmpty()) {
      copyText(profile.writeApiKey, writeKey);
    }
    localChanged = true;
    localSaved = registry_->syncThingSpeakProfile(slot, profile) && localSaved;
  }
  if (localChanged) {
    localSaved = store_->saveStationConfig(*config_) && localSaved;
  }
  sendJsonResult(localSaved,
                 localSaved ? "ThingSpeak channel updated."
                            : "Cloud updated, but the local assignment could not be saved.");
}

void WebPortal::clearManagedThingSpeakChannel() {
  const uint32_t channelId = server_.arg("channelId").toInt();
  if (channelId == 0) {
    sendJsonResult(false, "Invalid Channel ID.");
    return;
  }
  const ThingSpeakApiResult result = thingSpeak_->clearChannel(
      config_->thingSpeakUserApiKey, channelId);
  sendJsonResult(result.success,
                 result.success ? "All channel measurements were cleared."
                                : result.error);
}

void WebPortal::deleteManagedThingSpeakChannel() {
  const uint32_t channelId = server_.arg("channelId").toInt();
  if (channelId == 0) {
    sendJsonResult(false, "Invalid Channel ID.");
    return;
  }
  const ThingSpeakApiResult result = thingSpeak_->deleteChannel(
      config_->thingSpeakUserApiKey, channelId);
  if (!result.success) {
    sendJsonResult(false, result.error);
    return;
  }
  bool saved = true;
  bool changed = false;
  for (size_t slot = 0; slot < kMaxThingSpeakChannels; ++slot) {
    auto& profile = config_->thingSpeakChannels[slot];
    if (!profile.occupied || profile.channelId != channelId) {
      continue;
    }
    profile = ThingSpeakChannelProfile{};
    if (slot == 0) {
      config_->thingSpeakDefaultChannelId = 0;
      config_->thingSpeakReadApiKey[0] = '\0';
      config_->thingSpeakWriteApiKey[0] = '\0';
    }
    changed = true;
    saved = registry_->removeThingSpeakProfile(slot) && saved;
  }
  if (changed) {
    saved = store_->saveStationConfig(*config_) && saved;
  }
  sendJsonResult(saved,
                 saved ? "ThingSpeak channel and local assignment deleted."
                       : "Cloud channel deleted, but the local assignment could not be fully updated.");
}

void WebPortal::resetSensor() {
  uint8_t mac[6];
  const bool ok = SensorRegistry::parseMac(server_.arg("mac"), mac) &&
                  registry_->requestFactoryReset(mac);
  sendJsonResult(ok, ok ? "The sensor will be reset at its next check-in."
                        : "Sensor not found");
}

void WebPortal::resetSensorIaqCalibration() {
  uint8_t mac[6];
  const bool ok = SensorRegistry::parseMac(server_.arg("mac"), mac) &&
                  registry_->requestIaqCalibrationReset(mac);
  sendJsonResult(
      ok,
      ok ? "The IAQ learning state will be cleared at the next check-in, then calibration will restart."
         : "Sensor not found");
}

void WebPortal::deleteSensor() {
  uint8_t mac[6];
  const bool ok = SensorRegistry::parseMac(server_.arg("mac"), mac) &&
                  registry_->deleteSensor(mac);
  sendJsonResult(
      ok,
      ok ? "Sensor removed from the station. A powered sensor will be detected again at its next check-in."
         : "Sensor not found");
}

void WebPortal::factoryReset() {
  if (server_.arg("confirm") != "RESET") {
    sendJsonResult(false, "Confirmation does not match");
    return;
  }
  const bool cleared = store_->factoryReset();
  sendJsonResult(cleared,
                 cleared
                     ? "Factory settings cleared. W-Charger is restarting."
                     : "Factory settings could not be cleared.",
                 cleared);
}

void WebPortal::sendJsonResult(bool success, const String& message,
                               bool restartRequired,
                               bool authenticationChanged) {
  JsonDocument document;
  document["success"] = success;
  document["message"] = message;
  document["restart"] = restartRequired;
  document["authenticationChanged"] = authenticationChanged;
  String output;
  serializeJson(document, output);
  sendNoCache(server_);
  server_.send(success ? 200 : 400, "application/json", output);
  if (restartRequired) {
    scheduleRestart();
  }
}

void WebPortal::scheduleRestart() {
  restartAtMs_ = millis() + 2500;
}

}  // namespace station

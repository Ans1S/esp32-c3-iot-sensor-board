#include "web_portal.h"
#include "filesystem_guard.h"
#include "recording_json.h"
#include "ota_service.h"
#include "ota_page.h"
#include "recording_archive.h"
#include "sensor_timing.h"
#include <memory>
#include <new>

#include <ArduinoJson.h>
#include <esp_heap_caps.h>
#include <esp_random.h>
#include <esp_timer.h>
#ifdef ESP_PLATFORM
#include <esp_task_wdt.h>
#endif
#include <mbedtls/sha256.h>

#include "web_pages.h"
#include "web_input.h"
#include "history_json.h"
#include "accept_encoding.h"
#include "dashboard_gzip.h"
#include "setup_gzip.h"

namespace station {

namespace {
constexpr size_t kMaxScannedNetworks = 20;
constexpr char kSessionCookieName[] = "wch_session";
constexpr char kPasswordHashDomain[] = "W-Charger website password v1";
constexpr uint64_t kSessionLifetimeMs = 30ULL * 60ULL * 1000ULL;
constexpr uint32_t kSessionLifetimeSeconds = kSessionLifetimeMs / 1000UL;

bool readUnsignedArgument(WebServer& server, const char* name,
                          uint32_t& value, uint32_t minimum = 0,
                          uint32_t maximum = UINT32_MAX,
                          bool optional = false) {
  if (!server.hasArg(name)) return optional;
  return webinput::unsignedDecimal(server.arg(name).c_str(), value,
                                  minimum, maximum);
}

bool readFieldArgument(WebServer& server, const char* name, uint8_t& field) {
  uint32_t value = 0;
  if (!readUnsignedArgument(server, name, value, 0, 8, true)) return false;
  field = static_cast<uint8_t>(value);
  return true;
}

struct PsramJsonAllocator : ArduinoJson::Allocator {
  void* allocate(size_t size) override {
    void* ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return ptr ? ptr : malloc(size);
  }
  void deallocate(void* pointer) override { free(pointer); }
  void* reallocate(void* ptr, size_t newSize) override {
    void* resized =
        heap_caps_realloc(ptr, newSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return resized ? resized : realloc(ptr, newSize);
  }
};

PsramJsonAllocator* psramJsonAllocator() {
  static PsramJsonAllocator allocator;
  return &allocator;
}

// Stream an ArduinoJson document directly through WebServer::sendContent with
// exact Content-Length, zero large internal-DRAM String allocation, and
// Arduino-ESP32's built-in non-blocking lwIP retry loop (no premature socket
// drops on transient TCP send-window pressure).
class BufferedJsonClientWriter {
 public:
  explicit BufferedJsonClientWriter(WebServer& server, size_t expectedBytes = SIZE_MAX)
      : server_(server) {
    const size_t wanted = min(expectedBytes, kHeapBufBytes);
    if (wanted > sizeof(stackBuf_)) {
      heapBuf_ = static_cast<uint8_t*>(heap_caps_malloc(wanted, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    }
    buf_ = heapBuf_ ? heapBuf_ : stackBuf_;
    cap_ = heapBuf_ ? wanted : sizeof(stackBuf_);
  }
  ~BufferedJsonClientWriter() {
    if (heapBuf_) heap_caps_free(heapBuf_);
  }
  size_t write(uint8_t byte) {
    if (failed_) return 0;
    buf_[used_++] = byte;
    return used_ < cap_ || flush() ? 1 : 0;
  }
  size_t write(const uint8_t* bytes, size_t size) {
    size_t written = 0;
    while (!failed_ && written < size) {
      const size_t n = min(size - written, cap_ - used_);
      memcpy(buf_ + used_, bytes + written, n);
      used_ += n;
      written += n;
      if (used_ == cap_ && !flush()) break;
    }
    return written;
  }
  bool flush() {
    if (failed_ || used_ == 0) {
      used_ = 0;
      return !failed_;
    }
#ifdef ESP_PLATFORM
    (void)esp_task_wdt_reset();
#endif
    if (!server_.client().connected()) {
      failed_ = true;
      used_ = 0;
      return false;
    }
    server_.sendContent(reinterpret_cast<const char*>(buf_), used_);
    used_ = 0;
    if (!server_.client().connected()) {
      failed_ = true;
      return false;
    }
    return true;
  }

 private:
  static constexpr size_t kHeapBufBytes = 2920;  // 2 * 1460-byte TCP MSS.
  WebServer& server_;
  uint8_t* heapBuf_ = nullptr;
  uint8_t* buf_ = nullptr;
  size_t cap_ = 0;
  size_t used_ = 0;
  bool failed_ = false;
  uint8_t stackBuf_[256];
};

void sendStreamedJsonDocument(WebServer& server, const JsonDocument& document) {
  const size_t length = measureJson(document);
  server.client().setNoDelay(true);
  server.setContentLength(length);
  server.send(200, "application/json", "");
  BufferedJsonClientWriter writer(server, length);
  serializeJson(document, writer);
  (void)writer.flush();
  server.setContentLength(CONTENT_LENGTH_NOT_SET);
}

// Allocate rollback snapshots off loopTask's stack so config handlers save
// ~1 KiB of stack headroom during NVS/flash writes.
std::unique_ptr<StationConfig> cloneStationConfig(const StationConfig& source) {
  std::unique_ptr<StationConfig> copy(new (std::nothrow) StationConfig(source));
  return copy;
}

void hashAdminPassword(const String& password, char output[65]) {
  static constexpr char kHex[] = "0123456789abcdef";
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  const uint64_t deviceId = ESP.getEfuseMac();
  mbedtls_sha256_update(&ctx,
                        reinterpret_cast<const uint8_t*>(kPasswordHashDomain),
                        strlen(kPasswordHashDomain));
  mbedtls_sha256_update(&ctx, reinterpret_cast<const uint8_t*>(&deviceId),
                        sizeof(deviceId));
  mbedtls_sha256_update(&ctx, reinterpret_cast<const uint8_t*>(password.c_str()),
                        password.length());
  uint8_t digest[32]{};
  mbedtls_sha256_finish(&ctx, digest);
  mbedtls_sha256_free(&ctx);
  for (size_t i = 0; i < sizeof(digest); ++i) {
    output[i * 2] = kHex[digest[i] >> 4U];
    output[i * 2 + 1] = kHex[digest[i] & 0x0FU];
  }
  output[64] = '\0';
}

bool constantTimeEquals(const char* first, const char* second,
                        size_t length) {
  uint8_t difference = 0;
  for (size_t i = 0; i < length; ++i) {
    difference |= static_cast<uint8_t>(first[i] ^ second[i]);
  }
  return difference == 0;
}

void randomToken(char output[33]) {
  snprintf(output, 33, "%08lX%08lX%08lX%08lX",
           static_cast<unsigned long>(esp_random()),
           static_cast<unsigned long>(esp_random()),
           static_cast<unsigned long>(esp_random()),
           static_cast<unsigned long>(esp_random()));
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
  server.sendHeader("Permissions-Policy",
                    "camera=(), microphone=(), geolocation=()");
  server.sendHeader("Cross-Origin-Resource-Policy", "same-origin");
  server.sendHeader(
      "Content-Security-Policy",
      "default-src 'self'; style-src 'self' 'unsafe-inline'; script-src "
      "'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; "
      "object-src 'none'; frame-ancestors 'none'; base-uri 'none'; "
      "form-action 'self'");
}

template <size_t N>
void sendPage(WebServer& server, const char* plain, const unsigned char (&gzip)[N]) {
  server.sendHeader("Vary", "Accept-Encoding");
  if (acceptsDashboardGzip(server.header("Accept-Encoding").c_str())) {
    server.sendHeader("Content-Encoding", "gzip");
    server.send_P(200, "text/html; charset=utf-8",
                  reinterpret_cast<const char*>(gzip), N);
  } else {
    server.send_P(200, "text/html; charset=utf-8", plain);
  }
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
  refreshCsrfToken();
  clearAuthSessions();
  dashboardLayout_ = store_->loadDashboardLayout();
  dashboardLayoutLoaded_ = true;
  const char* collectedHeaders[] = {"Cookie", "Origin", "Accept-Encoding"};
  server_.collectHeaders(collectedHeaders, 3);
  server_.enableDelay(false);

  server_.on("/", HTTP_GET, [this]() {
    if (config_->wifiSsid[0] != '\0' && !requireAuthentication(true)) {
      return;
    }
    sendNoCache(server_);
    if (config_->wifiSsid[0] == '\0') {
      sendPage(server_, kSetupWizardPage, kSetupWizardGzip);
    } else {
      sendPage(server_, kDashboardPage, kDashboardGzip);
    }
  });
  server_.on("/setup", HTTP_GET, [this]() {
    sendNoCache(server_);
    if (config_->wifiSsid[0] != '\0') {
      server_.sendHeader("Location", "/dashboard", true);
      server_.send(302, "text/plain", "");
      return;
    }
    sendPage(server_, kSetupWizardPage, kSetupWizardGzip);
  });
  server_.on("/dashboard", HTTP_GET, [this]() {
    if (!requireAuthentication(true)) {
      return;
    }
    sendNoCache(server_);
    sendPage(server_, kDashboardPage, kDashboardGzip);
  });
  server_.on("/login", HTTP_GET, [this]() { handleLogin(); });
  server_.on("/login", HTTP_POST, [this]() { handleLogin(); });
  server_.on("/logout", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf()) handleLogout();
  });
  server_.on("/api/status", HTTP_GET, [this]() {
    if (!requireAuthentication()) return;
    if (server_.hasArg("reconnect")) {
      JsonDocument doc;
      doc["wifi"]["connected"] = wifi_->stationConnected();
      sendNoCache(server_);
      sendStreamedJsonDocument(server_, doc);
    } else if (server_.hasArg("setupSensors")) sendDiscoveryStatus();
    else sendJsonStatus();
  });
  server_.on("/api/ota", HTTP_GET, [this]() {
    if (!requireAuthentication()) return;
    JsonDocument doc;
    otaService.json(doc.to<JsonObject>());
    String body; serializeJson(doc, body);
    sendNoCache(server_); server_.send(200, "application/json", body);
  });
  server_.on("/updates", HTTP_GET, [this]() {
    if (!requireAuthentication(true)) return;
    sendNoCache(server_); server_.send_P(200, "text/html; charset=utf-8", kOtaPage);
  });
  server_.on("/api/ota/cancel", HTTP_POST, [this]() {
    if (!requireAuthentication() || !verifyCsrf()) return;
    const bool cancelled = otaService.cancel();
    sendJsonResult(cancelled, cancelled
                       ? "Update cancellation saved; a reboot already initiated cannot be cancelled"
                       : "No pending update could be cancelled");
  });
  server_.on("/api/ota/upload", HTTP_POST, [this]() {
    const bool seen = otaUploadSeen_;
    const bool authorized = otaUploadAuthorized_;
    bool ok = otaUploadOk_;
    const bool ended = otaUploadEnded_;
    String error = otaUploadError_;
    otaUploadSeen_ = false;
    otaUploadAuthorized_ = false;
    otaUploadOk_ = false;
    otaUploadEnded_ = false;
    otaUploadError_ = "";
    if (!seen) {
      if (!requireAuthentication() || !verifyCsrf()) return;
      sendJsonResult(false, "No firmware file received");
      return;
    }
    if (!authorized) return;
    if (ok && ended) {
      ok = otaService.finishUpload(error);
    } else if (ok) {
      otaService.abortUpload();
      ok = false;
      error = "Upload incomplete";
    }
    sendJsonResult(ok, ok ? "Update queued for the next node contact" : error);
  }, [this]() {
#ifdef ESP_PLATFORM
    (void)esp_task_wdt_reset();
#endif
    HTTPUpload& upload = server_.upload();
    if (upload.status == UPLOAD_FILE_START) {
      if (otaUploadSeen_) {
        if (otaUploadAuthorized_ && otaUploadOk_) otaService.abortUpload();
        otaUploadOk_ = false;
        otaUploadEnded_ = false;
        otaUploadError_ = "Only one firmware file may be uploaded at a time";
        return;
      }
      otaUploadSeen_ = true;
      otaUploadEnded_ = false;
      otaUploadAuthorized_ = requireAuthentication() && verifyCsrf();
      otaUploadOk_ = false; otaUploadError_ = "Upload failed";
      if (!otaUploadAuthorized_) return;
      uint8_t mac[6];
      if (!SensorRegistry::parseMac(server_.arg("mac"), mac)) {
        otaUploadError_ = "Invalid node address"; return;
      }
      SensorConfig node{};
      if (!registry_->findConfig(mac, node) || !node.provisioned) {
        otaUploadError_ = "Select a configured node"; return;
      }
      otaUploadOk_ = otaService.startUpload(mac, otaUploadError_);
    } else if (upload.status == UPLOAD_FILE_WRITE && otaUploadAuthorized_ && otaUploadOk_) {
      if (!otaService.upload(upload.buf, upload.currentSize)) {
        otaUploadOk_ = false; otaUploadError_ = "Firmware exceeds capacity or flash write failed";
        otaService.abortUpload();
      }
    } else if (upload.status == UPLOAD_FILE_END && otaUploadAuthorized_ && otaUploadOk_) {
      otaUploadEnded_ = true;
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
      if (otaUploadAuthorized_ && otaUploadOk_) otaService.abortUpload();
      otaUploadOk_ = false;
      otaUploadEnded_ = false;
      otaUploadError_ = "Upload aborted";
    }
  });
  server_.on("/api/history", HTTP_GET, [this]() {
    if (requireAuthentication()) sendJsonHistory();
  });
  server_.on("/api/recordings", HTTP_GET, [this]() { if (requireAuthentication()) sendRecordings(); });
  server_.on("/api/recording", HTTP_GET, [this]() { if (requireAuthentication()) sendRecording(); });
  server_.on("/api/motion-reference", HTTP_POST, [this]() { if (requireAuthentication() && verifyCsrf()) setMotionReference(); });
  server_.on("/api/recording/delete", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf()) deleteRecording();
  });
  server_.on("/api/config", HTTP_GET, [this]() {
    if (requireAuthentication()) sendJsonConfig();
  });
  server_.on("/api/dashboard-layout",HTTP_POST,[this](){if(requireAuthentication()&&verifyCsrf())saveDashboardLayout();});
  server_.on("/api/wifi-scan", HTTP_GET, [this]() {
    if (requireAuthentication()) sendWifiScan();
  });
  server_.on("/api/setup", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf()) saveSetup();
  });
  server_.on("/api/enrollment", HTTP_POST, [this]() {
    if (!requireAuthentication() || !verifyCsrf()) return;
    registry_->openEnrollmentWindow();
    sendJsonResult(true, "Listening for nearby sensors.");
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
  server_.on("/api/thingspeak/channels", HTTP_POST, [this]() {
    if (requireAuthentication() && verifyCsrf()) startThingSpeakChannelList();
  });
  server_.on("/api/thingspeak/operation", HTTP_GET, [this]() {
    if (requireAuthentication()) pollThingSpeakMutation();
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
  server_.on("/favicon.ico", HTTP_GET, [this]() {
    sendNoCache(server_);
    server_.send(204, "text/plain", "");
  });

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
  server_.onNotFound([this, captiveRedirect]() {
    const bool apiPath = server_.uri().startsWith("/api/");
    if (apiPath ||
        (config_->wifiSsid[0] != '\0' && !wifi_->setupPortalActive())) {
      sendNoCache(server_);
      server_.send(404, apiPath ? "application/json" : "text/plain",
                   apiPath
                       ? "{\"success\":false,\"message\":\"Not found\"}"
                       : "Not found");
      return;
    }
    captiveRedirect();
  });
  server_.begin();
  return true;
}

bool WebPortal::authenticationEnabled() const {
  return config_ != nullptr && config_->adminPassword[0] != '\0';
}

int WebPortal::findActiveSessionIndex(bool pruneExpired) {
  const uint64_t now = lil::timing::signedUsToMs(esp_timer_get_time());
  if (pruneExpired) {
    for (AuthSession& session : authSessions_) {
      if (session.active && now - session.createdAtMs >= kSessionLifetimeMs) {
        session = AuthSession{};
      }
    }
  }
  const String cookies = server_.header("Cookie");
  const String prefix = String(kSessionCookieName) + '=';
  int start = 0;
  while (start < static_cast<int>(cookies.length())) {
    int end = cookies.indexOf(';', start);
    if (end < 0) {
      end = cookies.length();
    }
    String cookie = cookies.substring(start, end);
    cookie.trim();
    if (cookie.startsWith(prefix)) {
      const String token = cookie.substring(prefix.length());
      if (token.length() == 32) {
        for (size_t i = 0; i < 4; ++i) {
          if (authSessions_[i].active &&
              now - authSessions_[i].createdAtMs < kSessionLifetimeMs &&
              constantTimeEquals(token.c_str(), authSessions_[i].token, 32)) {
            return static_cast<int>(i);
          }
        }
      }
    }
    start = end + 1;
  }
  return -1;
}

String WebPortal::currentSessionToken() {
  const int activeIndex = findActiveSessionIndex(false);
  if (activeIndex >= 0) {
    return String(authSessions_[activeIndex].token);
  }
  const String cookies = server_.header("Cookie");
  const String prefix = String(kSessionCookieName) + '=';
  int start = 0;
  while (start < static_cast<int>(cookies.length())) {
    int end = cookies.indexOf(';', start);
    if (end < 0) {
      end = cookies.length();
    }
    String cookie = cookies.substring(start, end);
    cookie.trim();
    if (cookie.startsWith(prefix)) {
      return cookie.substring(prefix.length());
    }
    start = end + 1;
  }
  return String();
}

bool WebPortal::isAuthenticated() {
  if (!authenticationEnabled()) {
    return true;
  }
  return findActiveSessionIndex(true) >= 0;
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

void WebPortal::refreshCsrfToken() { randomToken(csrfToken_); }

const char* WebPortal::createAuthSession() {
  const uint64_t now = lil::timing::signedUsToMs(esp_timer_get_time());
  size_t selected = 0;
  uint64_t oldestAge = 0;
  for (size_t i = 0; i < 4; ++i) {
    AuthSession& session = authSessions_[i];
    if (session.active && now - session.createdAtMs >= kSessionLifetimeMs) {
      session = AuthSession{};
    }
    if (!session.active) {
      selected = i;
      break;
    }
    const uint64_t age = now - session.createdAtMs;
    if (i == 0 || age > oldestAge) {
      selected = i;
      oldestAge = age;
    }
  }
  AuthSession& session = authSessions_[selected];
  randomToken(session.token);
  session.createdAtMs = now;
  session.active = true;
  return session.token;
}

void WebPortal::clearAuthSessions() {
  for (AuthSession& session : authSessions_) {
    session = AuthSession{};
  }
}

void WebPortal::revokeCurrentSession() {
  const int index = findActiveSessionIndex(true);
  if (index >= 0) {
    authSessions_[index] = AuthSession{};
  }
}

bool WebPortal::loginBlocked(uint32_t& retryAfterSeconds) {
  return loginThrottle_.blocked(lil::timing::signedUsToMs(esp_timer_get_time()),
                                retryAfterSeconds);
}

bool WebPortal::recordFailedLogin(uint32_t& retryAfterSeconds) {
  return loginThrottle_.recordFailure(lil::timing::signedUsToMs(esp_timer_get_time()),
                                      retryAfterSeconds);
}

void WebPortal::clearLoginFailures() {
  loginThrottle_ = LoginThrottle{};
}

bool WebPortal::validRequestOrigin() const {
  const String origin = server_.header("Origin");
  if (origin.isEmpty()) {
    return true;
  }
  String host = server_.hostHeader();
  if (host.isEmpty()) {
    return false;
  }
  auto normalizeDefaultPort = [](const String& value, const char* scheme,
                                 const char* defaultPort) -> String {
    if (value.startsWith(scheme)) {
      const String suffix(defaultPort);
      if (value.endsWith(suffix)) {
        return value.substring(0, value.length() - suffix.length());
      }
    }
    return value;
  };
  String originLower = origin;
  originLower.toLowerCase();
  String hostLower = host;
  hostLower.toLowerCase();
  const String normalizedOrigin =
      originLower.startsWith("https://")
          ? normalizeDefaultPort(originLower, "https://", ":443")
          : normalizeDefaultPort(originLower, "http://", ":80");
  String httpHost = "http://" + hostLower;
  String httpsHost = "https://" + hostLower;
  httpHost = normalizeDefaultPort(httpHost, "http://", ":80");
  httpsHost = normalizeDefaultPort(httpsHost, "https://", ":443");
  return normalizedOrigin == httpHost || normalizedOrigin == httpsHost;
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

  if (!validRequestOrigin()) {
    JsonDocument document;
    document["success"] = false;
    document["message"] = "Request origin is not allowed";
    String output;
    serializeJson(document, output);
    server_.send(403, "application/json", output);
    return;
  }

  uint32_t retryAfterSeconds = 0;
  if (loginBlocked(retryAfterSeconds)) {
    server_.sendHeader("Retry-After", String(retryAfterSeconds));
    JsonDocument document;
    document["success"] = false;
    document["message"] = "Too many attempts. Try again in a few seconds.";
    String output;
    serializeJson(document, output);
    server_.send(429, "application/json", output);
    return;
  }
  if (!verifyAdminPassword(server_.arg("password"))) {
    const bool nowBlocked = recordFailedLogin(retryAfterSeconds);
    JsonDocument document;
    document["success"] = false;
    document["message"] = nowBlocked
                              ? "Too many attempts. Try again in 30 seconds."
                              : "Incorrect password";
    String output;
    serializeJson(document, output);
    if (nowBlocked) {
      server_.sendHeader("Retry-After", String(retryAfterSeconds));
    }
    server_.send(nowBlocked ? 429 : 401, "application/json", output);
    return;
  }

  clearLoginFailures();
  bool anyActiveBefore = false;
  const uint64_t now = lil::timing::signedUsToMs(esp_timer_get_time());
  for (const AuthSession& session : authSessions_) {
    if (session.active && now - session.createdAtMs < kSessionLifetimeMs) {
      anyActiveBefore = true;
      break;
    }
  }
  const char* sessionToken = createAuthSession();
  if (!anyActiveBefore) {
    refreshCsrfToken();
  }
  server_.sendHeader(
      "Set-Cookie",
      String(kSessionCookieName) + '=' + sessionToken +
          "; Path=/; Max-Age=" + String(kSessionLifetimeSeconds) +
          "; HttpOnly; SameSite=Strict");
  server_.send(200, "application/json", "{\"success\":true}");
}

void WebPortal::handleLogout() {
  sendNoCache(server_);
  revokeCurrentSession();
  bool anyActiveAfter = false;
  const uint64_t now = lil::timing::signedUsToMs(esp_timer_get_time());
  for (const AuthSession& session : authSessions_) {
    if (session.active && now - session.createdAtMs < kSessionLifetimeMs) {
      anyActiveAfter = true;
      break;
    }
  }
  if (!anyActiveAfter) {
    refreshCsrfToken();
  }
  server_.sendHeader(
      "Set-Cookie",
      String(kSessionCookieName) +
          "=; Path=/; Max-Age=0; HttpOnly; SameSite=Strict");
  server_.send(200, "application/json", "{\"success\":true}");
}

bool WebPortal::verifyCsrf() {
  if (!validRequestOrigin()) {
    sendNoCache(server_);
    server_.send(
        403, "application/json",
        "{\"success\":false,\"message\":\"Request origin is not allowed\"}");
    return false;
  }
  const String token = server_.arg("csrf");
  if (token.length() == 32 && constantTimeEquals(token.c_str(), csrfToken_, 32)) {
    return true;
  }
  sendNoCache(server_);
  server_.send(403, "application/json",
               "{\"success\":false,\"message\":\"Invalid CSRF token\"}");
  return false;
}

void WebPortal::loop() {
  finishThingSpeakMutation();
  server_.handleClient();
  if (otaUploadSeen_) {
    if (otaUploadAuthorized_ && otaUploadOk_) otaService.abortUpload();
    otaUploadSeen_ = false;
    otaUploadAuthorized_ = false;
    otaUploadOk_ = false;
    otaUploadEnded_ = false;
    otaUploadError_ = "";
  }
  if (restartPending_ && restartDelay_.ready(millis())) {
    if (gateway_ != nullptr) {
      gateway_->pauseForReset();
    }
    if (registry_ != nullptr) {
      registry_->flushAllHistory();
    }
    FilesystemGuard fsGuard;
    ESP.restart();
  }
}

void WebPortal::sendDiscoveryStatus() {
  DiscoveryView rows[kMaxSensors];
  const size_t count = registry_->discoveryViews(rows, kMaxSensors);
  JsonDocument doc(psramJsonAllocator());
  doc["capacity"] = kMaxSensors;
  auto sensors = doc["sensors"].to<JsonArray>();
  for (size_t i = 0; i < count; ++i) {
    const auto& row = rows[i];
    auto item = sensors.add<JsonObject>();
    item["mac"] = SensorRegistry::formatMac(row.mac);
    item["name"] = row.name;
    item["sensorType"] = row.sensorType;
    item["pcbVersion"] = row.pcbVersion;
    item["operatingMode"] = row.operatingMode;
    item["provisioned"] = row.provisioned;
    if (row.provisioned) {
      item["configuredSensorType"] = row.configuredType;
      item["sleepSeconds"] = row.sleepSeconds;
    }
    item["confirmed"] = row.confirmed;
    item["measured"] = row.measured;
    if (row.resetPending) item["resetPending"] = true;
  }
  sendNoCache(server_);
  if (doc.overflowed()) {
    server_.send(503, "application/json", "{\"success\":false}");
    return;
  }
  sendStreamedJsonDocument(server_, doc);
}

void WebPortal::sendJsonStatus() {
  JsonDocument document(psramJsonAllocator());
  document["capacity"] = kMaxSensors;
  document["storageAvailable"] = store_->storageAvailable();
  document["uptimeMs"] = lil::timing::signedUsToMs(esp_timer_get_time());
  document["freeHeap"] = ESP.getFreeHeap();
  JsonObject memory = document["memory"].to<JsonObject>();
  constexpr uint32_t internalCaps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
  constexpr uint32_t externalCaps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
  memory["internalFreeBytes"] = heap_caps_get_free_size(internalCaps);
  memory["internalMinimumFreeBytes"] = heap_caps_get_minimum_free_size(internalCaps);
  memory["internalLargestBlockBytes"] = heap_caps_get_largest_free_block(internalCaps);
  memory["psramFreeBytes"] = heap_caps_get_free_size(externalCaps);
  memory["psramLargestBlockBytes"] = heap_caps_get_largest_free_block(externalCaps);
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
  espnow["persistenceDrops"] = gateway_->persistenceDrops();
  espnow["peerFailures"] = gateway_->peerFailures();
  espnow["replyBusyDrops"] = gateway_->replyBusyDrops();
  espnow["replyMacFailures"] = gateway_->replyMacFailures();
  JsonObject cloud = document["cloud"].to<JsonObject>();
  cloud["lastHttpStatus"] = thingSpeak_->lastHttpStatus();
  cloud["lastEntryId"] = thingSpeak_->lastEntryId();
  cloud["lastUploadFailed"] = thingSpeak_->lastUploadFailed();
  cloud["successfulUploads"] = thingSpeak_->successfulUploads();
  cloud["failedUploads"] = thingSpeak_->failedUploads();
  cloud["lastAttemptAgeMs"] = thingSpeak_->lastAttemptAgeMs();
  cloud["lastSuccessAgeMs"] = thingSpeak_->lastSuccessAgeMs();
  cloud["droppedJobs"] = thingSpeak_->droppedJobs();

  // WebServer handlers are serialized; allocate the view snapshot buffer lazily
  // in PSRAM (with internal-RAM fallback) to keep 7.8 KiB out of internal DRAM.
  if (sensorViews_ == nullptr) {
    void* raw = heap_caps_malloc(kMaxSensors * sizeof(SensorView),
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!raw) {
      raw = heap_caps_malloc(kMaxSensors * sizeof(SensorView), MALLOC_CAP_8BIT);
    }
    if (!raw) {
      sendNoCache(server_);
      server_.send(503, "application/json",
                   "{\"success\":false,\"message\":\"Status buffer unavailable\"}");
      return;
    }
    sensorViews_ = new (raw) SensorView[kMaxSensors]();
  }
  const size_t count = registry_->views(sensorViews_, kMaxSensors);
  const uint64_t snapshotMs = lil::timing::signedUsToMs(esp_timer_get_time());
  auto* history = registry_->normalHistory();
  size_t provisionedCount = 0;
  size_t pendingCount = 0;
  size_t uploadEnabledCount = 0;
  JsonArray sensors = document["sensors"].to<JsonArray>();
  for (size_t i = 0; i < count; ++i) {
    const SensorView& view = sensorViews_[i];
    const bool batteryProtection =
        (view.runtime.telemetry.flags & lil::protocol::kBatteryProtectionActive) != 0;
    const bool recordingActive = (view.runtime.hasRecordingStatus &&
        view.runtime.recording.state == lil::recording::State::Recording) ||
        (view.runtime.hasTelemetry &&
         lil::protocol::isLiveSensor(view.runtime.telemetry.sensorType) &&
         (view.runtime.telemetry.operatingMode == lil::protocol::SensorOperatingMode::kContinuousMotion ||
          view.runtime.telemetry.operatingMode == lil::protocol::SensorOperatingMode::kContinuousPrecision));
    const bool periodicPrecision =
        (view.runtime.telemetry.sensorType == lil::protocol::EnvironmentalSensorType::kLsm6dsox ||
         view.runtime.telemetry.sensorType == lil::protocol::EnvironmentalSensorType::kTmp117) &&
        !recordingActive;
    const uint64_t ageMs = (view.runtime.hasTelemetry &&
                            snapshotMs >= view.runtime.lastSeenMonotonicMs)
                               ? snapshotMs - view.runtime.lastSeenMonotonicMs
                               : 0;
    const uint32_t onlineLimitMs =
        batteryProtection ? 26UL * 60UL * 60UL * 1000UL :
        lil::protocol::isLiveSensor(view.runtime.telemetry.sensorType) && !periodicPrecision ? 15000UL :
        max(view.config.sleepSeconds < 60 ? 5000UL : 120000UL, view.config.sleepSeconds * 3000UL);
    JsonObject item = sensors.add<JsonObject>();
    item["mac"] = SensorRegistry::formatMac(view.config.mac);
    item["name"] = view.config.name;
    uint32_t historyRevision = 0;
    if(history){
      const auto info=history->info(view.config.mac);
      historyRevision = info.revision;
      item["normalHistory"] = !recordingActive &&
          view.config.environmentalSensorType != lil::protocol::EnvironmentalSensorType::kMax30102 &&
          view.runtime.telemetry.sensorType != lil::protocol::EnvironmentalSensorType::kMax30102;
      JsonObject retention=item["history"].to<JsonObject>();
      retention["points"]=info.points;retention["durablePoints"]=info.durablePoints;
      retention["pendingPoints"]=info.pendingPoints;retention["capacityPoints"]=info.capacityPoints;
      retention["oldest"]=info.oldest;retention["newest"]=info.newest;
      retention["intervalSeconds"]=info.intervalSeconds;retention["writeFailed"]=info.writeFailed;
      retention["dropped"]=info.dropped;retention["available"]=info.available;
    }
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
    item["configPending"] = !commissioningConfirmed(view.config, view.runtime);
    item["resetPending"] = (view.config.pendingFlags & lil::protocol::kFactoryReset) != 0;
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
    item["sensorTxPowerDbm"] =
        static_cast<float>(view.runtime.sensorTxPowerQuarterDbm) / 4.0F;
    const auto& timing = view.runtime.telemetry.live;
    item["timingKnown"] = view.runtime.hasTelemetry &&
                          (timing.flags & lil::protocol::kLiveTimingKnown) != 0;
    item["acquisitionAgeMs"] =
        (!view.runtime.hasTelemetry || timing.acquisitionAgeMs == UINT16_MAX)
            ? -1
            : int64_t(timing.acquisitionAgeMs) + int64_t(ageMs);
    item["estimateAgeMs"] =
        (!view.runtime.hasTelemetry || timing.estimateAgeMs == UINT16_MAX)
            ? -1
            : int64_t(timing.estimateAgeMs) + int64_t(ageMs);
    // Materialize packed wire fields before ArduinoJson binds its argument.
    item["measurementWindowMs"] = uint16_t(timing.windowMs);
    item["warmupMs"] = uint16_t(timing.warmupMs);
    item["processingMs"] = uint16_t(timing.processingMs);
    item["receivedIntervalMs"] = view.runtime.receivedIntervalMs;
    if (view.runtime.hasRecordingStatus && !batteryProtection) {
      const auto& recording = view.runtime.recording;
      auto r = item["recording"].to<JsonObject>();
      r["state"] = static_cast<uint8_t>(recording.state);
      r["pending"] = uint32_t(recording.pending); r["capacity"] = uint32_t(recording.capacity);
      r["elapsedMs"] = uint32_t(recording.elapsedMs); r["dropped"] = uint32_t(recording.dropped);
      char session[17]; snprintf(session, sizeof(session), "%016llx", static_cast<unsigned long long>(recording.session));
      r["session"] = session;
    }
    const auto& pulse = view.runtime.telemetry.pulse;
    item["heartRate"] = (view.runtime.telemetry.capabilities & lil::protocol::kHeartRate) ? pulse.beatsPerMinute : 0;
    item["red"] = uint32_t(pulse.red); item["infrared"] = uint32_t(pulse.infrared);
    item["pulseQuality"] = pulse.quality; item["pulseStatus"] = pulse.status;
    item["live"] = !batteryProtection && lil::protocol::isLiveSensor(view.runtime.telemetry.sensorType);
    item["batteryProtection"] = batteryProtection;
    item["liveRevision"] = view.runtime.receivedPackets;
    item["normalSampling"] = periodicPrecision;
    item["reportIntervalMs"] = batteryProtection ? 86400000UL : lil::protocol::isLiveSensor(view.runtime.telemetry.sensorType) && !periodicPrecision ?
        (view.runtime.telemetry.sensorType == lil::protocol::EnvironmentalSensorType::kLsm6dsox && recordingActive ?
            lil::timing::imuRecordingIntervalMs(view.runtime.recording.elapsedMs) :
            lil::protocol::liveReportIntervalMs(view.runtime.telemetry.sensorType)) : view.config.sleepSeconds * 1000;
    const auto& motion = view.runtime.telemetry.motion;
    auto reference = item["motionReference"].to<JsonObject>();
    reference["enabled"] = view.motionReference.enabled;
    const char* accelerationKeys[] = {"ax", "ay", "az"};
    const char* angularKeys[] = {"gx", "gy", "gz"};
    for (size_t axis = 0; axis < 3; ++axis) {
      item[accelerationKeys[axis]] = float(motion.accelerationG[axis]);
      item[angularKeys[axis]] = float(motion.angularRateDps[axis]);
      reference[accelerationKeys[axis]] = view.motionReference.accelerationG[axis];
      reference[angularKeys[axis]] = view.motionReference.angularRateDps[axis];
    }
    item["peakAcceleration"] = float(motion.peakAccelerationG);
    item["peakAngularRate"] = float(motion.peakAngularRateDps);
    item["motionSamples"] = uint16_t(motion.sampleCount);
    item["motionOverrun"] = motion.fifoOverrun != 0;
    item["temperature"] = float(view.runtime.telemetry.temperatureC);
    item["humidity"] = float(view.runtime.telemetry.humidityPercent);
    item["pressure"] = float(view.runtime.telemetry.pressureHpa);
    item["iaq"] = float(view.runtime.telemetry.iaq);
    item["iaqAccuracy"] = view.runtime.telemetry.iaqAccuracy;
    item["iaqCalibrationPhase"] = static_cast<uint8_t>(
        view.runtime.telemetry.iaqCalibrationPhase);
    item["iaqCalibrationElapsedMinutes"] =
        uint16_t(view.runtime.telemetry.iaqCalibrationElapsedMinutes);
    item["iaqCalibrationRemainingMinutes"] =
        uint16_t(view.runtime.telemetry.iaqCalibrationRemainingMinutes);
    item["bme680QuickStartComplete"] =
        view.config.bme680QuickStartComplete;
    item["gasResistanceOhms"] =
        float(view.runtime.telemetry.gasResistanceOhms);
    if (view.runtime.telemetry.capabilities & lil::protocol::kSpectral) {
      JsonArray sp = item["spectral"].to<JsonArray>();
      for (size_t ch = 0; ch < 10; ++ch) {
        sp.add(uint16_t(view.runtime.telemetry.extended.spectral.channels[ch]));
      }
      item["spectralGain"] = uint8_t(view.runtime.telemetry.extended.spectral.gainCode);
      item["spectralIntMs"] = uint16_t(view.runtime.telemetry.extended.spectral.integrationMs);
      item["spectralStatus"] = uint8_t(view.runtime.telemetry.extended.spectral.status);
    }
    if (view.runtime.telemetry.capabilities & lil::protocol::kDistance) {
      item["distanceMm"] = uint16_t(view.runtime.telemetry.extended.distance.distanceMm);
      item["signalRateKcps"] = uint16_t(view.runtime.telemetry.extended.distance.signalRateKcps);
      item["ambientRateKcps"] = uint16_t(view.runtime.telemetry.extended.distance.ambientRateKcps);
      item["rangeStatus"] = uint8_t(view.runtime.telemetry.extended.distance.rangeStatus);
      item["distanceMode"] = uint8_t(view.runtime.telemetry.extended.distance.mode);
    } else if (view.runtime.telemetry.sensorType ==
               lil::protocol::EnvironmentalSensorType::kVl53l1x) {
      item["rangeStatus"] = uint8_t(view.runtime.telemetry.extended.distance.rangeStatus);
    }
    if (view.runtime.telemetry.capabilities & lil::protocol::kAirQualityGas) {
      item["eco2"] = uint16_t(view.runtime.telemetry.extended.airQuality.eco2Ppm);
      item["tvoc"] = uint16_t(view.runtime.telemetry.extended.airQuality.tvocPpb);
      item["ccsRawCurrent"] = uint8_t(view.runtime.telemetry.extended.airQuality.rawCurrentUa);
      item["ccsRawAdc"] = uint16_t(view.runtime.telemetry.extended.airQuality.rawAdc);
      item["ccsStatus"] = uint8_t(view.runtime.telemetry.extended.airQuality.status);
      item["ccsError"] = uint8_t(view.runtime.telemetry.extended.airQuality.errorId);
    } else if (view.runtime.telemetry.sensorType ==
               lil::protocol::EnvironmentalSensorType::kCcs811) {
      item["ccsStatus"] = uint8_t(view.runtime.telemetry.extended.airQuality.status);
      item["ccsError"] = uint8_t(view.runtime.telemetry.extended.airQuality.errorId);
    }
    item["capabilities"] = uint16_t(view.runtime.telemetry.capabilities);
    item["telemetryFlags"] = uint16_t(view.runtime.telemetry.flags);
    item["batteryMv"] = uint16_t(view.runtime.telemetry.batteryMillivolts);
    item["pcbVersion"] = view.runtime.telemetry.pcbVersion;
    item["operatingMode"] = static_cast<uint8_t>(
        (view.runtime.hasTelemetry || view.runtime.hasPersistedTelemetry)
            ? view.runtime.telemetry.operatingMode
            : lil::protocol::SensorOperatingMode::kUnknown);
    item["appliedRevision"] = uint32_t(view.runtime.telemetry.appliedConfigRevision);
    item["historyRevision"] = history ? historyRevision : registry_->historyRevision(view.config.mac);
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

  if (document.overflowed()) {
    sendNoCache(server_);
    server_.send(503, "application/json",
                 "{\"success\":false,\"message\":\"Status memory temporarily unavailable\"}");
    return;
  }
  sendNoCache(server_);
  sendStreamedJsonDocument(server_, document);
}


namespace {
uint64_t recordingSession(const String& text) {
  if (text.length() != 16) return 0;
  for (size_t i = 0; i < text.length(); ++i) if (!isxdigit(static_cast<unsigned char>(text[i]))) return 0;
  return strtoull(text.c_str(), nullptr, 16);
}

}
void WebPortal::sendRecordings() {
  uint8_t mac[6]{}; SensorConfig config{};
  if (!SensorRegistry::parseMac(server_.arg("mac"), mac) || !registry_->findConfig(mac, config)) {
    sendJsonResult(false, "Sensor not found"); return;
  }
  JsonDocument doc(psramJsonAllocator()); doc["freeBytes"] = recordingArchive.freeBytes();
  auto sessions = doc["sessions"].to<JsonArray>();
  for (const auto& info : recordingArchive.list(mac)) recordingInfoJson(sessions.add<JsonObject>(), info);
  if (doc.overflowed()) { sendJsonResult(false, "Recording list memory unavailable"); return; }
  sendNoCache(server_);
  sendStreamedJsonDocument(server_, doc);
}
void WebPortal::sendRecording() {
  uint8_t mac[6]{}; SensorConfig config{};
  const uint64_t session = recordingSession(server_.arg("session"));
  if (!session || !SensorRegistry::parseMac(server_.arg("mac"), mac) || !registry_->findConfig(mac, config)) {
    sendJsonResult(false, "Invalid recording"); return;
  }
  uint32_t requestedCapacity = 128;
  if (!readUnsignedArgument(server_, "limit", requestedCapacity, 1, 512, true)) {
    sendJsonResult(false, "Invalid recording page size"); return;
  }
  size_t capacity = requestedCapacity;
  auto* records = static_cast<lil::recording::Record*>(heap_caps_calloc(
      capacity, sizeof(lil::recording::Record), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!records) {
    capacity = min(capacity, size_t(128));
    if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) >
        capacity * sizeof(lil::recording::Record) + 96U * 1024U)
      records = static_cast<lil::recording::Record*>(heap_caps_calloc(
          capacity, sizeof(lil::recording::Record), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  }
  if (!records) { sendJsonResult(false, "Recording buffer unavailable"); return; }
  uint32_t offset = 0, fromMs = UINT32_MAX;
  if (!readUnsignedArgument(server_, "offset", offset, 0, UINT32_MAX, true) ||
      !readUnsignedArgument(server_, "fromMs", fromMs, 0, UINT32_MAX, true)) {
    heap_caps_free(records); sendJsonResult(false, "Invalid recording cursor"); return;
  }
  RecordingInfo info{};
  const size_t count = recordingArchive.read(mac, session, offset, records, capacity, info, fromMs);
  if (!info.session) { heap_caps_free(records); sendJsonResult(false, "Recording not found"); return; }
  if (info.readError) { heap_caps_free(records); sendJsonResult(false, "Stored recording could not be read. Retry before exporting."); return; }
  JsonDocument metadata; recordingInfoJson(metadata.to<JsonObject>(), info);
  metadata["nextOffset"] = offset;
  if (metadata.overflowed()) { heap_caps_free(records); sendJsonResult(false, "Recording metadata unavailable"); return; }
  String opening; serializeJson(metadata, opening);
  if (opening.length() <= 1) { heap_caps_free(records); sendJsonResult(false, "Recording metadata unavailable"); return; }
  opening.remove(opening.length()-1);
  opening += ",\"points\":[";
  size_t chunkCap = 16384;
  size_t chunkFlush = 11680;  // 8 * 1460-byte TCP MSS.
  char* cb = static_cast<char*>(
      heap_caps_malloc(chunkCap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!cb) {
    chunkCap = 4096;
    chunkFlush = 2920;
    cb = static_cast<char*>(
        heap_caps_malloc(chunkCap, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  }
  if (!cb) {
    heap_caps_free(records);
    sendJsonResult(false, "Recording response buffer unavailable");
    return;
  }
  sendNoCache(server_); server_.client().setNoDelay(true); server_.setContentLength(CONTENT_LENGTH_UNKNOWN); server_.send(200, "application/json", "");
  server_.sendContent(opening);
  size_t clen = 0;
  cb[0] = '\0';
  for (size_t i = 0; i < count; ++i) {
    const size_t savedLen = clen;
    bool appended = (!i || appendRawChar(cb, chunkCap, clen, ',')) &&
                    appendRecordingPointJson(cb, chunkCap, clen, records[i]);
    if (!appended && savedLen > 0) {
#ifdef ESP_PLATFORM
      (void)esp_task_wdt_reset();
#endif
      if (!server_.client().connected()) {
        heap_caps_free(cb);
        heap_caps_free(records);
        server_.setContentLength(CONTENT_LENGTH_NOT_SET);
        return;
      }
      server_.sendContent(cb, savedLen);
      clen = 0;
      cb[0] = '\0';
      appended = (!i || appendRawChar(cb, chunkCap, clen, ',')) &&
                 appendRecordingPointJson(cb, chunkCap, clen, records[i]);
    }
    if (!appended) {
      heap_caps_free(cb);
      heap_caps_free(records);
      server_.client().stop();
      server_.setContentLength(CONTENT_LENGTH_NOT_SET);
      return;
    }
    if (clen >= chunkFlush) {
#ifdef ESP_PLATFORM
      (void)esp_task_wdt_reset();
#endif
      if (!server_.client().connected()) {
        heap_caps_free(cb);
        heap_caps_free(records);
        server_.setContentLength(CONTENT_LENGTH_NOT_SET);
        return;
      }
      server_.sendContent(cb, clen);
      clen = 0;
      cb[0] = '\0';
    }
  }
  heap_caps_free(records);
  if (!appendLit(cb, chunkCap, clen, "]}")) {
    heap_caps_free(cb);
    server_.client().stop();
    server_.setContentLength(CONTENT_LENGTH_NOT_SET);
    return;
  }
  if (server_.client().connected()) {
    server_.sendContent(cb, clen);
    server_.sendContent("");
  }
  heap_caps_free(cb);
  server_.setContentLength(CONTENT_LENGTH_NOT_SET);
}
void WebPortal::deleteRecording() {
  uint8_t mac[6]{}; SensorView view{};
  const uint64_t session = recordingSession(server_.arg("session"));
  if (!session || !SensorRegistry::parseMac(server_.arg("mac"), mac) || !registry_->findView(mac, view)) {
    sendJsonResult(false, "Invalid recording"); return;
  }
  // A partial session still being replayed cannot be removed underneath ACKs.
  if (view.runtime.hasRecordingStatus && view.runtime.recording.session == session &&
      view.runtime.recording.pending) { sendJsonResult(false, "Wait for synchronization or start a replacement recording on the sensor"); return; }
  const bool removed = recordingArchive.remove(mac, session);
  sendJsonResult(removed, removed ? "Recording deleted" : "Recording could not be deleted");
}

void WebPortal::setMotionReference() {
  uint8_t mac[6]{};
  const bool clear = server_.arg("clear") == "1";
  const bool saved = SensorRegistry::parseMac(server_.arg("mac"), mac) && registry_->setMotionReference(mac, clear);
  sendJsonResult(saved, saved ? (clear ? "Motion reference cleared." : "All six axes are now relative to this resting position.") :
      "Could not set the reference. Keep the sensor still on the table and wait for a fresh motion reading.");
}

void WebPortal::sendJsonHistory() {
  uint32_t afterMs = 0, since = 0;
  if (!readUnsignedArgument(server_, "afterMs", afterMs, 0, UINT32_MAX, true) ||
      !readUnsignedArgument(server_, "since", since, 0, UINT32_MAX, true)) {
    sendJsonResult(false, "Invalid history cursor"); return;
  }
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

  SensorView liveView{};
  const bool hasLiveView = registry_->findView(mac, liveView);
  const bool isMax30102 =
      config.environmentalSensorType == lil::protocol::EnvironmentalSensorType::kMax30102 ||
      (hasLiveView &&
       liveView.runtime.telemetry.sensorType == lil::protocol::EnvironmentalSensorType::kMax30102);
  if (registry_->normalHistory() && server_.arg("source") != "live" && !isMax30102) {
    sendPagedHistory(mac); return;
  }
  if (server_.arg("source") == "live" || isMax30102 ||
      (hasLiveView && lil::protocol::isLiveSensor(liveView.runtime.telemetry.sensorType) &&
       !(liveView.runtime.telemetry.flags & lil::protocol::kBatteryProtectionActive))) {
    const auto liveType =
        (hasLiveView && lil::protocol::isLiveSensor(liveView.runtime.telemetry.sensorType))
            ? liveView.runtime.telemetry.sensorType
            : (lil::protocol::isLiveSensor(config.environmentalSensorType)
                   ? config.environmentalSensorType
                   : lil::protocol::EnvironmentalSensorType::kMax30102);
    const size_t liveCapacity = 60000 / lil::protocol::liveHistoryIntervalMs(liveType) + 1;
    auto* samples = static_cast<LiveSample*>(heap_caps_malloc(liveCapacity * sizeof(LiveSample), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!samples && heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) > liveCapacity * sizeof(LiveSample) + 96U * 1024U)
      samples = static_cast<LiveSample*>(heap_caps_malloc(liveCapacity * sizeof(LiveSample), MALLOC_CAP_8BIT));
    if (!samples) { sendJsonResult(false, "Live history is temporarily unavailable"); return; }
    const uint32_t* cursor = server_.hasArg("afterMs") ? &afterMs : nullptr;
    const size_t count = registry_->liveHistory(mac, samples, liveCapacity, cursor);
    const uint32_t now = millis();
    if (server_.arg("format") == "bin-v1") {
      BinaryLiveHeader hdr{
          0x31424357UL, 1U, static_cast<uint16_t>(sizeof(BinaryLiveRecord)),
          static_cast<uint32_t>(count),
          static_cast<uint32_t>(lil::protocol::liveReportIntervalMs(liveType))};
      sendNoCache(server_);
      server_.client().setNoDelay(true);
      server_.setContentLength(sizeof(hdr) + count * sizeof(BinaryLiveRecord));
      server_.send(200, "application/octet-stream", "");
      auto client = server_.client();
      auto writeAll = [&](const uint8_t* ptr, size_t len) -> bool {
        size_t sent = 0;
        while (sent < len && client.connected()) {
          const size_t n = client.write(ptr + sent, len - sent);
          if (n == 0) {
            client.stop();
            return false;
          }
          sent += n;
        }
        return sent == len;
      };
      if (client.connected() &&
          writeAll(reinterpret_cast<const uint8_t*>(&hdr), sizeof(hdr))) {
        constexpr size_t kBatch = 42;  // 42 * 68 = 2856 B (~2 * 1460-byte TCP MSS).
        std::unique_ptr<BinaryLiveRecord[]> batch(new (std::nothrow) BinaryLiveRecord[kBatch]);
        if (batch) {
          for (size_t offset = 0; offset < count && client.connected(); offset += kBatch) {
#ifdef ESP_PLATFORM
            (void)esp_task_wdt_reset();
#endif
            const size_t n = min(kBatch, count - offset);
            for (size_t i = 0; i < n; ++i) {
              packBinaryLiveRecord(batch[i], samples[offset + i].telemetry,
                                   samples[offset + i].receivedMs, now);
            }
            if (!writeAll(reinterpret_cast<const uint8_t*>(batch.get()),
                          n * sizeof(BinaryLiveRecord))) {
              break;
            }
          }
        } else {
          for (size_t i = 0; i < count && client.connected(); ++i) {
#ifdef ESP_PLATFORM
            (void)esp_task_wdt_reset();
#endif
            BinaryLiveRecord single{};
            packBinaryLiveRecord(single, samples[i].telemetry, samples[i].receivedMs, now);
            if (!writeAll(reinterpret_cast<const uint8_t*>(&single), sizeof(single))) {
              break;
            }
          }
        }
      }
      heap_caps_free(samples);
      server_.setContentLength(CONTENT_LENGTH_NOT_SET);
      return;
    }
    sendNoCache(server_); server_.client().setNoDelay(true); server_.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server_.send(200, "application/json", "");
    server_.sendContent(String("{\"live\":true,\"bucketSeconds\":") +
        String(lil::protocol::liveReportIntervalMs(liveType) * 0.001F, 1) + ",\"points\":[");
    constexpr size_t kChunkCap = 4096;
    constexpr size_t kChunkFlush = 2920;  // 2 * 1460-byte TCP MSS.
    std::unique_ptr<char[]> chunkBuf(new (std::nothrow) char[kChunkCap]);
    if (!chunkBuf) {
      heap_caps_free(samples);
      server_.client().stop();
      server_.setContentLength(CONTENT_LENGTH_NOT_SET);
      return;
    }
    char* cb = chunkBuf.get();
    size_t clen = 0;
    cb[0] = '\0';
    for (size_t i = 0; i < count; ++i) {
      const auto& sample = samples[i];
      const auto& t = sample.telemetry;
      const uint32_t ageMs = static_cast<uint32_t>(now - sample.receivedMs);
      const bool timed = (t.live.flags & lil::protocol::kLiveTimingKnown) != 0;
      const uint32_t measAge = ageMs + (timed && t.live.acquisitionAgeMs != UINT16_MAX ? t.live.acquisitionAgeMs : 0U);
      const uint32_t estAge = ageMs + (timed && t.live.estimateAgeMs != UINT16_MAX ? t.live.estimateAgeMs : 0U);
      const bool gap = t.motion.fifoOverrun || (t.live.flags & lil::protocol::kLiveGap);
      bool ok = (!i || appendRawChar(cb, kChunkCap, clen, ',')) &&
                appendRawStr(cb, kChunkCap, clen, "{\"id\":") &&
                appendUnsigned(cb, kChunkCap, clen, sample.receivedMs) &&
                appendRawStr(cb, kChunkCap, clen, ",\"sensorType\":") &&
                appendUnsigned(cb, kChunkCap, clen, static_cast<uint8_t>(t.sensorType)) &&
                appendRawStr(cb, kChunkCap, clen, ",\"ageMs\":") &&
                appendUnsigned(cb, kChunkCap, clen, ageMs) &&
                appendRawStr(cb, kChunkCap, clen, ",\"measurementAgeMs\":") &&
                appendUnsigned(cb, kChunkCap, clen, measAge) &&
                appendRawStr(cb, kChunkCap, clen, ",\"windowMs\":") &&
                appendUnsigned(cb, kChunkCap, clen, uint16_t(t.live.windowMs)) &&
                appendRawStr(cb, kChunkCap, clen, ",\"estimateAgeMs\":") &&
                appendUnsigned(cb, kChunkCap, clen, estAge) &&
                appendRawStr(cb, kChunkCap, clen, gap ? ",\"gap\":true" : ",\"gap\":false");
      if (ok && timed && t.live.count <= lil::protocol::kOpticalBatchSize) {
        ok = appendRawStr(cb, kChunkCap, clen, ",\"wave\":[");
        for (uint8_t j = 0; ok && j < t.live.count; ++j) {
          ok = (!j || appendRawChar(cb, kChunkCap, clen, ',')) &&
               appendRawStr(cb, kChunkCap, clen, "{\"ageMs\":") &&
               appendUnsigned(cb, kChunkCap, clen, ageMs + t.live.optical[j].ageMs) &&
               appendRawStr(cb, kChunkCap, clen, ",\"red\":") &&
               appendUnsigned(cb, kChunkCap, clen, uint32_t(t.live.optical[j].red)) &&
               appendRawStr(cb, kChunkCap, clen, ",\"infrared\":") &&
               appendUnsigned(cb, kChunkCap, clen, uint32_t(t.live.optical[j].infrared)) &&
               appendRawChar(cb, kChunkCap, clen, '}');
        }
        ok = ok && appendRawChar(cb, kChunkCap, clen, ']');
      }
      if (ok && !(t.flags & lil::protocol::kSensorReadFailed)) {
        if ((t.capabilities & lil::protocol::kTemperature) && (!timed || (t.live.flags & lil::protocol::kLiveSampleFresh))) {
          ok = appendRawStr(cb, kChunkCap, clen, ",\"temperature\":") &&
               appendScaledFloat(cb, kChunkCap, clen, float(t.temperatureC), 4);
        }
        if (ok && (t.capabilities & lil::protocol::kMotion) && !t.motion.fifoOverrun) {
          static constexpr const char* kAxisKeys[] = {
              ",\"ax\":", ",\"ay\":", ",\"az\":", ",\"gx\":", ",\"gy\":", ",\"gz\":"};
          for (int axis = 0; ok && axis < 6; ++axis) {
            ok = appendRawStr(cb, kChunkCap, clen, kAxisKeys[axis]) &&
                 appendScaledFloat(cb, kChunkCap, clen,
                                   float(axis < 3 ? t.motion.accelerationG[axis] : t.motion.angularRateDps[axis - 3]),
                                   axis < 3 ? 4 : 3);
          }
          ok = ok && appendRawStr(cb, kChunkCap, clen, ",\"peakAcceleration\":") &&
               appendScaledFloat(cb, kChunkCap, clen, float(t.motion.peakAccelerationG), 4) &&
               appendRawStr(cb, kChunkCap, clen, ",\"peakAngularRate\":") &&
               appendScaledFloat(cb, kChunkCap, clen, float(t.motion.peakAngularRateDps), 3);
        }
        if (ok && (t.capabilities & lil::protocol::kOptical)) {
          ok = appendRawStr(cb, kChunkCap, clen, ",\"red\":") &&
               appendUnsigned(cb, kChunkCap, clen, uint32_t(t.pulse.red)) &&
               appendRawStr(cb, kChunkCap, clen, ",\"infrared\":") &&
               appendUnsigned(cb, kChunkCap, clen, uint32_t(t.pulse.infrared));
        }
        if (ok && (t.capabilities & lil::protocol::kHeartRate) && (!timed || (t.live.flags & lil::protocol::kLiveEstimateFresh))) {
          ok = appendRawStr(cb, kChunkCap, clen, ",\"heartRate\":") &&
               appendScaledFloat(cb, kChunkCap, clen, float(t.pulse.beatsPerMinute), 2);
        }
      }
      ok = ok && appendRawStr(cb, kChunkCap, clen, t.motion.fifoOverrun ? ",\"motionOverrun\":true" : ",\"motionOverrun\":false") &&
           appendRawStr(cb, kChunkCap, clen, ",\"pulseStatus\":") &&
           appendUnsigned(cb, kChunkCap, clen, t.pulse.status) &&
           appendRawStr(cb, kChunkCap, clen, ",\"quality\":") &&
           appendUnsigned(cb, kChunkCap, clen, t.pulse.quality);
      if (ok && (t.live.flags & lil::protocol::kMotionFeedbackPresent)) {
        ok = appendRawStr(cb, kChunkCap, clen, ",\"steps\":") &&
             appendUnsigned(cb, kChunkCap, clen, uint16_t(t.motionFeedback.steps)) &&
             appendRawStr(cb, kChunkCap, clen, ",\"activeSeconds\":") &&
             appendUnsigned(cb, kChunkCap, clen, uint16_t(t.motionFeedback.activeSeconds));
      }
      if (ok && (t.capabilities & lil::protocol::kBattery)) {
        ok = appendRawStr(cb, kChunkCap, clen, ",\"battery\":") &&
             appendScaledFloat(cb, kChunkCap, clen, t.batteryMillivolts * 0.001F, 3);
      }
      ok = ok && appendRawChar(cb, kChunkCap, clen, '}');
      if (!ok) {
        heap_caps_free(samples);
        server_.client().stop();
        server_.setContentLength(CONTENT_LENGTH_NOT_SET);
        return;
      }
      if (clen >= kChunkFlush) {
#ifdef ESP_PLATFORM
        (void)esp_task_wdt_reset();
#endif
        if (!server_.client().connected()) {
          heap_caps_free(samples);
          server_.setContentLength(CONTENT_LENGTH_NOT_SET);
          return;
        }
        server_.sendContent(cb, clen);
        clen = 0;
        cb[0] = '\0';
      }
    }
    heap_caps_free(samples);
    if (!appendRawStr(cb, kChunkCap, clen, "]}")) {
      server_.client().stop();
      server_.setContentLength(CONTENT_LENGTH_NOT_SET);
      return;
    }
    if (server_.client().connected()) {
      server_.sendContent(cb, clen);
      server_.sendContent("");
    }
    server_.setContentLength(CONTENT_LENGTH_NOT_SET);
    return;
  }
  sendLegacyJsonHistory(mac, since);
}

#if defined(__GNUC__) && !defined(__clang__)
[[gnu::noinline]]
#endif
void WebPortal::sendLegacyJsonHistory(const uint8_t mac[6], uint32_t since) {
  const size_t capacity = registry_->historyCapacity(mac);
  HistorySample* samples = nullptr;
  if (capacity > 0) {
    samples = static_cast<HistorySample*>(heap_caps_malloc(
        capacity * sizeof(HistorySample),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (samples == nullptr &&
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) >
            capacity * sizeof(HistorySample) + 96U * 1024U) {
      samples = static_cast<HistorySample*>(heap_caps_malloc(
          capacity * sizeof(HistorySample), MALLOC_CAP_8BIT));
    }
    if (samples == nullptr) {
      sendJsonResult(false, "History is temporarily unavailable");
      return;
    }
  }
  const size_t count = registry_->history(mac, samples, capacity);
  const uint32_t bucketSeconds = registry_->historyBucketSeconds(mac);
  sendNoCache(server_);
  server_.client().setNoDelay(true);
  server_.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server_.send(200, "application/json", "");
  char openingBuf[128];
  snprintf(openingBuf, sizeof(openingBuf),
           "{\"mac\":\"%s\",\"bucketSeconds\":%lu,\"bucketMinutes\":%.2f,\"points\":[",
           SensorRegistry::formatMac(mac).c_str(),
           static_cast<unsigned long>(bucketSeconds),
           static_cast<double>(bucketSeconds * (1.0F / 60.0F)));
  server_.sendContent(openingBuf);
  String chunk;
  chunk.reserve(2048);
  size_t emitted = 0;
  char point[384];
  for (size_t i = 0; i < count; ++i) {
    const HistorySample& sample = samples[i];
    if (sample.timestamp < since) continue;
    size_t pos = 0;
    const auto appendFmt = [&](const char* fmt, auto... args) {
      if (pos >= sizeof(point)) return;
      const int n = snprintf(point + pos, sizeof(point) - pos, fmt, args...);
      if (n > 0) pos += min<size_t>(static_cast<size_t>(n), sizeof(point) - pos);
    };
    appendFmt("%s\"t\":%llu,\"sensorType\":%u,\"pcbVersion\":%u,\"iaqAccuracy\":%u",
              emitted++ == 0 ? "{" : ",{",
              static_cast<unsigned long long>(sample.timestamp) * 1000ULL,
              unsigned(sample.sensorType), unsigned(sample.pcbVersion),
              unsigned(sample.iaqAccuracy));
    if ((sample.capabilities & lil::protocol::kMotion) != 0) {
      appendFmt(",\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f"
                ",\"peakAcceleration\":%.3f,\"peakAngularRate\":%.1f,\"motionOverrun\":%s",
                static_cast<double>(sample.accelerationMilliG[0] * 0.001F),
                static_cast<double>(sample.accelerationMilliG[1] * 0.001F),
                static_cast<double>(sample.accelerationMilliG[2] * 0.001F),
                static_cast<double>(sample.angularRateDeciDps[0] * 0.1F),
                static_cast<double>(sample.angularRateDeciDps[1] * 0.1F),
                static_cast<double>(sample.angularRateDeciDps[2] * 0.1F),
                static_cast<double>(sample.peakAccelerationMilliG * 0.001F),
                static_cast<double>(sample.peakAngularRateDeciDps * 0.1F),
                sample.motionOverrun ? "true" : "false");
    }
    if ((sample.capabilities & lil::protocol::kTemperature) != 0) {
      appendFmt(",\"temperature\":%.2f",
                static_cast<double>(sample.temperatureCentiC * 0.01F));
    }
    if ((sample.capabilities & lil::protocol::kHumidity) != 0) {
      appendFmt(",\"humidity\":%.2f",
                static_cast<double>(sample.humidityCentiPercent * 0.01F));
    }
    if ((sample.capabilities & lil::protocol::kPressure) != 0) {
      appendFmt(",\"pressure\":%.1f",
                static_cast<double>(sample.pressureDeciHpa * 0.1F));
    }
    if ((sample.capabilities & lil::protocol::kIaq) != 0 &&
        sample.iaqAccuracy > 0) {
      appendFmt(",\"iaq\":%.1f", static_cast<double>(sample.iaqDeci * 0.1F));
    }
    if ((sample.capabilities & lil::protocol::kGasResistance) != 0) {
      appendFmt(",\"gas\":%.3f",
                static_cast<double>(sample.gasResistanceOhms * 0.001F));
    }
    if ((sample.capabilities & lil::protocol::kBattery) != 0) {
      appendFmt(",\"battery\":%.3f",
                static_cast<double>(sample.batteryMillivolts * 0.001F));
    }
    if (pos + 1 < sizeof(point)) {
      point[pos++] = '}';
      point[pos] = '\0';
    }
    if (chunk.length() + pos > 2048) {
#ifdef ESP_PLATFORM
      (void)esp_task_wdt_reset();
#endif
      if (!server_.client().connected()) {
        if (samples != nullptr) heap_caps_free(samples);
        server_.setContentLength(CONTENT_LENGTH_NOT_SET);
        return;
      }
      server_.sendContent(chunk);
      chunk = "";
    }
    chunk.concat(point, pos);
  }
  if (server_.client().connected()) {
    if (!chunk.isEmpty()) {
      server_.sendContent(chunk);
    }
    server_.sendContent("]}");
    server_.sendContent("");
  }
  server_.setContentLength(CONTENT_LENGTH_NOT_SET);
  if (samples != nullptr) {
    heap_caps_free(samples);
  }
}

void WebPortal::sendPagedHistory(const uint8_t mac[6]) {
  uint32_t after=0,until=0,oldest=0,requestedCapacity=128;
  if(!readUnsignedArgument(server_,"after",after,0,UINT32_MAX,true)||
     !readUnsignedArgument(server_,"until",until,0,UINT32_MAX,true)||
     !readUnsignedArgument(server_,"oldest",oldest,0,UINT32_MAX,true)||
     !readUnsignedArgument(server_,"limit",requestedCapacity,1,512,true)){
    sendJsonResult(false,"Invalid history page");return;
  }
  auto* history=registry_->normalHistory();const auto info=history->info(mac);
  if(!info.available){sendJsonResult(false,"History storage is unavailable; existing files were preserved.");return;}
  if(!until)until=info.newest;
  if(!oldest)oldest=info.oldest;
  size_t capacity=requestedCapacity;
  auto* points=static_cast<NormalHistoryPoint*>(heap_caps_calloc(capacity,sizeof(NormalHistoryPoint),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
  if(!points){capacity=min(capacity,size_t(128));
    if(heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT)>capacity*sizeof(NormalHistoryPoint)+96U*1024U)
      points=static_cast<NormalHistoryPoint*>(heap_caps_calloc(capacity,sizeof(NormalHistoryPoint),MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));}
  if(!points){sendJsonResult(false,"History memory is temporarily unavailable");return;}
  bool stale=false,error=false;const size_t count=history->read(mac,after,until,oldest,points,capacity,stale,error);
  if(stale||error){heap_caps_free(points);sendJsonResult(false,stale?"History changed while loading. Retry the selected window.":"History page failed CRC validation.");return;}
  sendNoCache(server_);server_.client().setNoDelay(true);server_.setContentLength(CONTENT_LENGTH_UNKNOWN);server_.send(200,"application/json","");
  JsonDocument header;header["mac"]=SensorRegistry::formatMac(mac);header["paged"]=true;
  header["bucketSeconds"]=info.intervalSeconds;header["capacityPoints"]=info.capacityPoints;
  header["pointsTotal"]=info.points;header["durablePoints"]=info.durablePoints;header["pendingPoints"]=info.pendingPoints;
  header["until"]=until;header["oldest"]=oldest;header["nextAfter"]=count==capacity&&points[count-1].timestamp<until?points[count-1].timestamp:0;
  const bool compact=server_.arg("format")=="array-v1";
  const uint32_t columns=compact?historyColumnMask(points,count):0;
  if(compact){header["format"]="array-v1";JsonArray fields=header["fields"].to<JsonArray>();
    for(size_t i=0;i<kHistoryColumnCount;++i)if(columns&(1UL<<i))fields.add(kHistoryColumns[i]);}
  if(header.overflowed()){heap_caps_free(points);server_.client().stop();server_.setContentLength(CONTENT_LENGTH_NOT_SET);return;}
  String opening;serializeJson(header,opening);
  if(opening.length()<=1){heap_caps_free(points);server_.client().stop();server_.setContentLength(CONTENT_LENGTH_NOT_SET);return;}
  opening.remove(opening.length()-1);opening+=",\"points\":[";server_.sendContent(opening);
  if (compact) {
    size_t chunkCap = 16384;
    size_t chunkFlush = 11680;  // 8 * 1460-byte TCP MSS.
    char* cb = static_cast<char*>(
        heap_caps_malloc(chunkCap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!cb) {
      chunkCap = 4096;
      chunkFlush = 2920;
      cb = static_cast<char*>(
          heap_caps_malloc(chunkCap, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    if (!cb) {
      heap_caps_free(points);
      server_.client().stop();
      server_.setContentLength(CONTENT_LENGTH_NOT_SET);
      return;
    }
    size_t clen = 0;
    cb[0] = '\0';
    for (size_t i = 0; i < count; ++i) {
      size_t savedLen = clen;
      bool appended = (!i || appendRawChar(cb, chunkCap, clen, ',')) &&
                      appendCompactHistoryPoint(cb, chunkCap, clen, points[i], columns);
      if (!appended && savedLen > 0) {
#ifdef ESP_PLATFORM
        (void)esp_task_wdt_reset();
#endif
        if (!server_.client().connected()) {
          heap_caps_free(cb);
          heap_caps_free(points);
          server_.setContentLength(CONTENT_LENGTH_NOT_SET);
          return;
        }
        server_.sendContent(cb, savedLen);
        clen = 0;
        cb[0] = '\0';
        appended = (!i || appendRawChar(cb, chunkCap, clen, ',')) &&
                   appendCompactHistoryPoint(cb, chunkCap, clen, points[i], columns);
      }
      if (!appended) {
        heap_caps_free(cb);
        heap_caps_free(points);
        server_.client().stop();
        server_.setContentLength(CONTENT_LENGTH_NOT_SET);
        return;
      }
      if (clen >= chunkFlush) {
#ifdef ESP_PLATFORM
        (void)esp_task_wdt_reset();
#endif
        if (!server_.client().connected()) {
          heap_caps_free(cb);
          heap_caps_free(points);
          server_.setContentLength(CONTENT_LENGTH_NOT_SET);
          return;
        }
        server_.sendContent(cb, clen);
        clen = 0;
        cb[0] = '\0';
      }
    }
    heap_caps_free(points);
    if (!appendRawStr(cb, chunkCap, clen, "]}")) {
      heap_caps_free(cb);
      server_.client().stop();
      server_.setContentLength(CONTENT_LENGTH_NOT_SET);
      return;
    }
    if (server_.client().connected()) {
      server_.sendContent(cb, clen);
      server_.sendContent("");
    }
    heap_caps_free(cb);
    server_.setContentLength(CONTENT_LENGTH_NOT_SET);
    return;
  }
  String chunk;chunk.reserve(4096);
  JsonDocument row;
  for(size_t i=0;i<count;++i){
    row.clear();
    historyPointJson(row,points[i]);
    if (row.overflowed() || (i && !chunk.concat(',')) || !appendJson(row,chunk)) {
      heap_caps_free(points);
      server_.client().stop();
      server_.setContentLength(CONTENT_LENGTH_NOT_SET);
      return;
    }
    if(chunk.length()>=2920){
#ifdef ESP_PLATFORM
      (void)esp_task_wdt_reset();
#endif
      if(!server_.client().connected()){heap_caps_free(points);server_.setContentLength(CONTENT_LENGTH_NOT_SET);return;}
      server_.sendContent(chunk);chunk="";
    }
  }
  heap_caps_free(points);
  if (!chunk.concat("]}")) { server_.client().stop(); server_.setContentLength(CONTENT_LENGTH_NOT_SET); return; }
  if (server_.client().connected()) {
    server_.sendContent(chunk);server_.sendContent("");
  }
  server_.setContentLength(CONTENT_LENGTH_NOT_SET);
}

void WebPortal::saveDashboardLayout() {
  DashboardLayout layout{};uint32_t columns=0,comfortable=0;
  if(!readUnsignedArgument(server_,"columns",columns,0,3)||!readUnsignedArgument(server_,"comfortable",comfortable,0,1)){
    sendJsonResult(false,"Invalid dashboard layout");return;
  }
  layout.columns=columns;layout.comfortable=comfortable;
  const String order=server_.arg("order"),hidden=server_.arg("hidden");
  if(order.length()>kMaxSensors*18||hidden.length()>kMaxSensors*18){sendJsonResult(false,"Too many dashboard sensors");return;}
  auto parse=[&](const String& text,bool hide){
    if(!text.length())return true;
    char bytes[kMaxSensors*18+1];text.toCharArray(bytes,sizeof(bytes));char* state=nullptr;
    for(char* token=strtok_r(bytes,",",&state);token;token=strtok_r(nullptr,",",&state)){
      uint8_t mac[6]{};SensorConfig sensor{};if(!SensorRegistry::parseMac(token,mac)||!registry_->findConfig(mac,sensor)||!sensor.provisioned)return false;
      size_t index=0;while(index<layout.count&&memcmp(layout.entries[index].mac,mac,6))++index;
      if(hide){if(index==layout.count)return false;layout.entries[index].hidden=1;}
      else{if(index<layout.count||layout.count==kMaxSensors)return false;memcpy(layout.entries[layout.count++].mac,mac,6);}
    }return true;
  };
  if(!parse(order,false)||!parse(hidden,true)){sendJsonResult(false,"Dashboard sensor list is invalid or changed. Reload and try again.");return;}
  const bool saved=store_->saveDashboardLayout(layout);
  if(saved){dashboardLayout_=layout;dashboardLayoutLoaded_=true;}
  sendJsonResult(saved,saved?"Dashboard layout saved.":"Could not save dashboard layout.");
}

void WebPortal::sendJsonConfig() {
  JsonDocument document(psramJsonAllocator());
  document["storageAvailable"] = store_->storageAvailable();
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
  if (!dashboardLayoutLoaded_) {
    dashboardLayout_ = store_->loadDashboardLayout();
    dashboardLayoutLoaded_ = true;
  }
  const auto& layout = dashboardLayout_;
  JsonObject dashboard=document["dashboard"].to<JsonObject>();
  dashboard["columns"]=layout.columns;dashboard["comfortable"]=bool(layout.comfortable);
  JsonArray order=dashboard["order"].to<JsonArray>();JsonArray hidden=dashboard["hidden"].to<JsonArray>();
  for(size_t i=0;i<layout.count;++i){const String mac=SensorRegistry::formatMac(layout.entries[i].mac);order.add(mac);if(layout.entries[i].hidden)hidden.add(mac);}
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
  if (document.overflowed()) {
    sendNoCache(server_);
    server_.send(503, "application/json",
                 "{\"success\":false,\"message\":\"Configuration memory temporarily unavailable\"}");
    return;
  }
  sendNoCache(server_);
  sendStreamedJsonDocument(server_, document);
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
  if (document.overflowed()) {
    sendNoCache(server_);
    server_.send(503, "application/json",
                 "{\"success\":false,\"message\":\"Configuration memory temporarily unavailable\"}");
    return;
  }
  sendNoCache(server_);
  sendStreamedJsonDocument(server_, document);
}

void WebPortal::saveSetup() {
  if (config_->wifiSsid[0] != '\0') {
    sendJsonResult(false,
                   "The station is already configured. Use Settings.");
    return;
  }
  auto previousConfig = cloneStationConfig(*config_);
  if (!previousConfig) {
    sendJsonResult(false, "Configuration memory temporarily unavailable.");
    return;
  }
  const String oldSsid = config_->wifiSsid;
  const String ssid = server_.arg("ssid");
  const String password = server_.arg("wifiPassword");
  const String userApiKey = server_.arg("userApiKey");
  const String adminPassword = server_.arg("adminPassword");
  const String confirmAdminPassword = server_.arg("confirmAdminPassword");
  uint32_t defaultSleep = 0;
  if (!readUnsignedArgument(server_, "defaultSleep", defaultSleep,
                            kMinSleepSeconds, kMaxSleepSeconds)) {
    sendJsonResult(false, "Invalid measurement interval"); return;
  }
  if ((!adminPassword.isEmpty() &&
       (adminPassword.length() < 8 || adminPassword.length() > 64)) ||
      adminPassword != confirmAdminPassword) {
    sendJsonResult(false,
                   "The website passwords must match and contain 8 to 64 characters.");
    return;
  }
  if (ssid.isEmpty() || ssid.length() > 32 || password.length() > 64 ||
      userApiKey.length() > 40 || defaultSleep < kMinSleepSeconds ||
      defaultSleep > kMaxSleepSeconds) {
    sendJsonResult(false, "Check Wi-Fi and the measurement interval.");
    return;
  }

  copyText(config_->hostname, "w-charger");
  copyText(config_->wifiSsid, ssid);
  if (!password.isEmpty() || ssid != oldSsid) {
    copyText(config_->wifiPassword, password);
  }
  copyText(config_->thingSpeakUserApiKey, userApiKey);
  setAdminPassword(adminPassword);
  config_->defaultSleepSeconds = defaultSleep;
  // Keep the setup portal alive through this reboot.  WifiService clears it
  // and persists completion only after the home WLAN really connects.
  config_->setupPortalRequired = true;
  const bool saved = store_->saveStationConfig(*config_);
  if (!saved) {
    *config_ = *previousConfig;
  }
  sendJsonResult(saved,
                 saved ? "Setup saved. W-Charger is restarting."
                       : "Setup could not be saved.",
                 saved);
}

void WebPortal::saveSetupSensor() {
  uint8_t mac[6];
  SensorView view{};
  const String name = server_.arg("name");
  uint32_t sleepSeconds = 0, sensorTypeValue = 0;
  if (!readUnsignedArgument(server_, "sleepSeconds", sleepSeconds,
                            kMinSleepSeconds, kMaxSleepSeconds) ||
      !readUnsignedArgument(server_, "sensorType", sensorTypeValue, 0, 255)) {
    sendJsonResult(false, "Invalid sensor type or interval"); return;
  }
  const auto sensorType =
      static_cast<lil::protocol::EnvironmentalSensorType>(sensorTypeValue);
  const bool validSensorType = lil::protocol::isValidSensorType(sensorType);
  if (!wifi_->setupPortalActive() ||
      !SensorRegistry::parseMac(server_.arg("mac"), mac) ||
      !registry_->findView(mac, view) || view.config.provisioned) {
    sendJsonResult(false,
                   "The sensor could not be configured. Check the selection and connection.");
    return;
  }
  const bool bme680 =
      lil::protocol::isBoschGasSensor(sensorType) ||
      (sensorType == lil::protocol::EnvironmentalSensorType::kAutoDetect &&
       lil::protocol::isBoschGasSensor(view.runtime.telemetry.sensorType));
  if (name.isEmpty() || name.length() > 24 || !validSensorType ||
      sleepSeconds < kMinSleepSeconds || sleepSeconds > kMaxSleepSeconds ||
      (bme680 && (sleepSeconds < 300 || sleepSeconds % 300 != 0))) {
    sendJsonResult(false,
                   "Sensor name, sensor type or interval is invalid.");
    return;
  }

  ThingSpeakFieldMapping fields{};
  const auto effectiveType =
      sensorType == lil::protocol::EnvironmentalSensorType::kAutoDetect
          ? view.runtime.telemetry.sensorType
          : sensorType;
  if (bme680) {
    fields.iaq = 4;
    fields.gasResistance = 5;
    fields.battery = 6;
  } else if (effectiveType == lil::protocol::EnvironmentalSensorType::kBme280) {
    fields.battery = 4;
  } else if (effectiveType == lil::protocol::EnvironmentalSensorType::kBmp280) {
    fields.humidity = 0;
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
  auto previousConfig = cloneStationConfig(*config_);
  if (!previousConfig) {
    sendJsonResult(false, "Configuration memory temporarily unavailable.");
    return;
  }
  const String oldSsid = config_->wifiSsid;
  const String oldApPassword = config_->accessPointPassword;
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
  uint32_t defaultSleep = 0, fallbackChannelValue = 0;
  if (!readUnsignedArgument(server_, "defaultSleep", defaultSleep,
                            kMinSleepSeconds, kMaxSleepSeconds) ||
      !readUnsignedArgument(server_, "fallbackChannel", fallbackChannelValue, 1, 13)) {
    sendJsonResult(false, "Invalid station interval or radio channel"); return;
  }
  if (ssid.isEmpty() || ssid.length() > 32 || wifiPassword.length() > 64 ||
      userApiKey.length() > 40 || defaultSleep < kMinSleepSeconds ||
      defaultSleep > kMaxSleepSeconds || fallbackChannelValue < 1 ||
      fallbackChannelValue > 13 ||
      (!apPassword.isEmpty() &&
       (apPassword.length() < 8 || apPassword.length() > 63)) ||
      (!newAdminPassword.isEmpty() &&
       (newAdminPassword.length() < 8 || newAdminPassword.length() > 64)) ||
      newAdminPassword != confirmAdminPassword ||
      (removeAdminPassword && !newAdminPassword.isEmpty())) {
    sendJsonResult(false, "Invalid station configuration");
    return;
  }
  const uint8_t fallbackChannel =
      static_cast<uint8_t>(fallbackChannelValue);

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
  if (!saved) {
    *config_ = *previousConfig;
  }
  const bool restartRequested = server_.arg("restartNow") == "1";
  const bool restartRequired =
      saved && (ssid != oldSsid || !wifiPassword.isEmpty() ||
                (!apPassword.isEmpty() && apPassword != oldApPassword) ||
                defaultSleep != oldDefaultSleep ||
                fallbackChannel != oldFallbackChannel || restartRequested);
  if (authenticationChanged) {
    clearAuthSessions();
    refreshCsrfToken();
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
  uint32_t sleepSeconds = 0, channelId = 0, sensorTypeValue = 0;
  if (!readUnsignedArgument(server_, "sleepSeconds", sleepSeconds,
                            kMinSleepSeconds, kMaxSleepSeconds) ||
      !readUnsignedArgument(server_, "channelId", channelId, 0, UINT32_MAX, true) ||
      !readUnsignedArgument(server_, "sensorType", sensorTypeValue, 0, 255)) {
    sendJsonResult(false, "Invalid sensor configuration"); return;
  }
  String writeKey = server_.arg("writeKey");
  int profileIndex = -1;
  if (server_.hasArg("profileSlot") && server_.arg("profileSlot") != "-1") {
    uint32_t parsedSlot = 0;
    if (!readUnsignedArgument(server_, "profileSlot", parsedSlot, 0,
                              kMaxThingSpeakChannels - 1) ||
        !config_->thingSpeakChannels[parsedSlot].occupied) {
      sendJsonResult(false, "Invalid channel profile"); return;
    }
    profileIndex = static_cast<int>(parsedSlot);
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
  if (!readFieldArgument(server_, "temperatureField", fields.temperature) ||
      !readFieldArgument(server_, "humidityField", fields.humidity) ||
      !readFieldArgument(server_, "pressureField", fields.pressure) ||
      !readFieldArgument(server_, "iaqField", fields.iaq) ||
      !readFieldArgument(server_, "batteryField", fields.battery) ||
      !readFieldArgument(server_, "gasResistanceField", fields.gasResistance)) {
    sendJsonResult(false, "ThingSpeak fields must be integers from 0 to 8"); return;
  }
  const auto sensorType =
      static_cast<lil::protocol::EnvironmentalSensorType>(sensorTypeValue);
  const bool validSensorType = lil::protocol::isValidSensorType(sensorType);
  SensorView existingView{};
  if (!SensorRegistry::parseMac(server_.arg("mac"), mac) ||
      !registry_->findView(mac, existingView) ||
      server_.arg("name").isEmpty() || server_.arg("name").length() > 24 ||
      sleepSeconds < kMinSleepSeconds || sleepSeconds > kMaxSleepSeconds ||
      writeKey.length() > 32 || !validSensorType ||
      (lil::protocol::isBoschGasSensor(sensorType) &&
       (sleepSeconds < 300 || sleepSeconds % 300 != 0))) {
    sendJsonResult(false, "Invalid sensor configuration");
    return;
  }
  const SensorConfig& existing = existingView.config;
  float temperatureOffsetC = existing.temperatureOffsetC;
  if (server_.hasArg("temperatureOffsetC") &&
      !webinput::finiteDecimal(server_.arg("temperatureOffsetC").c_str(),
                               temperatureOffsetC, -10.0F, 10.0F)) {
    sendJsonResult(false, "Invalid temperature offset"); return;
  }
  const bool bme680Effective =
      lil::protocol::isBoschGasSensor(sensorType) ||
      (sensorType == lil::protocol::EnvironmentalSensorType::kAutoDetect &&
       lil::protocol::isBoschGasSensor(existingView.runtime.telemetry.sensorType));
  if (bme680Effective && (sleepSeconds < 300 || sleepSeconds % 300 != 0)) {
    sendJsonResult(
        false,
        "A detected BME680/BME688 requires a five-minute interval or a multiple of five minutes.");
    return;
  }
  float batteryCalibrationFactor = existing.batteryCalibrationFactor;
  if (server_.hasArg("resetBatteryCalibration")) {
    batteryCalibrationFactor = 1.0F;
  } else {
    const String batteryReference = server_.arg("batteryReferenceVoltage");
    if (!batteryReference.isEmpty()) {
      float referenceVoltage = 0.0F;
      const uint16_t measuredMillivolts =
          existingView.runtime.telemetry.batteryMillivolts;
      if (!webinput::finiteDecimal(batteryReference.c_str(), referenceVoltage,
                                   2.0F, 5.0F) ||
          !existingView.runtime.hasTelemetry || measuredMillivolts < 2000 ||
          (existingView.runtime.telemetry.flags &
           lil::protocol::kBatteryReadFailed) != 0) {
        sendJsonResult(
            false,
            "Battery calibration failed. Check the reference voltage and latest measurement.");
        return;
      }
      if (existingView.runtime.telemetry.appliedConfigRevision !=
          existing.revision) {
        sendJsonResult(
            false,
            "Wait for the sensor to apply its previous settings before calibrating the battery.");
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
        "Do not assign ThingSpeak fields 1-8 more than once. At least "
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
  auto previousConfig = cloneStationConfig(*config_);
  if (!previousConfig) {
    sendJsonResult(false, "Configuration memory temporarily unavailable.");
    return;
  }
  uint32_t requestedSlot = 0;
  if (server_.hasArg("slot") &&
      !readUnsignedArgument(server_, "slot", requestedSlot, 0, kMaxThingSpeakChannels - 1)) {
    sendJsonResult(false, "Invalid channel profile"); return;
  }
  int slot = server_.hasArg("slot") ? static_cast<int>(requestedSlot) : -1;
  if (slot < 0) {
    slot = -1;
    for (size_t i = 0; i < kMaxThingSpeakChannels; ++i) {
      if (!config_->thingSpeakChannels[i].occupied) {
        slot = static_cast<int>(i);
        break;
      }
    }
  }
  const String name = server_.arg("name");
  uint32_t channelId = 0;
  if (!readUnsignedArgument(server_, "channelId", channelId, 1)) {
    sendJsonResult(false, "Invalid Channel ID"); return;
  }
  const String readApiKey = server_.arg("readApiKey");
  const String writeApiKey = server_.arg("writeApiKey");
  if (slot < 0 || channelId == 0 || name.length() > 24 ||
      readApiKey.length() > 32 || writeApiKey.length() > 32) {
    sendJsonResult(false,
                   slot < 0 ? "A maximum of six channels is supported."
                            : "Check the Channel ID and API keys.");
    return;
  }
  for (size_t i = 0; i < kMaxThingSpeakChannels; ++i) {
    if (static_cast<int>(i) != slot && config_->thingSpeakChannels[i].occupied &&
        config_->thingSpeakChannels[i].channelId == channelId) {
      sendJsonResult(false, "This Channel ID is already saved in another local profile.");
      return;
    }
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
  const bool stationSaved = store_->saveStationConfig(*config_);
  const bool saved = stationSaved && registry_->syncThingSpeakProfile(slot, profile);
  if (!saved) {
    *config_ = *previousConfig;
    if (stationSaved) store_->saveStationConfig(*previousConfig);
  }
  sendJsonResult(saved, saved ? "ThingSpeak channel saved."
                               : "Channel could not be saved.");
}

void WebPortal::deleteChannelProfile() {
  auto previousConfig = cloneStationConfig(*config_);
  if (!previousConfig) {
    sendJsonResult(false, "Configuration memory temporarily unavailable.");
    return;
  }
  uint32_t slot = 0;
  if (!readUnsignedArgument(server_, "slot", slot, 0, kMaxThingSpeakChannels - 1) ||
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
  const bool stationSaved = store_->saveStationConfig(*config_);
  const bool saved = stationSaved && registry_->removeThingSpeakProfile(slot);
  if (!saved) {
    *config_ = *previousConfig;
    if (stationSaved) store_->saveStationConfig(*previousConfig);
  }
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
  ThingSpeakAccountJob::Request request;
  request.operation = ThingSpeakAccountJob::Operation::QuickCreate;
  copyText(request.name, channelName);
  runThingSpeakMutation(std::move(request));
}

void WebPortal::listThingSpeakChannels() {
  if (!server_.hasArg("ticket")) {
    // Route legacy synchronous calls through the 10 KiB ThingSpeak worker task
    // so TLS handshakes never run on loopTask's 8 KiB stack.
    ThingSpeakAccountJob::Request request;
    memcpy(request.account, config_->thingSpeakUserApiKey, sizeof(request.account));
    String owner;
    if (!thingSpeakJobOwner(owner)) return;
    const char* ownerPtr = owner.length() ? owner.c_str() : "";
    const uint32_t ticket = thingSpeak_->startAccountJob(std::move(request), ownerPtr);
    if (!ticket) {
      sendJsonResult(false, config_->thingSpeakUserApiKey[0] == '\0'
                         ? "Save the ThingSpeak User API Key first."
                         : "ThingSpeak account worker is busy or unavailable. Try again shortly.");
      return;
    }
    const uint32_t startedMs = millis();
    ThingSpeakApiResult result;
    for (;;) {
      const auto poll = thingSpeak_->collectAccountJob(
          ticket, config_->thingSpeakUserApiKey, ownerPtr, result);
      if (poll == ThingSpeakAccountJob::Poll::Ready) {
        sendThingSpeakChannels(result);
        return;
      }
      if (poll == ThingSpeakAccountJob::Poll::Missing ||
          uint32_t(millis() - startedMs) >= 16000UL) {
        sendJsonResult(false, "ThingSpeak channel list timed out. Try again shortly.");
        return;
      }
#ifdef ESP_PLATFORM
      (void)esp_task_wdt_reset();
#endif
      delay(20);
    }
  }
  uint32_t ticket = 0;
  if (!readUnsignedArgument(server_, "ticket", ticket, 1)) {
    sendJsonResult(false, "Invalid channel list ticket.");
    return;
  }
  ThingSpeakApiResult result;
  String owner;
  if (!thingSpeakJobOwner(owner)) return;
  switch (thingSpeak_->collectAccountJob(ticket, config_->thingSpeakUserApiKey,
                                         owner.length() ? owner.c_str() : "", result)) {
    case ThingSpeakAccountJob::Poll::Pending:
      sendNoCache(server_);
      server_.send(202, "application/json", "{\"success\":true,\"pending\":true}");
      return;
    case ThingSpeakAccountJob::Poll::Missing:
      sendJsonResult(false, "Channel list request is no longer available. Load channels again.");
      return;
    case ThingSpeakAccountJob::Poll::Ready:
      sendThingSpeakChannels(result);
      return;
  }
}

void WebPortal::startThingSpeakChannelList() {
  ThingSpeakAccountJob::Request request;
  memcpy(request.account, config_->thingSpeakUserApiKey, sizeof(request.account));
  String owner;
  if (!thingSpeakJobOwner(owner)) return;
  sendThingSpeakTicket(thingSpeak_->startAccountJob(std::move(request), owner.length() ? owner.c_str() : ""));
}

bool WebPortal::thingSpeakJobOwner(String& owner) {
  if (!authenticationEnabled()) return true;
  owner = currentSessionToken();
  if (owner.length() == 32) return true;
  sendJsonResult(false, "ThingSpeak session is unavailable.");
  return false;
}

void WebPortal::sendThingSpeakTicket(uint32_t ticket) {
  if (!ticket) {
    sendJsonResult(false, config_->thingSpeakUserApiKey[0] == '\0' ? "Save the ThingSpeak User API Key first." :
                   "ThingSpeak account worker is busy or unavailable. Try again shortly.");
    return;
  }
  sendNoCache(server_);
  server_.send(202, "application/json", "{\"success\":true,\"ticket\":" + String(ticket) + "}");
}

void WebPortal::runThingSpeakMutation(ThingSpeakAccountJob::Request request) {
  if (restartPending_) {
    sendJsonResult(false, "The station is restarting. Try again when it is available.");
    return;
  }
  request.configResetGeneration = configResetGeneration_;
  memcpy(request.account, config_->thingSpeakUserApiKey, sizeof(request.account));
  if (server_.hasArg("requestId")) {
    if (!readUnsignedArgument(server_, "requestId", request.nonce, 1)) {
      sendJsonResult(false, "Invalid ThingSpeak request identifier.");
      return;
    }
    String owner;
    if (!thingSpeakJobOwner(owner)) return;
    sendThingSpeakTicket(thingSpeak_->startAccountJob(std::move(request), owner.length() ? owner.c_str() : ""));
    return;
  }
  if (thingSpeak_->mutationPending()) {
    sendJsonResult(false, "Another ThingSpeak change is in progress. Try again shortly.");
    return;
  }
  // Route legacy synchronous mutations through the 10 KiB ThingSpeak worker
  // so TLS handshakes never overflow loopTask's 8 KiB stack.
  String owner;
  if (!thingSpeakJobOwner(owner)) return;
  const char* ownerPtr = owner.length() ? owner.c_str() : "";
  request.nonce = millis() ? millis() : 1U;
  const uint32_t ticket = thingSpeak_->startAccountJob(std::move(request), ownerPtr);
  if (!ticket) {
    sendJsonResult(false, config_->thingSpeakUserApiKey[0] == '\0'
                       ? "Save the ThingSpeak User API Key first."
                       : "ThingSpeak account worker is busy or unavailable. Try again shortly.");
    return;
  }
  const uint32_t startedMs = millis();
  ThingSpeakMutationResult result;
  for (;;) {
    finishThingSpeakMutation();
    const auto poll = thingSpeak_->collectAccountJob(
        ticket, config_->thingSpeakUserApiKey, ownerPtr, result);
    if (poll == ThingSpeakAccountJob::Poll::Ready) {
      sendThingSpeakMutation(result);
      return;
    }
    if (poll == ThingSpeakAccountJob::Poll::Missing ||
        uint32_t(millis() - startedMs) >= 16000UL) {
      sendThingSpeakMutation({false, "ThingSpeak operation timed out. Reload account channels before retrying."});
      return;
    }
#ifdef ESP_PLATFORM
    (void)esp_task_wdt_reset();
#endif
    delay(20);
  }
}

void WebPortal::finishThingSpeakMutation() {
  if (!thingSpeak_->mutationReady()) return;
  ThingSpeakAccountJob::Request request;
  ThingSpeakApiResult result;
  if (thingSpeak_->takeMutation(request, result))
    thingSpeak_->finishMutation(applyThingSpeakMutation(request, result));
}

void WebPortal::pollThingSpeakMutation() {
  uint32_t ticket = 0;
  if (!readUnsignedArgument(server_, "ticket", ticket, 1)) {
    sendJsonResult(false, "Invalid ThingSpeak operation ticket.");
    return;
  }
  String owner;
  if (!thingSpeakJobOwner(owner)) return;
  ThingSpeakMutationResult result;
  switch (thingSpeak_->collectAccountJob(ticket, config_->thingSpeakUserApiKey,
                                         owner.length() ? owner.c_str() : "", result)) {
    case ThingSpeakAccountJob::Poll::Pending:
      sendNoCache(server_);
      server_.send(202, "application/json", "{\"success\":true,\"pending\":true}");
      return;
    case ThingSpeakAccountJob::Poll::Missing:
      sendThingSpeakMutation({false, "ThingSpeak operation is no longer available. Reload account channels before retrying."}, true);
      return;
    case ThingSpeakAccountJob::Poll::Ready:
      sendThingSpeakMutation(result, true);
      return;
  }
}

void WebPortal::sendThingSpeakMutation(const ThingSpeakMutationResult& result, bool polled) {
  JsonDocument document;
  document["success"] = result.success;
  document["message"] = result.message;
  document["restart"] = false;
  document["authenticationChanged"] = false;
  if (result.channelId) {
    document["channelId"] = result.channelId;
    document["slot"] = result.slot;
  }
  if (document.overflowed()) {
    sendNoCache(server_);
    server_.send(503, "application/json",
                 "{\"success\":false,\"message\":\"Configuration memory temporarily unavailable\"}");
    return;
  }
  String output;
  output.reserve(1536);
  serializeJson(document, output);
  sendNoCache(server_);
  server_.send(result.success || polled ? 200 : 400, "application/json", output);
}

ThingSpeakMutationResult WebPortal::applyThingSpeakMutation(const ThingSpeakAccountJob::Request& request,
                                                             const ThingSpeakApiResult& result) {
  if (!result.success) return {false, result.error};
  if (request.configResetGeneration != configResetGeneration_)
    return {false, "The cloud request succeeded, but the station reset its settings before local completion."};
  if (strcmp(request.account, config_->thingSpeakUserApiKey))
    return {false, "The cloud request succeeded for the previous account. Local profiles were not changed."};
  using Operation = ThingSpeakAccountJob::Operation;
  switch (request.operation) {
    case Operation::QuickCreate:
    case Operation::Create:
      return importThingSpeakChannel(request, result);
    case Operation::Update:
      return applyThingSpeakUpdate(request, result);
    case Operation::Clear:
      return {true, "All channel measurements were cleared."};
    case Operation::Delete:
      return applyThingSpeakDelete(request.channelId);
    case Operation::List:
      return {false, "Invalid ThingSpeak mutation."};
  }
  return {};
}

void WebPortal::sendThingSpeakChannels(const ThingSpeakApiResult& result) {
  if (!result.success) {
    sendJsonResult(false, result.error);
    return;
  }

  JsonDocument remote(psramJsonAllocator());
  if (deserializeJson(remote, result.response) ||
      !remote.is<JsonArray>()) {
    sendJsonResult(false, "ThingSpeak returned an invalid channel list.");
    return;
  }

  JsonDocument document(psramJsonAllocator());
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
  // Free the parsed upstream response before serializing the station response.
  remote.clear();
  if (document.overflowed()) {
    sendJsonResult(false, "Not enough memory to format ThingSpeak channels.");
    return;
  }
  sendNoCache(server_);
  sendStreamedJsonDocument(server_, document);
}

void WebPortal::createManagedThingSpeakChannel() {
  if (!validManagedChannelRequest(server_)) {
    sendJsonResult(false, "Check the name and channel fields.");
    return;
  }
  ThingSpeakAccountJob::Request request;
  request.operation = ThingSpeakAccountJob::Operation::Create;
  copyText(request.name, server_.arg("name"));
  request.settings = managedChannelSettings(server_);
  runThingSpeakMutation(std::move(request));
}

ThingSpeakMutationResult WebPortal::importThingSpeakChannel(const ThingSpeakAccountJob::Request& request,
                                                             const ThingSpeakApiResult& result) {
  JsonDocument remote(psramJsonAllocator());
  if (deserializeJson(remote, result.response)) {
    return {false, "Channel created, but the response was invalid. Reload account channels before retrying."};
  }
  const uint32_t channelId = remote["id"] | 0U;
  if (!channelId) return {false, "The cloud response contained no Channel ID. Reload account channels before retrying."};
  String readKey;
  String writeKey;
  extractApiKeys(remote.as<JsonObjectConst>(), readKey, writeKey);
  const bool quick = request.operation == ThingSpeakAccountJob::Operation::QuickCreate;
  if (quick && writeKey.isEmpty())
    return {false, "Channel created without a Write API Key in the response. Reload account channels before retrying.", channelId};
  int localSlot = -1;
  for (size_t slot = 0; slot < kMaxThingSpeakChannels; ++slot) {
    if (!config_->thingSpeakChannels[slot].occupied) {
      localSlot = static_cast<int>(slot);
      break;
    }
  }
  if (localSlot >= 0) {
    auto previousConfig = cloneStationConfig(*config_);
    if (!previousConfig) {
      return {false, "Configuration memory temporarily unavailable.", channelId};
    }
    auto& profile = config_->thingSpeakChannels[localSlot];
    profile.occupied = true;
    memcpy(profile.name, request.name, sizeof(profile.name));
    profile.channelId = channelId;
    copyText(profile.readApiKey, readKey);
    copyText(profile.writeApiKey, writeKey);
    if (localSlot == 0) {
      config_->thingSpeakDefaultChannelId = channelId;
      copyText(config_->thingSpeakReadApiKey, readKey);
      copyText(config_->thingSpeakWriteApiKey, writeKey);
    }
    if (!store_->saveStationConfig(*config_)) {
      *config_ = *previousConfig;
      return {false, "ThingSpeak channel " + String(channelId) +
                     " was created, but its local profile could not be saved. Reload account channels before retrying.",
              channelId};
    }
  }
  if (localSlot < 0) return {!quick, "ThingSpeak channel created. The local profile list is full.", channelId};
  if (quick) return {true, "ThingSpeak channel " + String(channelId) + " was created. Assign its fields next.", channelId, localSlot};
  return {true, "ThingSpeak channel created and imported locally.", channelId, localSlot};
}

void WebPortal::updateManagedThingSpeakChannel() {
  uint32_t channelId = 0;
  if (!readUnsignedArgument(server_, "channelId", channelId, 1)) {
    sendJsonResult(false, "Invalid Channel ID"); return;
  }
  if (channelId == 0 || !validManagedChannelRequest(server_)) {
    sendJsonResult(false, "Check the Channel ID, name and fields.");
    return;
  }
  ThingSpeakAccountJob::Request request;
  request.operation = ThingSpeakAccountJob::Operation::Update;
  request.channelId = channelId;
  copyText(request.name, server_.arg("name"));
  request.settings = managedChannelSettings(server_);
  runThingSpeakMutation(std::move(request));
}

ThingSpeakMutationResult WebPortal::applyThingSpeakUpdate(const ThingSpeakAccountJob::Request& request,
                                                           const ThingSpeakApiResult& result) {
  JsonDocument remote(psramJsonAllocator());
  String readKey;
  String writeKey;
  if (!deserializeJson(remote, result.response)) {
    extractApiKeys(remote.as<JsonObjectConst>(), readKey, writeKey);
  }
  auto previousConfig = cloneStationConfig(*config_);
  if (!previousConfig) {
    return {false, "Configuration memory temporarily unavailable."};
  }
  bool changed[kMaxThingSpeakChannels]{};
  bool localChanged = false;
  for (size_t slot = 0; slot < kMaxThingSpeakChannels; ++slot) {
    auto& profile = config_->thingSpeakChannels[slot];
    if (!profile.occupied || profile.channelId != request.channelId) continue;
    memcpy(profile.name, request.name, sizeof(profile.name));
    if (!readKey.isEmpty()) copyText(profile.readApiKey, readKey);
    if (!writeKey.isEmpty()) copyText(profile.writeApiKey, writeKey);
    if (slot == 0) {
      config_->thingSpeakDefaultChannelId = profile.channelId;
      copyText(config_->thingSpeakReadApiKey, profile.readApiKey);
      copyText(config_->thingSpeakWriteApiKey, profile.writeApiKey);
    }
    changed[slot] = true;
    localChanged = true;
  }
  // Save the station profile before applying it to live sensor assignments.
  // A successful cloud mutation remains successful even if this local save
  // fails; only local state can be restored here.
  const bool stationSaved = !localChanged || store_->saveStationConfig(*config_);
  bool localSaved = stationSaved;
  if (stationSaved && localChanged) {
    for (size_t slot = 0; slot < kMaxThingSpeakChannels; ++slot) if (changed[slot]) {
      localSaved = registry_->syncThingSpeakProfile(slot, config_->thingSpeakChannels[slot]) && localSaved;
    }
  }
  if (!localSaved) {
    *config_ = *previousConfig;
    if (stationSaved) {
      // Rollback is best effort when NVS remains unavailable. Registry helpers
      // also retain pending persistence so restored assignments can retry.
      store_->saveStationConfig(*previousConfig);
      for (size_t slot = 0; slot < kMaxThingSpeakChannels; ++slot) if (changed[slot])
        registry_->syncThingSpeakProfile(slot, previousConfig->thingSpeakChannels[slot]);
    }
  }
  return {localSaved, localSaved ? "ThingSpeak channel updated."
                                : "Cloud updated, but the local assignment could not be saved."};
}

void WebPortal::clearManagedThingSpeakChannel() {
  uint32_t channelId = 0;
  if (!readUnsignedArgument(server_, "channelId", channelId, 1)) {
    sendJsonResult(false, "Invalid Channel ID"); return;
  }
  if (channelId == 0) {
    sendJsonResult(false, "Invalid Channel ID.");
    return;
  }
  ThingSpeakAccountJob::Request request;
  request.operation = ThingSpeakAccountJob::Operation::Clear;
  request.channelId = channelId;
  runThingSpeakMutation(std::move(request));
}

void WebPortal::deleteManagedThingSpeakChannel() {
  uint32_t channelId = 0;
  if (!readUnsignedArgument(server_, "channelId", channelId, 1)) {
    sendJsonResult(false, "Invalid Channel ID"); return;
  }
  if (channelId == 0) {
    sendJsonResult(false, "Invalid Channel ID.");
    return;
  }
  ThingSpeakAccountJob::Request request;
  request.operation = ThingSpeakAccountJob::Operation::Delete;
  request.channelId = channelId;
  runThingSpeakMutation(std::move(request));
}

ThingSpeakMutationResult WebPortal::applyThingSpeakDelete(uint32_t channelId) {
  auto previousConfig = cloneStationConfig(*config_);
  if (!previousConfig) {
    return {false, "Configuration memory temporarily unavailable."};
  }
  bool removedSlot[kMaxThingSpeakChannels]{};
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
    removedSlot[slot] = true;
    changed = true;
  }
  const bool stationSaved = !changed || store_->saveStationConfig(*config_);
  bool localSaved = stationSaved;
  if (stationSaved && changed) {
    for (size_t slot = 0; slot < kMaxThingSpeakChannels; ++slot) {
      if (removedSlot[slot]) {
        localSaved = registry_->removeThingSpeakProfile(slot) && localSaved;
      }
    }
  }
  if (!localSaved) {
    *config_ = *previousConfig;
    if (stationSaved) {
      store_->saveStationConfig(*previousConfig);
      for (size_t slot = 0; slot < kMaxThingSpeakChannels; ++slot) {
        if (removedSlot[slot]) {
          registry_->syncThingSpeakProfile(slot, previousConfig->thingSpeakChannels[slot]);
        }
      }
    }
  }
  return {localSaved, localSaved ? "ThingSpeak channel and local assignment deleted."
                                 : "Cloud channel deleted, but the local assignment could not be fully updated."};
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
                  registry_->deleteSensor(mac, server_.arg("pendingOnly") == "1");
  if (ok) otaService.forget(mac);
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
  // Ignore local changes from earlier account jobs even if the reset only
  // partially succeeds, since clearing NVS can succeed before formatting.
  ++configResetGeneration_;
  if (gateway_ != nullptr) gateway_->pauseForReset();
  if (registry_ != nullptr) registry_->resetForFactory();
  dashboardLayout_ = DashboardLayout{};
  dashboardLayoutLoaded_ = false;
  const bool cleared = store_->factoryReset();
  if (cleared) otaService.forget();
  sendJsonResult(cleared,
                 cleared
                     ? "Factory settings cleared. W-Charger is restarting."
                     : "Factory settings could not be cleared.",
                 true);
}

void WebPortal::sendJsonResult(bool success, const String& message,
                               bool restartRequired,
                               bool authenticationChanged) {
  JsonDocument document;
  document["success"] = success;
  document["message"] = message;
  document["restart"] = restartRequired;
  document["authenticationChanged"] = authenticationChanged;
  if (document.overflowed()) {
    sendNoCache(server_);
    server_.send(503, "application/json",
                 "{\"success\":false,\"message\":\"Configuration memory temporarily unavailable\"}");
    return;
  }
  String output;
  output.reserve(1536);
  serializeJson(document, output);
  sendNoCache(server_);
  server_.send(success ? 200 : 400, "application/json", output);
  if (restartRequired) {
    scheduleRestart();
  }
}

void WebPortal::scheduleRestart() {
  restartDelay_.wait(millis(), 2500);
  restartPending_ = true;
}

}  // namespace station

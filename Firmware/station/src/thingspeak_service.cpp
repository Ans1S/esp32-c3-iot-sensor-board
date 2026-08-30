#include "thingspeak_service.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

namespace station {

namespace {
constexpr size_t kUploadQueueSize = 64;
constexpr uint32_t kMinimumWriteIntervalMs = 15000;
constexpr uint32_t kHttpTimeoutMs = 7000;

// DigiCert Global Root G2, valid until 2038-01-15.
constexpr char kThingSpeakRootCa[] PROGMEM = R"CERT(
-----BEGIN CERTIFICATE-----
MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBhMQswCQYDVQQG
EwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3d3cuZGlnaWNlcnQuY29tMSAw
HgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBHMjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUx
MjAwMDBaMGExCzAJBgNVBAYTAlVTMRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3
dy5kaWdpY2VydC5jb20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkq
hkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI2/Ou8jqJ
kTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx1x7e/dfgy5SDN67sH0NO
3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQq2EGnI/yuum06ZIya7XzV+hdG82MHauV
BJVJ8zUtluNJbd134/tJS7SsVQepj5WztCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyM
UNGPHgm+F6HmIcr9g+UQvIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQAB
o0IwQDAPBgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV5uNu
5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY1Yl9PMWLSn/pvtsr
F9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4NeF22d+mQrvHRAiGfzZ0JFrabA0U
WTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NGFdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBH
QRFXGU7Aj64GxJUTFy8bJZ918rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/
iyK5S9kJRaTepLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl
MrY=
-----END CERTIFICATE-----
)CERT";
}  // namespace

bool ThingSpeakService::begin() {
  queue_ = xQueueCreate(kUploadQueueSize, sizeof(CloudUploadJob));
  tlsMutex_ = xSemaphoreCreateMutex();
  if (queue_ == nullptr || tlsMutex_ == nullptr) {
    return false;
  }
  return xTaskCreate(taskEntry, "thingspeak", 10240, this, 1, &task_) == pdPASS;
}

bool ThingSpeakService::queue(
    const SensorConfig& config,
    const lil::protocol::TelemetryPayload& telemetry, int8_t stationRssi,
    uint32_t sequence) {
  if (!config.provisioned || !config.cloudUploadEnabled ||
      config.thingSpeakChannelId == 0 ||
      config.thingSpeakWriteKey[0] == '\0') {
    return false;
  }

  CloudUploadJob job{};
  memcpy(job.mac, config.mac, sizeof(job.mac));
  job.channelId = config.thingSpeakChannelId;
  strlcpy(job.writeKey, config.thingSpeakWriteKey, sizeof(job.writeKey));
  job.stationRssi = stationRssi;
  job.sequence = sequence;
  job.fields = config.thingSpeakFields;
  job.telemetry = telemetry;

  if (xQueueSend(queue_, &job, 0) == pdTRUE) {
    return true;
  }

  CloudUploadJob discarded{};
  xQueueReceive(queue_, &discarded, 0);
  droppedJobs_.fetch_add(1, std::memory_order_relaxed);
  return xQueueSend(queue_, &job, 0) == pdTRUE;
}

void ThingSpeakService::taskEntry(void* context) {
  static_cast<ThingSpeakService*>(context)->taskLoop();
}

void ThingSpeakService::taskLoop() {
  CloudUploadJob job{};
  uint32_t lastWriteMs = 0;
  for (;;) {
    if (xQueueReceive(queue_, &job, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    const uint32_t elapsed = millis() - lastWriteMs;
    if (lastWriteMs != 0 && elapsed < kMinimumWriteIntervalMs) {
      vTaskDelay(pdMS_TO_TICKS(kMinimumWriteIntervalMs - elapsed));
    }

    while (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(5000));
    }

    upload(job);
    lastWriteMs = millis();
  }
}

bool ThingSpeakService::upload(const CloudUploadJob& job) {
  lastAttemptMs_.store(millis(), std::memory_order_relaxed);
  xSemaphoreTake(tlsMutex_, portMAX_DELAY);
  WiFiClientSecure client;
  client.setCACert(kThingSpeakRootCa);
  HTTPClient https;
  https.setConnectTimeout(kHttpTimeoutMs);
  https.setTimeout(kHttpTimeoutMs);

  bool success = false;
  if (https.begin(client, "https://api.thingspeak.com/update.json")) {
    https.addHeader("Content-Type", "application/x-www-form-urlencoded");
    https.addHeader("THINGSPEAKAPIKEY", job.writeKey);

    String body;
    body.reserve(160);
    const auto appendField = [&body](uint8_t field, const String& value) {
      if (field == 0 || field > 8) {
        return;
      }
      if (!body.isEmpty()) {
        body += '&';
      }
      body += "field" + String(field) + '=' + value;
    };
    const bool environmentValid =
        (job.telemetry.flags & lil::protocol::kSensorReadFailed) == 0;
    const bool batteryValid =
        (job.telemetry.flags & lil::protocol::kBatteryReadFailed) == 0;
    const bool freshBsecIaq =
        (job.telemetry.flags & lil::protocol::kBme680RawFallback) == 0;
    if (environmentValid &&
        (job.telemetry.capabilities & lil::protocol::kTemperature) != 0) {
      appendField(job.fields.temperature,
                  String(job.telemetry.temperatureC, 2));
    }
    if (environmentValid &&
        (job.telemetry.capabilities & lil::protocol::kHumidity) != 0) {
      appendField(job.fields.humidity,
                  String(job.telemetry.humidityPercent, 2));
    }
    if (environmentValid &&
        (job.telemetry.capabilities & lil::protocol::kPressure) != 0) {
      appendField(job.fields.pressure,
                  String(job.telemetry.pressureHpa, 2));
    }
    if (environmentValid &&
        freshBsecIaq &&
        (job.telemetry.capabilities & lil::protocol::kIaq) != 0 &&
        job.telemetry.iaqAccuracy > 0) {
      appendField(job.fields.iaq, String(job.telemetry.iaq, 2));
    }
    if (environmentValid &&
        (job.telemetry.capabilities & lil::protocol::kGasResistance) != 0) {
      appendField(job.fields.gasResistance,
                  String(job.telemetry.gasResistanceOhms, 0));
    }
    if (batteryValid &&
        (job.telemetry.capabilities & lil::protocol::kBattery) != 0) {
      appendField(job.fields.battery,
                  String(job.telemetry.batteryMillivolts / 1000.0F, 3));
    }

    if (body.isEmpty()) {
      lastHttpStatus_.store(-2, std::memory_order_relaxed);
      lastEntryId_.store(0, std::memory_order_relaxed);
      failedUploads_.fetch_add(1, std::memory_order_relaxed);
      https.end();
      xSemaphoreGive(tlsMutex_);
      return false;
    }

    const int status = https.POST(body);
    const String response = https.getString();
    lastHttpStatus_.store(status, std::memory_order_relaxed);
    int entryId = 0;
    if (status == HTTP_CODE_OK) {
      JsonDocument responseDocument;
      if (!deserializeJson(responseDocument, response)) {
        entryId = responseDocument["entry_id"] | 0;
      }
      if (entryId == 0) {
        entryId = response.toInt();
      }
    }
    lastEntryId_.store(entryId, std::memory_order_relaxed);
    success = status == HTTP_CODE_OK && entryId > 0;
    if (success) {
      successfulUploads_.fetch_add(1, std::memory_order_relaxed);
      lastSuccessMs_.store(millis(), std::memory_order_relaxed);
    } else {
      failedUploads_.fetch_add(1, std::memory_order_relaxed);
    }
    https.end();
  } else {
    lastHttpStatus_.store(-1, std::memory_order_relaxed);
    lastEntryId_.store(0, std::memory_order_relaxed);
    failedUploads_.fetch_add(1, std::memory_order_relaxed);
  }
  xSemaphoreGive(tlsMutex_);
  return success;
}

ChannelCreationResult ThingSpeakService::createChannel(
    const String& userApiKey, const String& sensorName) {
  ChannelCreationResult result{};
  String settings = "name=" + urlEncode("W-Charger Sensor - " + sensorName);
  settings += "&description=" +
              urlEncode("Automatisch von der W-Charger Station erstellt");
  settings += "&field1=Temperatur+C&field2=Luftfeuchte+Prozent";
  settings += "&field3=Luftdruck+hPa&field4=Static+IAQ";
  settings += "&field5=Batteriespannung+V&field6=Gaswiderstand+Ohm";
  settings += "&public_flag=false";
  const ThingSpeakApiResult api =
      createManagedChannel(userApiKey, settings);
  result.httpStatus = api.httpStatus;
  if (!api.success) {
    result.error = api.error;
    return result;
  }

  JsonDocument document;
  if (deserializeJson(document, api.response)) {
    result.error = "Ungueltige ThingSpeak-Antwort";
    return result;
  }
  result.channelId = document["id"] | 0U;
  for (JsonObject key : document["api_keys"].as<JsonArray>()) {
    if (key["write_flag"] | false) {
      result.writeKey = key["api_key"].as<String>();
    } else {
      result.readKey = key["api_key"].as<String>();
    }
  }
  result.success = result.channelId != 0 && !result.writeKey.isEmpty();
  if (!result.success) {
    result.error = "Channel-ID oder Write-Key fehlt in der Antwort";
  }
  return result;
}

ThingSpeakApiResult ThingSpeakService::listChannels(
    const String& userApiKey) {
  const String url = "https://api.thingspeak.com/channels.json?api_key=" +
                     urlEncode(userApiKey);
  return accountRequest("GET", url, userApiKey);
}

ThingSpeakApiResult ThingSpeakService::createManagedChannel(
    const String& userApiKey, const String& encodedSettings) {
  return accountRequest("POST", "https://api.thingspeak.com/channels.json",
                        userApiKey, encodedSettings);
}

ThingSpeakApiResult ThingSpeakService::updateChannel(
    const String& userApiKey, uint32_t channelId,
    const String& encodedSettings) {
  return accountRequest(
      "PUT", "https://api.thingspeak.com/channels/" + String(channelId) +
                 ".json",
      userApiKey, encodedSettings);
}

ThingSpeakApiResult ThingSpeakService::clearChannel(
    const String& userApiKey, uint32_t channelId) {
  return accountRequest(
      "DELETE", "https://api.thingspeak.com/channels/" + String(channelId) +
                    "/feeds.json",
      userApiKey);
}

ThingSpeakApiResult ThingSpeakService::deleteChannel(
    const String& userApiKey, uint32_t channelId) {
  return accountRequest(
      "DELETE", "https://api.thingspeak.com/channels/" + String(channelId) +
                    ".json",
      userApiKey);
}

ThingSpeakApiResult ThingSpeakService::accountRequest(
    const char* method, const String& url, const String& userApiKey,
    const String& encodedSettings) {
  ThingSpeakApiResult result{};
  if (WiFi.status() != WL_CONNECTED) {
    result.error = "Wi-Fi is not connected";
    return result;
  }
  if (userApiKey.isEmpty()) {
    result.error = "ThingSpeak User API Key fehlt";
    return result;
  }

  xSemaphoreTake(tlsMutex_, portMAX_DELAY);
  WiFiClientSecure client;
  client.setCACert(kThingSpeakRootCa);
  HTTPClient https;
  https.setConnectTimeout(kHttpTimeoutMs);
  https.setTimeout(kHttpTimeoutMs);
  if (!https.begin(client, url)) {
    result.error = "TLS connection could not be started";
    xSemaphoreGive(tlsMutex_);
    return result;
  }
  https.addHeader("Content-Type", "application/x-www-form-urlencoded");
  String body;
  if (strcmp(method, "GET") != 0) {
    body = "api_key=" + urlEncode(userApiKey);
    if (!encodedSettings.isEmpty()) {
      body += '&';
      body += encodedSettings;
    }
  }
  if (strcmp(method, "GET") == 0) {
    result.httpStatus = https.GET();
  } else {
    result.httpStatus = https.sendRequest(method, body);
  }
  result.response = https.getString();
  https.end();
  xSemaphoreGive(tlsMutex_);

  result.success = result.httpStatus >= 200 && result.httpStatus < 300;
  if (!result.success) {
    result.error = "ThingSpeak HTTP " + String(result.httpStatus);
    if (!result.response.isEmpty() && result.response.length() < 180) {
      result.error += ": " + result.response;
    }
  }
  return result;
}

String ThingSpeakService::urlEncode(const String& value) {
  static constexpr char kHex[] = "0123456789ABCDEF";
  String encoded;
  encoded.reserve(value.length() * 2);
  for (size_t i = 0; i < value.length(); ++i) {
    const uint8_t c = static_cast<uint8_t>(value[i]);
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      encoded += static_cast<char>(c);
    } else {
      encoded += '%';
      encoded += kHex[c >> 4U];
      encoded += kHex[c & 0x0FU];
    }
  }
  return encoded;
}

int ThingSpeakService::lastHttpStatus() const {
  return lastHttpStatus_.load(std::memory_order_relaxed);
}

int ThingSpeakService::lastEntryId() const {
  return lastEntryId_.load(std::memory_order_relaxed);
}

uint32_t ThingSpeakService::successfulUploads() const {
  return successfulUploads_.load(std::memory_order_relaxed);
}

uint32_t ThingSpeakService::failedUploads() const {
  return failedUploads_.load(std::memory_order_relaxed);
}

uint32_t ThingSpeakService::lastAttemptMs() const {
  return lastAttemptMs_.load(std::memory_order_relaxed);
}

uint32_t ThingSpeakService::lastSuccessMs() const {
  return lastSuccessMs_.load(std::memory_order_relaxed);
}

uint32_t ThingSpeakService::droppedJobs() const {
  return droppedJobs_.load(std::memory_order_relaxed);
}

}  // namespace station

#pragma once
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_partition.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "ota_protocol.h"

namespace station {
class OtaService {
 public:
  bool begin();
  bool startUpload(const uint8_t mac[6], String& error);
  bool upload(const uint8_t* bytes, size_t size);
  bool finishUpload(String& error);
  void abortUpload();
  bool cancel();
  void forget(const uint8_t* mac = nullptr);
  bool respond(const uint8_t mac[6], const lil::ota::Packet& request,
               lil::ota::Packet& response);
  void json(JsonObject output);
 private:
  struct Job {
    uint32_t magic = 0;
    uint8_t mac[6]{};
    lil::ota::Manifest manifest{};
    lil::ota::State state = lil::ota::State::Waiting;
    lil::ota::Failure failure = lil::ota::Failure::None;
    uint32_t confirmed = 0;
  } job_;
  struct Node { uint8_t mac[6]{}; lil::ota::Identity identity{}; } nodes_[19]{};
  bool save();
  SemaphoreHandle_t mutex_ = nullptr;
  Preferences preferences_;
  const esp_partition_t* partition_ = nullptr;
  bool uploading_ = false;
  bool uploadFailed_ = false;
  uint32_t written_ = 0;
  uint32_t erased_ = 0;
  uint32_t lastRequestMs_ = 0;
  uint32_t activeNonce_ = 0;
};
extern OtaService otaService;
}

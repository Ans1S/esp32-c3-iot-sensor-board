#pragma once

#include <Arduino.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "app_config.h"
#include "lil_protocol.h"

namespace station {

struct CloudUploadJob {
  uint8_t mac[6]{};
  uint32_t channelId = 0;
  char writeKey[33]{};
  int8_t stationRssi = 0;
  uint32_t sequence = 0;
  ThingSpeakFieldMapping fields{};
  lil::protocol::TelemetryPayload telemetry{};
};

struct ChannelCreationResult {
  bool success = false;
  int httpStatus = 0;
  uint32_t channelId = 0;
  String readKey;
  String writeKey;
  String error;
};

struct ThingSpeakApiResult {
  bool success = false;
  int httpStatus = 0;
  String response;
  String error;
};

class ThingSpeakService {
 public:
  bool begin();
  bool queue(const SensorConfig& config,
             const lil::protocol::TelemetryPayload& telemetry,
             int8_t stationRssi, uint32_t sequence);
  ChannelCreationResult createChannel(const String& userApiKey,
                                      const String& sensorName);
  ThingSpeakApiResult listChannels(const String& userApiKey);
  ThingSpeakApiResult createManagedChannel(const String& userApiKey,
                                           const String& encodedSettings);
  ThingSpeakApiResult updateChannel(const String& userApiKey,
                                    uint32_t channelId,
                                    const String& encodedSettings);
  ThingSpeakApiResult clearChannel(const String& userApiKey,
                                   uint32_t channelId);
  ThingSpeakApiResult deleteChannel(const String& userApiKey,
                                    uint32_t channelId);
  static String urlEncode(const String& value);
  int lastHttpStatus() const;
  int lastEntryId() const;
  uint32_t successfulUploads() const;
  uint32_t failedUploads() const;
  uint32_t lastAttemptMs() const;
  uint32_t lastSuccessMs() const;
  uint32_t droppedJobs() const;

 private:
  static void taskEntry(void* context);
  void taskLoop();
  bool upload(const CloudUploadJob& job);
  ThingSpeakApiResult accountRequest(const char* method, const String& url,
                                     const String& userApiKey,
                                     const String& encodedSettings = "");

  QueueHandle_t queue_ = nullptr;
  SemaphoreHandle_t tlsMutex_ = nullptr;
  TaskHandle_t task_ = nullptr;
  std::atomic<int> lastHttpStatus_{0};
  std::atomic<int> lastEntryId_{0};
  std::atomic<uint32_t> successfulUploads_{0};
  std::atomic<uint32_t> failedUploads_{0};
  std::atomic<uint32_t> lastAttemptMs_{0};
  std::atomic<uint32_t> lastSuccessMs_{0};
  std::atomic<uint32_t> droppedJobs_{0};
};

}  // namespace station

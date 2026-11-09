#pragma once

#include <Arduino.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "app_config.h"
#include "lil_protocol.h"
#include "thingspeak_account_job.h"
#include "upload_scheduler.h"

namespace station {
class SensorRegistry;

struct CloudUploadJob {
  uint32_t receivedAt = 0;
  uint32_t generation = 0;
  bool sharedChannel = false;
  uint8_t mac[6]{};
  uint32_t channelId = 0;
  char writeKey[33]{};
  int8_t stationRssi = 0;
  uint32_t sequence = 0;
  ThingSpeakFieldMapping fields{};
  lil::protocol::TelemetryPayload telemetry{};
};

class ThingSpeakService {
 public:
  bool begin(SensorRegistry& registry);
  bool queue(const SensorConfig& config,
             const lil::protocol::TelemetryPayload& telemetry,
             int8_t stationRssi, uint32_t sequence, uint32_t receivedAt,
             bool sharedChannel = false, uint32_t generation = 0);
  ThingSpeakApiResult executeAccount(const ThingSpeakAccountJob::Request& request);
  uint32_t startAccountJob(ThingSpeakAccountJob::Request request, const char* owner);
  template <typename Result>
  ThingSpeakAccountJob::Poll collectAccountJob(uint32_t ticket, const char* account,
                                              const char* owner, Result& result) {
    if (!queueMutex_) return ThingSpeakAccountJob::Poll::Missing;
    xSemaphoreTake(queueMutex_, portMAX_DELAY);
    const auto state = accountJob_.collect(ticket, account, owner, millis(), result);
    xSemaphoreGive(queueMutex_);
    return state;
  }
  bool takeMutation(ThingSpeakAccountJob::Request& request, ThingSpeakApiResult& result);
  bool mutationReady() const { return mutationReady_.load(std::memory_order_acquire); }
  void finishMutation(ThingSpeakMutationResult result);
  bool mutationPending();
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
  bool ensureTask();
  void taskLoop();
  bool upload(const CloudUploadJob& job);
  ThingSpeakApiResult accountRequest(const char* method, const String& url,
                                     const String& userApiKey,
                                     const String& encodedSettings = "");

  lil::UploadScheduler<CloudUploadJob, 64> scheduler_;
  ThingSpeakAccountJob accountJob_;
  SensorRegistry* registry_ = nullptr;
  SemaphoreHandle_t queueMutex_ = nullptr;
  SemaphoreHandle_t tlsMutex_ = nullptr;
  TaskHandle_t task_ = nullptr;
  std::atomic<bool> mutationReady_{false};
  std::atomic<int> lastHttpStatus_{0};
  std::atomic<int> lastEntryId_{0};
  std::atomic<uint32_t> successfulUploads_{0};
  std::atomic<uint32_t> failedUploads_{0};
  std::atomic<uint32_t> lastAttemptMs_{0};
  std::atomic<uint32_t> lastSuccessMs_{0};
  std::atomic<uint32_t> droppedJobs_{0};
};

}  // namespace station

#pragma once

#include <Arduino.h>
#include <atomic>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "lil_protocol.h"
#include "sensor_registry.h"
#include "thingspeak_service.h"
#include "wifi_service.h"

namespace station {

struct EspNowRxEvent {
  uint32_t receivedAt = 0;
  uint8_t sourceMac[6]{};
  int8_t rssi = 0;
  uint16_t length = 0;
  uint8_t data[lil::protocol::kMaxPacketSize]{};
};

struct EspNowPersistenceEvent {
  uint32_t receivedAt = 0;
  uint32_t generation = 0;
  uint8_t sourceMac[6]{};
  uint32_t sequence = 0;
  int8_t rssi = 0;
  lil::protocol::TelemetryPayload telemetry{};
  SensorConfig responseConfig{};
  bool queueCloudUpload = false;
};

class EspNowGateway {
 public:
  bool begin(SensorRegistry& registry, ThingSpeakService& thingSpeak,
             WifiService& wifi);
  uint32_t receivedPackets() const;
  uint32_t invalidPackets() const;
  uint32_t droppedPackets() const;
  uint32_t persistenceDrops() const { return persistenceDrops_.load(); }
  uint32_t peerFailures() const { return peerFailures_.load(); }

 private:
  static void receiveCallback(const esp_now_recv_info_t* info,
                              const uint8_t* data, int length);
  static void sendCallback(const wifi_tx_info_t* info,
                           esp_now_send_status_t status);
  static void taskEntry(void* context);
  static void persistenceTaskEntry(void* context);
  void taskLoop();
  void persistenceTaskLoop();
  void handle(const EspNowRxEvent& event);
  void persist(const EspNowPersistenceEvent& event);
  bool ensurePeer(const uint8_t mac[6]);

  static EspNowGateway* instance_;
  SensorRegistry* registry_ = nullptr;
  ThingSpeakService* thingSpeak_ = nullptr;
  WifiService* wifi_ = nullptr;
  QueueHandle_t receiveQueue_ = nullptr;
  QueueHandle_t persistenceQueue_ = nullptr;
  TaskHandle_t task_ = nullptr;
  TaskHandle_t persistenceTask_ = nullptr;
  std::atomic<bool> sendPending_{false};
  std::atomic<uint32_t> persistenceDrops_{0};
  std::atomic<uint32_t> peerFailures_{0};
  std::atomic<uint32_t> receivedPackets_{0};
  std::atomic<uint32_t> invalidPackets_{0};
  std::atomic<uint32_t> droppedPackets_{0};
};

}  // namespace station

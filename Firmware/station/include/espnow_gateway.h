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
  uint8_t sourceMac[6]{};
  int8_t rssi = 0;
  uint16_t length = 0;
  uint8_t data[lil::protocol::kMaxPacketSize]{};
};

class EspNowGateway {
 public:
  bool begin(SensorRegistry& registry, ThingSpeakService& thingSpeak,
             WifiService& wifi);
  uint32_t receivedPackets() const;
  uint32_t invalidPackets() const;
  uint32_t droppedPackets() const;

 private:
  static void receiveCallback(const esp_now_recv_info_t* info,
                              const uint8_t* data, int length);
  static void taskEntry(void* context);
  void taskLoop();
  void handle(const EspNowRxEvent& event);
  bool ensurePeer(const uint8_t mac[6]);

  static EspNowGateway* instance_;
  SensorRegistry* registry_ = nullptr;
  ThingSpeakService* thingSpeak_ = nullptr;
  WifiService* wifi_ = nullptr;
  QueueHandle_t receiveQueue_ = nullptr;
  TaskHandle_t task_ = nullptr;
  std::atomic<uint32_t> receivedPackets_{0};
  std::atomic<uint32_t> invalidPackets_{0};
  std::atomic<uint32_t> droppedPackets_{0};
};

}  // namespace station

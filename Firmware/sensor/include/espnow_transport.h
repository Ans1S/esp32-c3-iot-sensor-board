#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "lil_protocol.h"
#include "ota_protocol.h"
#include "sensor_config_store.h"

namespace sensor {

struct ExchangeResult {
  bool delivered = false;
  bool configReceived = false;
  lil::protocol::ConfigResponsePayload config{};
  int8_t stationRssi = 0;
};

class EspNowTransport {
 public:
  bool begin();
  ExchangeResult exchange(const lil::protocol::TelemetryPacket& packet,
                          const SensorRuntimeConfig& config,
                          bool trialBoot = false);
  ExchangeResult exchangeLpChannel(
      const lil::protocol::TelemetryPacket& packet,
      const SensorRuntimeConfig& config, uint8_t channel,
      bool recoveryAttempt = false);
  void end();
  bool otaExchange(const uint8_t mac[6], const lil::ota::Packet& request,
                   lil::ota::Packet& response);

 private:
  static void sendCallback(const wifi_tx_info_t* info,
                           esp_now_send_status_t status);
  static void receiveCallback(const esp_now_recv_info_t* info,
                              const uint8_t* data, int length);
  bool tryChannel(uint8_t channel, const uint8_t destination[6],
                   const lil::protocol::TelemetryPacket& packet,
                   ExchangeResult& result, int8_t baseTxPowerQuarterDbm,
                   uint8_t attempts, bool forceMaximumPower = false);
  bool ensurePeer(const uint8_t mac[6]);

  static EspNowTransport* instance_;
  struct ResponseEvent {
    lil::protocol::ConfigResponsePacket packet{};
    uint8_t sourceMac[6]{};
    int8_t rssi = 0;
  };
  bool receiveResponse(ExchangeResult& result, const uint8_t destination[6],
                       TickType_t waitTicks);
  QueueHandle_t responses_ = nullptr;
  struct OtaEvent { uint8_t mac[6]; lil::ota::Packet packet; };
  QueueHandle_t otaResponses_ = nullptr;
  // Release/acquire publishes completion without racing the Wi-Fi task.
  std::atomic<uint8_t> sendState_{0};  // idle, pending, success, failure
  uint32_t expectedSequence_ = 0;

};

}  // namespace sensor

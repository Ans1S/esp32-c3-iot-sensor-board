#pragma once

#include <Arduino.h>
#include <esp_now.h>

#include "lil_protocol.h"
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
                          const SensorRuntimeConfig& config);
  ExchangeResult exchangeLpChannel(
      const lil::protocol::TelemetryPacket& packet,
      const SensorRuntimeConfig& config, uint8_t channel,
      bool recoveryAttempt = false);
  void end();

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
  volatile bool sendFinished_ = false;
  volatile bool sendSucceeded_ = false;
  volatile bool responseReady_ = false;
  uint32_t expectedSequence_ = 0;
  lil::protocol::ConfigResponsePayload response_{};
  int8_t responseRssi_ = 0;
};

}  // namespace sensor

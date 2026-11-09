#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "lil_protocol.h"
#include "ota_protocol.h"
#include "recording_protocol.h"
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
  // Callers provide payload and header.sequence. Each actual send constructs
  // its final compatible wire header/CRC after refreshing relative ages.
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
  bool recordingExchange(const SensorRuntimeConfig& config, const void* request,
      size_t length, uint64_t session, uint32_t sampleMs, uint32_t crc,
      lil::recording::Ack& response);

 private:
  static void sendCallback(const wifi_tx_info_t* info,
                           esp_now_send_status_t status);
  static void receiveCallback(const esp_now_recv_info_t* info,
                              const uint8_t* data, int length);
  struct TelemetryFrame {
    lil::protocol::PacketHeader header;
    uint8_t payload[sizeof(lil::protocol::TelemetryPayload)];
  };
  struct PreparedTelemetry {
    explicit PreparedTelemetry(uint32_t now) : startedMs(now) {}
    TelemetryFrame frame;
    uint32_t startedMs;
    size_t size = 0;
  };
  bool tryChannel(uint8_t channel, const uint8_t destination[6],
                   const lil::protocol::TelemetryPacket& packet,
                   PreparedTelemetry& prepared,
                   ExchangeResult& result, int8_t baseTxPowerQuarterDbm,
                   uint8_t attempts, bool forceMaximumPower = false);
  void waitForTelemetrySend(ExchangeResult& result, const uint8_t destination[6]);
  bool sendFrame(const uint8_t destination[6], const void* data, size_t size,
                 bool telemetry = false);
  bool sendPending() const;
  bool ensurePeer(const uint8_t mac[6]);

  static EspNowTransport* instance_;
  struct ResponseEvent {
    lil::protocol::ConfigResponsePacket packet{};
    uint8_t sourceMac[6]{};
    int8_t rssi = 0;
    bool completion = false;
  };
  bool receiveResponse(ExchangeResult& result, const uint8_t destination[6],
                       TickType_t waitTicks);
  QueueHandle_t responses_ = nullptr;
  struct OtaEvent { uint8_t mac[6]; lil::ota::Packet packet; };
  // Publish lazy queues only after initialization; Wi-Fi callbacks acquire them.
  std::atomic<QueueHandle_t> otaResponses_{nullptr};
  struct RecordingEvent { uint8_t mac[6]; lil::recording::AckPacket packet; };
  std::atomic<QueueHandle_t> recordingResponses_{nullptr};
  // Release/acquire publishes completion without racing the Wi-Fi task.
  enum class SendState : uint8_t { kIdle, kTelemetry, kAuxiliary, kSuccess, kFailure };
  std::atomic<SendState> sendState_{SendState::kIdle};
  uint32_t sendStartedMs_ = 0;
  uint32_t expectedSequence_ = 0;

};

}  // namespace sensor

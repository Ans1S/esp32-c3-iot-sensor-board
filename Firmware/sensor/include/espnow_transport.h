#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#ifdef ESP_PLATFORM
#include <freertos/task.h>
#endif

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
                          bool trialBoot = false,
                          bool liveStream = false);
  ExchangeResult exchangeLpChannel(
      const lil::protocol::TelemetryPacket& packet,
      const SensorRuntimeConfig& config, uint8_t channel,
      bool recoveryAttempt = false);
  void end();
  bool sendStalled(uint32_t nowMs, uint32_t timeoutMs = 5000) const;
  bool configureLink(uint8_t channel, int8_t txPowerQuarterDbm);
  bool otaExchange(const uint8_t mac[6], const lil::ota::Packet& request,
                   lil::ota::Packet& response);
  bool otaExchangeLarge(const uint8_t mac[6], const lil::ota::Packet& request,
                        lil::ota::LargePacket& response);
  bool recordingExchange(const SensorRuntimeConfig& config, const void* request,
      size_t length, uint64_t session, uint32_t sampleMs, uint32_t crc,
      lil::recording::Ack& response, uint32_t responseWaitMs = 0);

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
                   uint8_t attempts, bool forceMaximumPower = false,
                   uint32_t maxResponseWaitMs = 0);
  void waitForTelemetrySend(ExchangeResult& result, const uint8_t destination[6]);
  bool waitForAuxiliarySend(uint32_t maxWaitMs, bool relativeToSendStart);
  // Release/acquire publishes completion without racing the Wi-Fi task.
  enum class SendState : uint8_t {
    kIdle,
    kTelemetry,
    kOta,
    kRecording,
    kAuxiliary,
    kSuccess,
    kFailure,
  };
  bool sendFrame(const uint8_t destination[6], const void* data, size_t size,
                 SendState mode = SendState::kAuxiliary);
  bool sendPending() const;
  bool ensurePeer(const uint8_t mac[6]);
  bool setChannelCached(uint8_t channel);
  void setTxPowerCached(int8_t txPowerQuarterDbm);
  void setPeerRate(const uint8_t mac[6], bool highRate);

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
  struct OtaEvent { uint8_t mac[6]{}; lil::ota::Packet packet{}; bool completion = false; };
  // Publish lazy queues only after initialization; Wi-Fi callbacks acquire them.
  std::atomic<QueueHandle_t> otaResponses_{nullptr};
  struct RecordingEvent { uint8_t mac[6]{}; lil::recording::AckPacket packet{}; bool completion = false; };
  std::atomic<QueueHandle_t> recordingResponses_{nullptr};
  std::atomic<SendState> sendState_{SendState::kIdle};
#ifdef ESP_PLATFORM
  std::atomic<TaskHandle_t> auxWaiter_{nullptr};
  std::atomic<TaskHandle_t> otaLargeNotify_{nullptr};
  portMUX_TYPE otaLargeMux_ = portMUX_INITIALIZER_UNLOCKED;
  TaskHandle_t otaLargeWaiter_ = nullptr;
  lil::ota::LargePacket* otaLargeTarget_ = nullptr;
  uint8_t otaLargeMac_[6]{};
  uint32_t otaLargeSeq_ = 0;
  uint32_t otaLargeNonce_ = 0;
  bool otaLargeReady_ = false;
  bool lightSleepInhibited_ = false;
#endif
  uint32_t sendStartedMs_ = 0;
  uint32_t expectedSequence_ = 0;
  uint8_t currentChannel_ = 0;
  int8_t currentTxPower_ = 0;
  int8_t baseTxPower_ = 52;
  int8_t otaTxPower_ = 0;
  uint8_t recordingFailures_ = 0;
#ifdef ESP_PLATFORM
  bool peerHighRate_ = false;
  bool otaLowRate_ = false;
  bool recordingLowRate_ = false;
  uint8_t ratedPeerMac_[6]{};
#endif

};

}  // namespace sensor

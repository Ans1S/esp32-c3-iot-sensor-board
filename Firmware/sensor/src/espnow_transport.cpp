#include "espnow_transport.h"

#include <WiFi.h>
#include <esp_random.h>
#include <esp_wifi.h>
#include <freertos/task.h>

#include "sensor_log.h"
#include "radio_benchmark.h"

namespace sensor {

EspNowTransport* EspNowTransport::instance_ = nullptr;

namespace {
constexpr uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF,
                                      0xFF, 0xFF, 0xFF};
constexpr uint32_t kSendWaitMs = 180;
constexpr uint32_t kResponseWaitMs = 260;
constexpr TickType_t kOtaResponseWait = pdMS_TO_TICKS(700);
constexpr TickType_t kRecordingResponseWait = pdMS_TO_TICKS(500);
constexpr uint8_t kDiscoveryChannelStride = 4;
constexpr uint8_t kKnownChannelAttempts = 2;
constexpr uint8_t kBroadcastAttempts = 1;
constexpr uint8_t kLpNormalAttempts = 2;
constexpr uint8_t kLpRecoveryAttempts = 1;
constexpr int8_t kTxPowerStepQuarterDbm = 12;  // 3 dBm per retry.
constexpr int8_t kMaximumTxPowerQuarterDbm = 84;  // API maximum: 20 dBm.

int8_t txPowerForAttempt(int8_t baseQuarterDbm, uint8_t attempt,
                         bool broadcast) {
  const int8_t base = constrain(baseQuarterDbm, 8, kMaximumTxPowerQuarterDbm);
  if (broadcast || attempt == 0) {
    return base;
  }
  return static_cast<int8_t>(
      min(static_cast<int>(kMaximumTxPowerQuarterDbm),
          static_cast<int>(base) +
              static_cast<int>(attempt) * kTxPowerStepQuarterDbm));
}
}

bool EspNowTransport::begin() {
  responses_ = xQueueCreate(4, sizeof(ResponseEvent));
  if (responses_ == nullptr) return false;
  instance_ = this;
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);
  WiFi.setSleep(false);
  if (esp_now_init() != ESP_OK) {
    end();
    return false;
  }
  if (esp_now_register_send_cb(sendCallback) != ESP_OK ||
      esp_now_register_recv_cb(receiveCallback) != ESP_OK) {
    end();
    return false;
  }
  if (!ensurePeer(kBroadcastMac)) {
    end();
    return false;
  }
  return true;
}

void EspNowTransport::sendCallback(const wifi_tx_info_t* info,
                                   esp_now_send_status_t status) {
  (void)info;
  if (instance_ != nullptr) {
    const auto previous = instance_->sendState_.load(std::memory_order_acquire);
    instance_->sendState_.store(status == ESP_NOW_SEND_SUCCESS ?
        SendState::kSuccess : SendState::kFailure, std::memory_order_release);
    if (previous == SendState::kTelemetry) {
      ResponseEvent event{};
      event.completion = true;
      xQueueSend(instance_->responses_, &event, 0);
    }
  }
}

void EspNowTransport::receiveCallback(const esp_now_recv_info_t* info,
                                      const uint8_t* data, int length) {
  lil::protocol::MessageType type;
  const size_t received = static_cast<size_t>(length);
  if (!instance_ || !info || !info->src_addr || !lil::protocol::readPacketType(data, received, type)) return;
  if (type == lil::recording::kAckMessage && received == sizeof(lil::recording::AckPacket)) {
    const auto responses = instance_->recordingResponses_.load(std::memory_order_acquire);
    if (!responses || !lil::protocol::validatePacket(data, received, type, sizeof(lil::recording::Ack))) return;
    RecordingEvent event{};
    memcpy(event.mac, info->src_addr, 6); memcpy(&event.packet, data, sizeof(event.packet));
    xQueueSend(responses, &event, 0);
    return;
  }
  if (type == lil::ota::kMessageType && received == sizeof(lil::ota::Packet)) {
    const auto responses = instance_->otaResponses_.load(std::memory_order_acquire);
    if (!responses) return;
    OtaEvent event{};
    memcpy(event.mac, info->src_addr, 6);
    memcpy(&event.packet, data, sizeof(event.packet));
    xQueueSend(responses, &event, 0);
    return;
  }
  if (type != lil::protocol::MessageType::kConfigResponse ||
      received != sizeof(lil::protocol::ConfigResponsePacket)) {
    return;
  }

  ResponseEvent event{};
  memcpy(&event.packet, data, sizeof(event.packet));
  memcpy(event.sourceMac, info->src_addr, 6);
  event.rssi = info->rx_ctrl != nullptr ? info->rx_ctrl->rssi : 0;
  xQueueSend(instance_->responses_, &event, 0);
}

bool EspNowTransport::receiveResponse(ExchangeResult& result,
                                      const uint8_t destination[6],
                                      TickType_t waitTicks) {
  ResponseEvent event{};
  while (xQueueReceive(responses_, &event, waitTicks) == pdTRUE) {
    waitTicks = 0;
    if (event.completion) continue;
    if (!lil::protocol::validate(event.packet, sizeof(event.packet),
                                  lil::protocol::MessageType::kConfigResponse) ||
        event.packet.payload.requestSequence != expectedSequence_ ||
        memcmp(event.sourceMac, event.packet.payload.stationMac, 6) != 0 ||
        (memcmp(destination, kBroadcastMac, 6) != 0 &&
         memcmp(destination, event.sourceMac, 6) != 0)) {
      continue;
    }
    result.configReceived = true;
    result.config = event.packet.payload;
    result.stationRssi = event.rssi;
    return true;
  }
  return false;
}

bool EspNowTransport::sendPending() const {
  const auto state = sendState_.load(std::memory_order_acquire);
  return state == SendState::kTelemetry || state == SendState::kAuxiliary;
}

bool EspNowTransport::sendFrame(const uint8_t destination[6], const void* data,
                                size_t size, bool telemetry) {
  sendStartedMs_ = millis();
  sendState_.store(telemetry ? SendState::kTelemetry : SendState::kAuxiliary,
                   std::memory_order_release);
  if (esp_now_send(destination, static_cast<const uint8_t*>(data), size) == ESP_OK) return true;
  sendState_.store(SendState::kIdle, std::memory_order_release);
  return false;
}

void EspNowTransport::waitForTelemetrySend(ExchangeResult& result,
                                           const uint8_t destination[6]) {
  for (;;) {
    const auto state = sendState_.load(std::memory_order_acquire);
    if (state != SendState::kTelemetry && state != SendState::kAuxiliary) return;
    const uint32_t elapsed = millis() - sendStartedMs_;
    if (elapsed >= kSendWaitMs) return;
    // Telemetry completion wakes this queue even if no application ACK arrives.
    // An earlier OTA/recording send does not post to this queue.
    const uint32_t remaining = kSendWaitMs - elapsed;
    const uint32_t wait = state == SendState::kTelemetry ? remaining : min(remaining, uint32_t(10));
    if (receiveResponse(result, destination, pdMS_TO_TICKS(wait))) return;
  }
}

ExchangeResult EspNowTransport::exchange(
    const lil::protocol::TelemetryPacket& packet,
    const SensorRuntimeConfig& config, bool trialBoot) {
  ExchangeResult result{};
  PreparedTelemetry prepared(millis());
  expectedSequence_ = packet.header.sequence;
  const bool configured = config.provisioned && config.stationKnown;
  const uint8_t* destination =
      configured ? config.stationMac : kBroadcastMac;

  if (tryChannel(config.wifiChannel, destination, packet, prepared, result,
                 config.txPowerQuarterDbm,
                 configured ? kKnownChannelAttempts : kBroadcastAttempts)) {
    return result;
  }

  // Configured sensors keep the normal radio window short. The caller decides
  // whether this is a normal bounded recovery or the one-time full recovery
  // needed while the station moves from its setup AP to the home Wi-Fi channel.
  if (configured) {
    // Trial boot needs an application ACK, not just a MAC ACK. Allow a busy
    // station a bounded retry window before the caller rolls back the image.
    const uint32_t trialStarted = millis();
    while (trialBoot && millis() - trialStarted < 5000UL) {
      if (sendPending()) break;
      if (receiveResponse(result, destination, pdMS_TO_TICKS(50))) return result;
      if (tryChannel(config.wifiChannel, destination, packet, prepared, result,
                     config.txPowerQuarterDbm, 1)) return result;
    }
    return result;
  }

  const uint8_t scanStart =
      static_cast<uint8_t>((packet.header.sequence *
                            kDiscoveryChannelStride) % 13U) +
      1U;
  // An unprovisioned sensor scans the complete 2.4 GHz channel range during
  // every discovery wake. Only the wake interval becomes slower after the
  // initial pairing window; channel coverage must not become incomplete.
  for (uint8_t offset = 0; offset < 13; ++offset) {
    const uint8_t channel =
        static_cast<uint8_t>((scanStart - 1U + offset) % 13U) + 1U;
    if (channel == config.wifiChannel) {
      continue;
    }
    if (tryChannel(channel, kBroadcastMac, packet, prepared, result,
                   config.txPowerQuarterDbm, kBroadcastAttempts)) {
      return result;
    }
  }
  return result;
}

ExchangeResult EspNowTransport::exchangeLpChannel(
    const lil::protocol::TelemetryPacket& packet,
    const SensorRuntimeConfig& config, uint8_t channel,
    bool recoveryAttempt) {
  ExchangeResult result{};
  if (!config.provisioned || !config.stationKnown || channel < 1 ||
      channel > 13) {
    return result;
  }
  expectedSequence_ = packet.header.sequence;
  PreparedTelemetry prepared(millis());
  // Keep each radio window short. Normal reports get two attempts on the known
  // channel. Recovery scans use one maximum-power attempt per scheduled report
  // and therefore do not unnecessarily extend the active time.
  tryChannel(channel, config.stationMac, packet, prepared, result,
             config.txPowerQuarterDbm,
             recoveryAttempt ? kLpRecoveryAttempts : kLpNormalAttempts,
             recoveryAttempt);
  return result;
}

bool EspNowTransport::tryChannel(
    uint8_t channel, const uint8_t destination[6],
    const lil::protocol::TelemetryPacket& packet, PreparedTelemetry& prepared,
    ExchangeResult& result,
    int8_t baseTxPowerQuarterDbm, uint8_t attempts,
    bool forceMaximumPower) {
  // Never retune or enqueue another frame while its callback is outstanding.
  waitForTelemetrySend(result, destination);
  if (result.configReceived || receiveResponse(result, destination, 0)) return true;
  if (sendPending()) return false;
  if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
      !ensurePeer(destination)) {
    return false;
  }
  // Environmental telemetry never needs the IMU suffix. Preserve the V5
  // wire size so an existing 4.0.0 station can acknowledge a trial boot.
  static_assert(sizeof(TelemetryFrame) == sizeof(packet));
  static_assert(offsetof(TelemetryFrame, payload) == sizeof(lil::protocol::PacketHeader));
  const bool motion = packet.payload.sensorType == lil::protocol::EnvironmentalSensorType::kLsm6dsox ||
                      (packet.payload.capabilities & lil::protocol::kMotion) != 0;
  const bool optical = packet.payload.sensorType == lil::protocol::EnvironmentalSensorType::kMax30102 ||
      (packet.payload.capabilities & (lil::protocol::kOptical | lil::protocol::kHeartRate)) != 0;
  const size_t payloadSize = (optical || lil::protocol::isLiveSensor(packet.payload.sensorType)) ? sizeof(packet.payload) : motion ?
      lil::protocol::kMotionTelemetryPayloadSize : lil::protocol::kLegacyTelemetryPayloadSize;
  const size_t wireSize = sizeof(lil::protocol::PacketHeader) + payloadSize;
  const bool broadcast = memcmp(destination, kBroadcastMac, 6) == 0;
  for (uint8_t attempt = 0; attempt < attempts; ++attempt) {
    const int8_t txPower = forceMaximumPower
                               ? kMaximumTxPowerQuarterDbm
                               : txPowerForAttempt(baseTxPowerQuarterDbm,
                                                   attempt, broadcast);
    if (esp_wifi_set_max_tx_power(txPower) != ESP_OK) {
      SENSOR_LOG_PRINTF(
          "[ESP-NOW] Could not set requested TX power %d.%02d dBm\n",
          txPower / 4, (abs(txPower) % 4) * 25);
    }
    if (attempt > 0) {
      if (receiveResponse(result, destination,
                          pdMS_TO_TICKS(8U + (esp_random() % 35U)))) return true;
    }
    if (receiveResponse(result, destination, 0)) return true;
    const bool refreshAge = payloadSize == sizeof(packet.payload);
    auto& wirePacket = prepared.frame;
    // Short frames have no ages and remain byte-identical on every retry.
    if (prepared.size == 0) memcpy(&wirePacket, &packet, wireSize);
    if (refreshAge) {
      const uint32_t waitMs = millis() - prepared.startedMs;
      auto addAge = [waitMs](uint16_t age) { return age == UINT16_MAX ? age : static_cast<uint16_t>(min(uint32_t(65534), uint32_t(age) + waitMs)); };
      auto setAge = [&](size_t offset, uint16_t age) {
        const uint16_t updated = addAge(age);
        memcpy(wirePacket.payload + offsetof(lil::protocol::TelemetryPayload, live) + offset,
               &updated, sizeof(updated));
      };
      setAge(offsetof(lil::protocol::LiveReading, acquisitionAgeMs), packet.payload.live.acquisitionAgeMs);
      setAge(offsetof(lil::protocol::LiveReading, estimateAgeMs), packet.payload.live.estimateAgeMs);
      for (uint8_t i = 0; i < min(size_t(packet.payload.live.count), lil::protocol::kOpticalBatchSize); ++i)
        setAge(offsetof(lil::protocol::LiveReading, optical) + i * sizeof(lil::protocol::OpticalFrame) +
               offsetof(lil::protocol::OpticalFrame, ageMs), packet.payload.live.optical[i].ageMs);
    }
    if (prepared.size == 0 || refreshAge) {
      lil::protocol::finalizePacket(&wirePacket, wireSize, lil::protocol::MessageType::kTelemetry,
                                    packet.header.sequence, payloadSize);
      prepared.size = wireSize;
    }
    if (!sendFrame(destination, &wirePacket, wireSize, true)) continue;
    WCH_RADIO_BENCHMARK_FRAME();

    waitForTelemetrySend(result, destination);
    const auto state = sendState_.load(std::memory_order_acquire);
    result.delivered = result.delivered || state == SendState::kSuccess;
    if (result.configReceived) return true;
    // A missing MAC ACK can accompany a valid application response. Retain a
    // short grace window after failure, and the full window after delivery.
    const uint32_t responseWaitMs = !broadcast && state == SendState::kFailure ? 30 : kResponseWaitMs;
    const uint32_t responseStarted = millis();
    for (;;) {
      const uint32_t elapsed = millis() - responseStarted;
      if (elapsed >= responseWaitMs) break;
      const uint32_t remaining = responseWaitMs - elapsed;
      if (receiveResponse(result, destination,
                          pdMS_TO_TICKS(remaining))) return true;
    }
    result.delivered = result.delivered ||
                      sendState_.load(std::memory_order_acquire) == SendState::kSuccess;
    if (sendPending()) return false;

  }
  return false;
}

bool EspNowTransport::ensurePeer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) {
    return true;
  }
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  const esp_err_t result = esp_now_add_peer(&peer);
  return result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST;
}

void EspNowTransport::end() {
  esp_now_unregister_recv_cb();
  esp_now_unregister_send_cb();
  esp_now_deinit();
  instance_ = nullptr;
  // Callbacks are unregistered and ESP-NOW is stopped before queues are freed.
  const auto recordingResponses = recordingResponses_.load(std::memory_order_relaxed);
  recordingResponses_.store(nullptr, std::memory_order_release);
  if (recordingResponses) vQueueDelete(recordingResponses);
  const auto otaResponses = otaResponses_.load(std::memory_order_relaxed);
  otaResponses_.store(nullptr, std::memory_order_release);
  if (otaResponses) vQueueDelete(otaResponses);
  if (responses_ != nullptr) {
    vQueueDelete(responses_);
    responses_ = nullptr;
  }
  sendState_.store(SendState::kIdle, std::memory_order_release);
  if (WiFi.getMode() != WIFI_OFF) {
    WiFi.mode(WIFI_OFF);
  }
}

bool EspNowTransport::otaExchange(const uint8_t mac[6],
    const lil::ota::Packet& request, lil::ota::Packet& response) {
  const size_t requestLength = sizeof(lil::protocol::PacketHeader) + request.header.payloadSize;
  if (!lil::ota::requestSizeSupported(requestLength)) return false;
  // Discovery, battery-protection and failed telemetry wakes need no OTA queue.
  // Allocate before sending the first pull request so its reply cannot race it.
  auto responses = otaResponses_.load(std::memory_order_acquire);
  if (!responses) {
    responses = xQueueCreate(4, sizeof(OtaEvent));
    otaResponses_.store(responses, std::memory_order_release);
  }
  if (!responses || !ensurePeer(mac)) return false;
  for (uint8_t attempt = 0; attempt < 5; ++attempt) {
    const uint32_t waitStarted = millis();
    while (sendPending() && millis() - waitStarted < 500) delay(1);
    if (sendPending()) return false;
    if (!sendFrame(mac, &request, requestLength)) {
      delay(20); continue;
    }
    const TickType_t started = xTaskGetTickCount();
    OtaEvent event{};
    for (;;) {
      const TickType_t elapsed = xTaskGetTickCount() - started;
      if (elapsed >= kOtaResponseWait ||
          xQueueReceive(responses, &event, kOtaResponseWait - elapsed) != pdTRUE) break;
      if (!memcmp(mac, event.mac, 6) &&
          lil::protocol::validate(event.packet, sizeof(event.packet), lil::ota::kMessageType) &&
          event.packet.header.sequence == request.header.sequence &&
          event.packet.payload.nonce == request.payload.nonce) {
        response = event.packet; return true;
      }
    }
  }
  return false;
}

bool EspNowTransport::recordingExchange(const SensorRuntimeConfig& config,
    const void* request, size_t length, uint64_t session, uint32_t sampleMs,
    uint32_t crc, lil::recording::Ack& response) {
  if (!responses_ || !config.stationKnown || !config.provisioned ||
      !request || length < sizeof(lil::protocol::PacketHeader) ||
      length > lil::protocol::kMaxPacketSize) return false;
  // An application ACK can arrive before our MAC callback. Give that send
  // its remaining time before retuning or issuing the next archive request.
  while (sendPending()) {
    if (millis() - sendStartedMs_ >= kSendWaitMs) return false;
    delay(1);
  }
  lil::protocol::PacketHeader requestHeader{};
  memcpy(&requestHeader, request, sizeof(requestHeader));
  // Environmental wake cycles never allocate an archive-response queue.
  auto responses = recordingResponses_.load(std::memory_order_acquire);
  if (!responses) {
    responses = xQueueCreate(4, sizeof(RecordingEvent));
    recordingResponses_.store(responses, std::memory_order_release);
  }
  if (!responses ||
      esp_wifi_set_channel(config.wifiChannel, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
      !ensurePeer(config.stationMac)) return false;
  if (!sendFrame(config.stationMac, request, length)) return false;
  const TickType_t started = xTaskGetTickCount(); RecordingEvent event{};
  for (;;) {
    const TickType_t elapsed = xTaskGetTickCount() - started;
    if (elapsed >= kRecordingResponseWait ||
        xQueueReceive(responses, &event, kRecordingResponseWait - elapsed) != pdTRUE) break;
    const auto& ack = event.packet.payload;
    if (!memcmp(event.mac, config.stationMac, 6) &&
        event.packet.header.sequence == requestHeader.sequence && ack.session == session &&
        ack.sampleMs == sampleMs && ack.recordCrc == crc) { response = ack; return true; }
  }
  return false;
}

}  // namespace sensor

#include "espnow_transport.h"

#include <WiFi.h>
#include <esp_random.h>
#include <esp_wifi.h>

#include "sensor_log.h"

namespace sensor {

EspNowTransport* EspNowTransport::instance_ = nullptr;

namespace {
constexpr uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF,
                                      0xFF, 0xFF, 0xFF};
constexpr uint32_t kSendWaitMs = 180;
constexpr uint32_t kResponseWaitMs = 260;
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
  otaResponses_ = xQueueCreate(4, sizeof(OtaEvent));
  if (otaResponses_ == nullptr) { if (responses_) vQueueDelete(responses_); responses_ = nullptr; return false; }
  if (responses_ == nullptr) { vQueueDelete(otaResponses_); otaResponses_ = nullptr; return false; }
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
    instance_->sendState_.store(status == ESP_NOW_SEND_SUCCESS ? 2 : 3,
                                std::memory_order_release);
  }
}

void EspNowTransport::receiveCallback(const esp_now_recv_info_t* info,
                                      const uint8_t* data, int length) {
  if (instance_ && info && info->src_addr && data &&
      length == sizeof(lil::ota::Packet) && instance_->otaResponses_) {
    OtaEvent event{};
    memcpy(event.mac, info->src_addr, 6);
    memcpy(&event.packet, data, sizeof(event.packet));
    xQueueSend(instance_->otaResponses_, &event, 0);
    return;
  }
  if (instance_ == nullptr || info == nullptr || info->src_addr == nullptr || data == nullptr ||
      length != static_cast<int>(sizeof(lil::protocol::ConfigResponsePacket))) {
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

ExchangeResult EspNowTransport::exchange(
    const lil::protocol::TelemetryPacket& packet,
    const SensorRuntimeConfig& config, bool trialBoot) {
  ExchangeResult result{};
  expectedSequence_ = packet.header.sequence;
  const bool configured = config.provisioned && config.stationKnown;
  const uint8_t* destination =
      configured ? config.stationMac : kBroadcastMac;

  if (tryChannel(config.wifiChannel, destination, packet, result,
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
      if (sendState_.load(std::memory_order_acquire) == 1) break;
      if (receiveResponse(result, destination, pdMS_TO_TICKS(50))) return result;
      if (tryChannel(config.wifiChannel, destination, packet, result,
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
    if (tryChannel(channel, kBroadcastMac, packet, result,
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
  // Keep each radio window short. Normal reports get two attempts on the known
  // channel. Recovery scans use one maximum-power attempt per scheduled report
  // and therefore do not unnecessarily extend the active time.
  tryChannel(channel, config.stationMac, packet, result,
             config.txPowerQuarterDbm,
             recoveryAttempt ? kLpRecoveryAttempts : kLpNormalAttempts,
             recoveryAttempt);
  return result;
}

bool EspNowTransport::tryChannel(
    uint8_t channel, const uint8_t destination[6],
    const lil::protocol::TelemetryPacket& packet, ExchangeResult& result,
    int8_t baseTxPowerQuarterDbm, uint8_t attempts,
    bool forceMaximumPower) {
  // Never retune or enqueue another frame while its callback is outstanding.
  if (sendState_.load(std::memory_order_acquire) == 1) return false;
  if (receiveResponse(result, destination, 0)) return true;
  if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
      !ensurePeer(destination)) {
    return false;
  }
  // Environmental telemetry never needs the IMU suffix. Preserve the V5
  // wire size so an existing 4.0.0 station can acknowledge a trial boot.
  auto wirePacket = packet;
  const bool motion = packet.payload.sensorType == lil::protocol::EnvironmentalSensorType::kLsm6dsox ||
                      (packet.payload.capabilities & lil::protocol::kMotion) != 0;
  const size_t payloadSize = motion ? sizeof(packet.payload) : lil::protocol::kLegacyTelemetryPayloadSize;
  const size_t wireSize = sizeof(lil::protocol::PacketHeader) + payloadSize;
  lil::protocol::finalizePacket(&wirePacket, wireSize,
      lil::protocol::MessageType::kTelemetry, packet.header.sequence, payloadSize);
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
    sendState_.store(1, std::memory_order_release);
    if (esp_now_send(destination, reinterpret_cast<const uint8_t*>(&wirePacket),
                     wireSize) != ESP_OK) {
      sendState_.store(0, std::memory_order_release);
      continue;
    }

    const uint32_t sendStarted = millis();
    while (sendState_.load(std::memory_order_acquire) == 1 &&
           millis() - sendStarted < kSendWaitMs) {
      // Wait on the response queue instead of repeatedly polling volatile data.
      if (receiveResponse(result, destination, pdMS_TO_TICKS(10))) break;
    }
    const uint8_t state = sendState_.load(std::memory_order_acquire);
    result.delivered = result.delivered || state == 2;
    if (result.configReceived) return true;
    // A missing MAC ACK can accompany a valid application response. Retain a
    // short grace window after failure, and the full window after delivery.
    const uint32_t responseWaitMs = !broadcast && state == 3 ? 30 : kResponseWaitMs;
    const uint32_t responseStarted = millis();
    for (;;) {
      const uint32_t elapsed = millis() - responseStarted;
      if (elapsed >= responseWaitMs) break;
      const uint32_t remaining = responseWaitMs - elapsed;
      if (receiveResponse(result, destination,
                          pdMS_TO_TICKS(remaining))) return true;
    }
    result.delivered = result.delivered ||
                      sendState_.load(std::memory_order_acquire) == 2;
    if (sendState_.load(std::memory_order_acquire) == 1) return false;

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
  if (otaResponses_) { vQueueDelete(otaResponses_); otaResponses_ = nullptr; }
  if (responses_ != nullptr) {
    vQueueDelete(responses_);
    responses_ = nullptr;
  }
  sendState_.store(0, std::memory_order_release);
  if (WiFi.getMode() != WIFI_OFF) {
    WiFi.mode(WIFI_OFF);
  }
}

bool EspNowTransport::otaExchange(const uint8_t mac[6],
    const lil::ota::Packet& request, lil::ota::Packet& response) {
  if (!otaResponses_ || !ensurePeer(mac)) return false;
  for (uint8_t attempt = 0; attempt < 5; ++attempt) {
    const uint32_t waitStarted = millis();
    while (sendState_.load(std::memory_order_acquire) == 1 && millis() - waitStarted < 500) delay(1);
    if (sendState_.load(std::memory_order_acquire) == 1) return false;
    sendState_.store(1, std::memory_order_release);
    if (esp_now_send(mac, reinterpret_cast<const uint8_t*>(&request), sizeof(request)) != ESP_OK) {
      sendState_.store(0, std::memory_order_release); delay(20); continue;
    }
    const uint32_t started = millis();
    OtaEvent event{};
    while (millis() - started < 700) {
      if (xQueueReceive(otaResponses_, &event, pdMS_TO_TICKS(20)) != pdTRUE) continue;
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

}  // namespace sensor

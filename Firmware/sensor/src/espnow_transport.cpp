#include "espnow_transport.h"

#include <WiFi.h>
#include <esp_random.h>
#include <esp_wifi.h>

namespace sensor {

EspNowTransport* EspNowTransport::instance_ = nullptr;

namespace {
constexpr uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF,
                                      0xFF, 0xFF, 0xFF};
constexpr uint32_t kSendWaitMs = 180;
constexpr uint32_t kResponseWaitMs = 260;
constexpr uint8_t kChannelsPerDiscoveryWake = 4;
constexpr uint8_t kUnicastAttempts = 4;
constexpr uint8_t kBroadcastAttempts = 1;
constexpr uint8_t kLpNormalAttempts = 2;
constexpr uint8_t kLpRecoveryAttempts = 1;
constexpr int8_t kTxPowerStepQuarterDbm = 12;  // 3 dBm per retry.
constexpr int8_t kMaximumTxPowerQuarterDbm = 84;  // 21 dBm.

int8_t txPowerForAttempt(int8_t baseQuarterDbm, uint8_t attempt,
                         bool broadcast) {
  const int8_t base = constrain(baseQuarterDbm, 8, kMaximumTxPowerQuarterDbm);
  if (broadcast || attempt == 0) {
    return base;
  }
  if (attempt >= kUnicastAttempts - 1) {
    return kMaximumTxPowerQuarterDbm;
  }
  return static_cast<int8_t>(
      min(static_cast<int>(kMaximumTxPowerQuarterDbm),
          static_cast<int>(base) +
              static_cast<int>(attempt) * kTxPowerStepQuarterDbm));
}
}

bool EspNowTransport::begin() {
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
  if (instance_ != nullptr) {
    instance_->sendSucceeded_ = status == ESP_NOW_SEND_SUCCESS;
    instance_->sendFinished_ = true;
  }
}

void EspNowTransport::receiveCallback(const esp_now_recv_info_t* info,
                                      const uint8_t* data, int length) {
  if (instance_ == nullptr || info == nullptr || data == nullptr ||
      length != static_cast<int>(sizeof(lil::protocol::ConfigResponsePacket))) {
    return;
  }

  lil::protocol::ConfigResponsePacket packet{};
  memcpy(&packet, data, sizeof(packet));
  if (!lil::protocol::validate(packet, length,
                               lil::protocol::MessageType::kConfigResponse) ||
      packet.payload.requestSequence != instance_->expectedSequence_) {
    return;
  }
  instance_->response_ = packet.payload;
  instance_->responseRssi_ =
      info->rx_ctrl != nullptr ? info->rx_ctrl->rssi : 0;
  instance_->responseReady_ = true;
}

ExchangeResult EspNowTransport::exchange(
    const lil::protocol::TelemetryPacket& packet,
    const SensorRuntimeConfig& config, bool fastDiscovery) {
  ExchangeResult result{};
  expectedSequence_ = packet.header.sequence;
  const bool configured = config.provisioned && config.stationKnown;
  const uint8_t* destination =
      configured ? config.stationMac : kBroadcastMac;

  // Sensors with identical intervals otherwise wake within the same few
  // milliseconds. A short random backoff prevents repeatable RF collisions.
  // Also stagger brand-new sensors. Several units are commonly powered at
  // once during setup and would otherwise broadcast on every channel in lock
  // step, repeatedly colliding before the station can answer.
  delay(esp_random() % 121U);

  if (tryChannel(config.wifiChannel, destination, packet, result,
                 config.txPowerQuarterDbm,
                 configured ? kUnicastAttempts : kBroadcastAttempts)) {
    return result;
  }

  // A configured sensor first uses the last channel confirmed by the station.
  // If that channel fails, search the remaining channels by unicast. This is
  // essential when setup finishes: the station AP can use the fallback
  // channel, while the home WLAN moves the station radio to another channel.
  // The scan only costs energy while the saved channel is actually stale.
  if (configured) {
    for (uint8_t channel = 1; channel <= 13; ++channel) {
      if (channel == config.wifiChannel) {
        continue;
      }
    if (tryChannel(channel, destination, packet, result,
                      config.txPowerQuarterDbm, kUnicastAttempts)) {
        return result;
      }
    }
    return result;
  }

  const uint8_t scanStart =
      static_cast<uint8_t>((packet.header.sequence *
                            kChannelsPerDiscoveryWake) % 13U) +
      1U;
  const uint8_t scanLimit = fastDiscovery ? 13 : kChannelsPerDiscoveryWake;
  uint8_t scanned = 0;
  for (uint8_t offset = 0; offset < 13 && scanned < scanLimit;
       ++offset) {
    const uint8_t channel =
        static_cast<uint8_t>((scanStart - 1U + offset) % 13U) + 1U;
    if (channel == config.wifiChannel) {
      continue;
    }
    ++scanned;
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
  // channel. Recovery scans use one maximum-power attempt per BSEC cycle and
  // therefore do not unnecessarily extend the active time.
  delay(esp_random() % 31U);
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
  if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
      !ensurePeer(destination)) {
    return false;
  }
  const bool broadcast = memcmp(destination, kBroadcastMac, 6) == 0;
  for (uint8_t attempt = 0; attempt < attempts; ++attempt) {
    const int8_t txPower = forceMaximumPower
                               ? kMaximumTxPowerQuarterDbm
                               : txPowerForAttempt(baseTxPowerQuarterDbm,
                                                   attempt, broadcast);
    if (esp_wifi_set_max_tx_power(txPower) != ESP_OK) {
      Serial.printf("[ESP-NOW] Sendeleistung %d.%02d dBm konnte nicht gesetzt werden\n",
                    txPower / 4, (abs(txPower) % 4) * 25);
    }
    sendFinished_ = false;
    sendSucceeded_ = false;
    responseReady_ = false;
    if (attempt > 0) {
      delay(8U + (esp_random() % 35U));
    }
    if (esp_now_send(destination, reinterpret_cast<const uint8_t*>(&packet),
                     sizeof(packet)) != ESP_OK) {
      continue;
    }

    const uint32_t sendStarted = millis();
    while (!sendFinished_ && millis() - sendStarted < kSendWaitMs) {
      delay(1);
    }
    result.delivered = result.delivered || sendSucceeded_;

    const uint32_t responseStarted = millis();
    while (!responseReady_ && millis() - responseStarted < kResponseWaitMs) {
      delay(1);
    }
    if (responseReady_) {
      result.configReceived = true;
      result.config = response_;
      result.stationRssi = responseRssi_;
      return true;
    }
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
  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
}

}  // namespace sensor

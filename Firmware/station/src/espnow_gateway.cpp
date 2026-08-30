#include "espnow_gateway.h"

#include <esp_wifi.h>

namespace station {

EspNowGateway* EspNowGateway::instance_ = nullptr;

namespace {
// Keep enough headroom while HTTPS or the web UI is busy. The receive callback
// stays non-blocking, so Wi-Fi/ESP-NOW system work is never held up by storage.
constexpr size_t kReceiveQueueSize = 64;
constexpr int8_t kDefaultSensorTxPowerQuarterDbm = 52;  // 13 dBm
}

bool EspNowGateway::begin(SensorRegistry& registry,
                          ThingSpeakService& thingSpeak, WifiService& wifi) {
  registry_ = &registry;
  thingSpeak_ = &thingSpeak;
  wifi_ = &wifi;
  receiveQueue_ = xQueueCreate(kReceiveQueueSize, sizeof(EspNowRxEvent));
  if (receiveQueue_ == nullptr) {
    return false;
  }

  instance_ = this;
  if (esp_now_init() != ESP_OK ||
      esp_now_register_recv_cb(receiveCallback) != ESP_OK) {
    return false;
  }
  return xTaskCreate(taskEntry, "espnow-gateway", 6144, this, 3, &task_) ==
         pdPASS;
}

void EspNowGateway::receiveCallback(const esp_now_recv_info_t* info,
                                    const uint8_t* data, int length) {
  if (instance_ == nullptr || info == nullptr || info->src_addr == nullptr ||
      data == nullptr || length <= 0 ||
      length > static_cast<int>(lil::protocol::kMaxPacketSize)) {
    if (instance_ != nullptr) {
      instance_->invalidPackets_.fetch_add(1, std::memory_order_relaxed);
    }
    return;
  }

  EspNowRxEvent event{};
  memcpy(event.sourceMac, info->src_addr, sizeof(event.sourceMac));
  event.rssi = info->rx_ctrl != nullptr ? info->rx_ctrl->rssi : 0;
  event.length = static_cast<uint16_t>(length);
  memcpy(event.data, data, event.length);
  if (xQueueSend(instance_->receiveQueue_, &event, 0) != pdTRUE) {
    instance_->droppedPackets_.fetch_add(1, std::memory_order_relaxed);
  }
}

void EspNowGateway::taskEntry(void* context) {
  static_cast<EspNowGateway*>(context)->taskLoop();
}

void EspNowGateway::taskLoop() {
  EspNowRxEvent event{};
  for (;;) {
    if (xQueueReceive(receiveQueue_, &event, portMAX_DELAY) == pdTRUE) {
      handle(event);
    }
  }
}

void EspNowGateway::handle(const EspNowRxEvent& event) {
  if (event.length != sizeof(lil::protocol::TelemetryPacket)) {
    invalidPackets_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  lil::protocol::TelemetryPacket packet{};
  memcpy(&packet, event.data, sizeof(packet));
  if (!lil::protocol::validate(packet, event.length,
                               lil::protocol::MessageType::kTelemetry)) {
    invalidPackets_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  SensorConfig responseConfig{};
  bool duplicate = false;
  if (!registry_->registerTelemetry(event.sourceMac, packet.header.sequence,
                                    packet.payload, event.rssi, responseConfig,
                                    duplicate)) {
    droppedPackets_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  receivedPackets_.fetch_add(1, std::memory_order_relaxed);

  if (!duplicate &&
      (packet.payload.flags &
       (lil::protocol::kDiscoveryBeacon |
        lil::protocol::kBme680Commissioning)) == 0) {
    thingSpeak_->queue(responseConfig, packet.payload, event.rssi,
                       packet.header.sequence);
  }

  if (!ensurePeer(event.sourceMac)) {
    return;
  }

  lil::protocol::ConfigResponsePacket response{};
  response.payload.revision = responseConfig.revision;
  response.payload.sleepIntervalSeconds = responseConfig.sleepSeconds;
  response.payload.requestSequence = packet.header.sequence;
  // pendingFlags also contains station-internal handshake state. Only actual
  // protocol commands may cross the radio boundary.
  response.payload.flags = responseConfig.pendingFlags & kSensorCommandFlags;
  response.payload.wifiChannel = wifi_->channel();
  response.payload.txPowerQuarterDbm = kDefaultSensorTxPowerQuarterDbm;
  response.payload.sensorType = responseConfig.environmentalSensorType;
  response.payload.temperatureOffsetC = responseConfig.temperatureOffsetC;
  response.payload.batteryCalibrationFactor =
      responseConfig.batteryCalibrationFactor;
  response.payload.provisioned = responseConfig.provisioned ? 1U : 0U;
  memcpy(response.payload.stationMac, wifi_->stationMac(),
         sizeof(response.payload.stationMac));
  lil::protocol::finalize(response,
                          lil::protocol::MessageType::kConfigResponse,
                          packet.header.sequence);

  esp_now_send(event.sourceMac, reinterpret_cast<uint8_t*>(&response),
               sizeof(response));
}

bool EspNowGateway::ensurePeer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) {
    return true;
  }

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;  // Use the AP/STA radio's current channel.
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  const esp_err_t result = esp_now_add_peer(&peer);
  return result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST;
}

uint32_t EspNowGateway::receivedPackets() const {
  return receivedPackets_.load(std::memory_order_relaxed);
}

uint32_t EspNowGateway::invalidPackets() const {
  return invalidPackets_.load(std::memory_order_relaxed);
}

uint32_t EspNowGateway::droppedPackets() const {
  return droppedPackets_.load(std::memory_order_relaxed);
}

}  // namespace station

#include "espnow_gateway.h"
#include "ota_service.h"

#include <esp_wifi.h>
#include <time.h>

namespace station {

EspNowGateway* EspNowGateway::instance_ = nullptr;

namespace {
// Keep enough headroom while HTTPS or the web UI is busy. The receive callback
// stays non-blocking, so Wi-Fi/ESP-NOW system work is never held up by storage.
constexpr size_t kReceiveQueueSize = 64;
constexpr size_t kPersistenceQueueSize = 64;
constexpr TickType_t kResponseCompletionWait = pdMS_TO_TICKS(80);
}

bool EspNowGateway::begin(SensorRegistry& registry,
                          ThingSpeakService& thingSpeak, WifiService& wifi) {
  registry_ = &registry;
  thingSpeak_ = &thingSpeak;
  wifi_ = &wifi;
  receiveQueue_ = xQueueCreate(kReceiveQueueSize, sizeof(EspNowRxEvent));
  persistenceQueue_ =
      xQueueCreate(kPersistenceQueueSize, sizeof(EspNowPersistenceEvent));
  if (receiveQueue_ == nullptr || persistenceQueue_ == nullptr) {
    return false;
  }

  instance_ = this;
  if (esp_now_init() != ESP_OK ||
      esp_now_register_recv_cb(receiveCallback) != ESP_OK ||
      esp_now_register_send_cb(sendCallback) != ESP_OK) {
    return false;
  }
  if (xTaskCreate(persistenceTaskEntry, "telemetry-store", 6144, this, 1,
                  &persistenceTask_) != pdPASS) {
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
  const time_t now = time(nullptr);
  event.receivedAt = now >= 1577836800 ? static_cast<uint32_t>(now) : 0;
  memcpy(event.sourceMac, info->src_addr, sizeof(event.sourceMac));
  event.rssi = info->rx_ctrl != nullptr ? info->rx_ctrl->rssi : 0;
  event.length = static_cast<uint16_t>(length);
  memcpy(event.data, data, event.length);
  if (xQueueSend(instance_->receiveQueue_, &event, 0) != pdTRUE) {
    instance_->droppedPackets_.fetch_add(1, std::memory_order_relaxed);
  }
}

void EspNowGateway::sendCallback(const wifi_tx_info_t* info,
                                 esp_now_send_status_t status) {
  (void)info;
  (void)status;
  if (instance_ != nullptr && instance_->task_ != nullptr) {
    instance_->sendPending_.store(false, std::memory_order_release);
    xTaskNotifyGive(instance_->task_);
  }
}

void EspNowGateway::taskEntry(void* context) {
  static_cast<EspNowGateway*>(context)->taskLoop();
}

void EspNowGateway::persistenceTaskEntry(void* context) {
  static_cast<EspNowGateway*>(context)->persistenceTaskLoop();
}

void EspNowGateway::taskLoop() {
  EspNowRxEvent event{};
  for (;;) {
    if (xQueueReceive(receiveQueue_, &event, portMAX_DELAY) == pdTRUE) {
      handle(event);
    }
  }
}

void EspNowGateway::persistenceTaskLoop() {
  EspNowPersistenceEvent event{};
  for (;;) {
    if (xQueueReceive(persistenceQueue_, &event, portMAX_DELAY) == pdTRUE) {
      persist(event);
    }
  }
}

void EspNowGateway::persist(const EspNowPersistenceEvent& event) {
  if (!registry_->persistTelemetry(event.sourceMac, event.sequence,
                                   event.telemetry, event.rssi, event.generation,
                                   event.receivedAt)) {
    droppedPackets_.fetch_add(1, std::memory_order_relaxed);
  }
  if (event.queueCloudUpload &&
      registry_->generationMatches(event.sourceMac, event.generation)) {
    thingSpeak_->queue(event.responseConfig, event.telemetry, event.rssi,
                       event.sequence, event.receivedAt,
                       registry_->channelShared(event.responseConfig.thingSpeakChannelId));
  }
}

void EspNowGateway::handle(const EspNowRxEvent& event) {
  if (event.length == sizeof(lil::ota::Packet)) {
    lil::ota::Packet request{}, response{};
    memcpy(&request, event.data, sizeof(request));
    SensorConfig config{};
    if (!lil::protocol::validate(request, event.length, lil::ota::kMessageType) ||
        !registry_->findConfig(event.sourceMac, config) || !config.provisioned) return;
    if (sendPending_.load(std::memory_order_acquire)) {
      ulTaskNotifyTake(pdTRUE, kResponseCompletionWait);
      if (sendPending_.load(std::memory_order_acquire)) return;
    }
    if (otaService.respond(event.sourceMac, request, response) && ensurePeer(event.sourceMac)) {
      ulTaskNotifyTake(pdTRUE, 0);
      sendPending_.store(true, std::memory_order_release);
      if (esp_now_send(event.sourceMac, reinterpret_cast<const uint8_t*>(&response), sizeof(response)) == ESP_OK)
        ulTaskNotifyTake(pdTRUE, kResponseCompletionWait);
      else sendPending_.store(false, std::memory_order_release);
    }
    return;
  }
  const bool legacy = event.length == sizeof(lil::protocol::PacketHeader) +
      lil::protocol::kLegacyTelemetryPayloadSize;
  if (!legacy && event.length != sizeof(lil::protocol::TelemetryPacket)) {
    invalidPackets_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  lil::protocol::TelemetryPacket packet{};
  memcpy(&packet, event.data, event.length);
  if (!lil::protocol::validatePacket(event.data, event.length,
          lil::protocol::MessageType::kTelemetry,
          legacy ? lil::protocol::kLegacyTelemetryPayloadSize : sizeof(packet.payload))) {
    invalidPackets_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  if (sendPending_.load(std::memory_order_acquire)) {
    ulTaskNotifyTake(pdTRUE, kResponseCompletionWait);
    if (sendPending_.load(std::memory_order_acquire)) {
      droppedPackets_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
  }
  uint32_t generation = 0;
  SensorConfig responseConfig{};
  int8_t txPowerQuarterDbm = 52;
  bool duplicate = false;
  if (!registry_->registerTelemetry(event.sourceMac, packet.header.sequence,
                                    packet.payload, event.rssi, responseConfig,
                                    txPowerQuarterDbm, duplicate, generation)) {
    droppedPackets_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  receivedPackets_.fetch_add(1, std::memory_order_relaxed);

  lil::protocol::ConfigResponsePacket response{};
  response.payload.revision = responseConfig.revision;
  response.payload.sleepIntervalSeconds = responseConfig.sleepSeconds;
  response.payload.requestSequence = packet.header.sequence;
  // pendingFlags also contains station-internal handshake state. Only actual
  // protocol commands may cross the radio boundary.
  response.payload.flags = responseConfig.pendingFlags & kSensorCommandFlags;
  if (wifi_->stationConnected()) {
    response.payload.flags |= lil::protocol::kStationChannelStable;
  }
  response.payload.wifiChannel = wifi_->channel();
  response.payload.txPowerQuarterDbm = txPowerQuarterDbm;
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

  if (ensurePeer(event.sourceMac)) {
    // Wait only on the mains-powered station. Flash persistence starts after
    // the Wi-Fi task has completed the response, so filesystem latency can no
    // longer consume the sensor's application-ack window.
    ulTaskNotifyTake(pdTRUE, 0);
    sendPending_.store(true, std::memory_order_release);
    if (esp_now_send(event.sourceMac, reinterpret_cast<uint8_t*>(&response),
                     sizeof(response)) == ESP_OK) {
      ulTaskNotifyTake(pdTRUE, kResponseCompletionWait);
    } else {
      sendPending_.store(false, std::memory_order_release);
      peerFailures_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  if (!duplicate) {
    EspNowPersistenceEvent persistence{};
    persistence.receivedAt = event.receivedAt;
    persistence.generation = generation;
    memcpy(persistence.sourceMac, event.sourceMac,
           sizeof(persistence.sourceMac));
    persistence.sequence = packet.header.sequence;
    persistence.rssi = event.rssi;
    persistence.telemetry = packet.payload;
    persistence.responseConfig = responseConfig;
    persistence.queueCloudUpload =
        (packet.payload.flags &
         (lil::protocol::kDiscoveryBeacon |
          lil::protocol::kBme680Commissioning)) == 0;
    if (xQueueSend(persistenceQueue_, &persistence, 0) != pdTRUE) {
      // Explicit bounded overload policy: retain recent data without blocking
      // the next sensor response on flash I/O. Expose any loss to diagnostics.
      EspNowPersistenceEvent discarded{};
      if (xQueueReceive(persistenceQueue_, &discarded, 0) == pdTRUE) {
        persistenceDrops_.fetch_add(1, std::memory_order_relaxed);
      }
      if (xQueueSend(persistenceQueue_, &persistence, 0) != pdTRUE) {
        persistenceDrops_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }
}

bool EspNowGateway::ensurePeer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) {
    return true;
  }

  // Prune deleted registry entries only after the outstanding send completed.
  uint8_t obsolete[ESP_NOW_MAX_TOTAL_PEER_NUM][6]{};
  size_t count = 0;
  esp_now_peer_info_t existing{};
  bool first = true;
  while (esp_now_fetch_peer(first, &existing) == ESP_OK) {
    first = false;
    SensorConfig config{};
    if (!registry_->findConfig(existing.peer_addr, config) &&
        count < ESP_NOW_MAX_TOTAL_PEER_NUM) {
      memcpy(obsolete[count++], existing.peer_addr, 6);
    }
  }
  for (size_t i = 0; i < count; ++i) esp_now_del_peer(obsolete[i]);
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;  // Use the AP/STA radio's current channel.
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  const esp_err_t result = esp_now_add_peer(&peer);
  const bool success = result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST;
  if (!success) peerFailures_.fetch_add(1, std::memory_order_relaxed);
  return success;
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

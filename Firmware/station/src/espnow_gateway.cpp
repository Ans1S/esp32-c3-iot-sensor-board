#include "espnow_gateway.h"
#include "ota_service.h"
#include "recording_archive.h"
#include "telemetry_validation.h"
#include <sys/time.h>

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
  if (!recordingArchive.begin()) return false;
  receiveQueue_ = xQueueCreate(kReceiveQueueSize, sizeof(EspNowRxEvent));
  persistenceQueue_ =
      xQueueCreate(kPersistenceQueueSize, sizeof(EspNowPersistenceEvent));
  archiveQueue_ = xQueueCreate(8, sizeof(EspNowArchiveEvent));
  if (receiveQueue_ == nullptr || persistenceQueue_ == nullptr || archiveQueue_ == nullptr) {
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
  if (xTaskCreate(archiveTaskEntry, "recording-store", 6144, this, 1, &archiveTask_) != pdPASS) return false;
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
void EspNowGateway::archiveTaskEntry(void* context) {
  auto* gateway = static_cast<EspNowGateway*>(context);
  EspNowArchiveEvent event{};
  for (;;) {
    if (xQueueReceive(gateway->archiveQueue_, &event, portMAX_DELAY) == pdTRUE) gateway->persistArchive(event);
    // Archive replay gets a separate bounded queue and yields between writes;
    // it cannot evict BME telemetry from the normal persistence queue.
    vTaskDelay(pdMS_TO_TICKS(1));
  }
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

void EspNowGateway::persistArchive(const EspNowArchiveEvent& event) {
    SensorConfig config{};
    uint32_t generation = 0;
    if (!registry_->findConfig(event.sourceMac, config, &generation) ||
        !config.provisioned || generation != event.generation) return;
    lil::recording::AckPacket ack{};
    const auto& record = event.recording.record;
    ack.payload.session = record.session; ack.payload.sampleMs = record.sampleMs;
    ack.payload.recordCrc = lil::recording::checksum(record);
    ack.payload.stored = recordingArchive.append(event.sourceMac, event.recording);
    lil::protocol::finalize(ack, lil::recording::kAckMessage, event.sequence);
    EspNowRxEvent reply{}; reply.archiveReply = true;
    reply.generation = event.generation;
    memcpy(reply.sourceMac, event.sourceMac, 6); memcpy(reply.data, &ack, sizeof(ack));
    reply.length = sizeof(ack);
    // A dropped ACK is retried by the sensor; the archive deduplicates it.
    xQueueSend(receiveQueue_, &reply, 0);
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
                       registry_->channelShared(event.responseConfig.thingSpeakChannelId),
                       event.generation);
  }
}

void EspNowGateway::handle(const EspNowRxEvent& event) {
  if (event.archiveReply || event.length == sizeof(lil::recording::StatusPacket) ||
      event.length == sizeof(lil::recording::UploadPacket)) {
    SensorConfig config{};
    uint32_t generation = 0;
    if (!registry_->findConfig(event.sourceMac, config, &generation) || !config.provisioned ||
        (event.archiveReply && event.generation != generation)) return;
    lil::recording::AckPacket ack{};
    if (event.archiveReply) memcpy(&ack, event.data, sizeof(ack));
    else if (event.length == sizeof(lil::recording::UploadPacket)) {
      lil::recording::UploadPacket request{}; memcpy(&request, event.data, sizeof(request));
      if (!lil::protocol::validate(request, event.length, lil::recording::kRecordMessage) ||
          !lil::recording::valid(request.payload.record)) return;
      EspNowArchiveEvent persistence{};
      persistence.generation = generation;
      persistence.recording = request.payload;
      persistence.sequence = request.header.sequence; memcpy(persistence.sourceMac, event.sourceMac, 6);
      // Do not evict environmental telemetry or another archive entry.
      if (xQueueSend(archiveQueue_, &persistence, 0) != pdTRUE) persistenceDrops_.fetch_add(1);
      return;
    } else {
      lil::recording::StatusPacket request{}; memcpy(&request, event.data, sizeof(request));
      if (!lil::protocol::validate(request, event.length, lil::recording::kStatusMessage) ||
          !lil::protocol::isLiveSensor(request.payload.type) || static_cast<uint8_t>(request.payload.state) > 7) return;
      registry_->updateRecording(event.sourceMac, request.payload);
      ack.payload.session = request.payload.session; ack.payload.sampleMs = request.payload.elapsedMs;
      timeval now{}; gettimeofday(&now, nullptr);
      if (now.tv_sec >= 1577836800) ack.payload.stationEpochMs = uint64_t(now.tv_sec)*1000 + now.tv_usec/1000;
      lil::protocol::finalize(ack, lil::recording::kAckMessage, request.header.sequence);
    }
    if (sendPending_.load(std::memory_order_acquire)) ulTaskNotifyTake(pdTRUE, kResponseCompletionWait);
    if (sendPending_.load(std::memory_order_acquire) || !ensurePeer(event.sourceMac)) return;
    ulTaskNotifyTake(pdTRUE, 0); sendPending_.store(true, std::memory_order_release);
    if (esp_now_send(event.sourceMac, reinterpret_cast<const uint8_t*>(&ack), sizeof(ack)) == ESP_OK)
      ulTaskNotifyTake(pdTRUE, kResponseCompletionWait);
    else sendPending_.store(false, std::memory_order_release);
    return;
  }
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
  const bool motionLegacy = event.length == sizeof(lil::protocol::PacketHeader) + lil::protocol::kMotionTelemetryPayloadSize;
  const bool precisionLegacy = event.length == sizeof(lil::protocol::PacketHeader) + lil::protocol::kPrecisionTelemetryPayloadSize;
  const bool timedLegacy = event.length == sizeof(lil::protocol::PacketHeader) + lil::protocol::kTimedTelemetryPayloadSize;
  if (!legacy && !motionLegacy && !precisionLegacy && !timedLegacy && event.length != sizeof(lil::protocol::TelemetryPacket)) {
    invalidPackets_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  lil::protocol::TelemetryPacket packet{};
  memcpy(&packet, event.data, event.length);
  if (!lil::protocol::validatePacket(event.data, event.length,
          lil::protocol::MessageType::kTelemetry,
          legacy ? lil::protocol::kLegacyTelemetryPayloadSize : motionLegacy ? lil::protocol::kMotionTelemetryPayloadSize : precisionLegacy ? lil::protocol::kPrecisionTelemetryPayloadSize : timedLegacy ? lil::protocol::kTimedTelemetryPayloadSize : sizeof(packet.payload))) {
    invalidPackets_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (!lil::protocol::validTelemetryValues(packet.payload)) {
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

  const bool liveTelemetry = lil::protocol::isLiveSensor(packet.payload.sensorType) &&
      !(packet.payload.flags & lil::protocol::kBatteryProtectionActive);
  if (!duplicate && (!liveTelemetry || registry_->needsPersistence(event.sourceMac))) {
    EspNowPersistenceEvent persistence{};
    persistence.receivedAt = event.receivedAt;
    persistence.generation = generation;
    memcpy(persistence.sourceMac, event.sourceMac,
           sizeof(persistence.sourceMac));
    persistence.sequence = packet.header.sequence;
    persistence.rssi = event.rssi;
    persistence.telemetry = packet.payload;
    persistence.responseConfig = responseConfig;
    persistence.queueCloudUpload = !liveTelemetry &&
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

#include "espnow_gateway.h"
#include "filesystem_guard.h"
#include "ota_service.h"
#include "recording_archive.h"
#include "telemetry_validation.h"
#include "normal_history_policy.h"
#include <sys/time.h>

#include <esp_wifi.h>
#ifdef ESP_PLATFORM
#include <esp_task_wdt.h>
#endif
#include <time.h>

namespace station {

EspNowGateway* EspNowGateway::instance_ = nullptr;

namespace {
// Keep enough headroom while HTTPS or the web UI is busy. The receive callback
// stays non-blocking, so Wi-Fi/ESP-NOW system work is never held up by storage.
constexpr size_t kReceiveQueueSize = 64;
constexpr size_t kPersistenceQueueSize = 64;
constexpr TickType_t kResponseCompletionWait = pdMS_TO_TICKS(80);
// 1052-byte ESP-NOW v2.0 OTA frames at 1 Mbps take ~9 ms per air attempt;
// allow additional MAC-retry headroom under Wi-Fi AP/STA contention.
constexpr TickType_t kLargeResponseCompletionWait = pdMS_TO_TICKS(180);
#ifdef ESP_PLATFORM
constexpr TickType_t kOtaBusyWait = pdMS_TO_TICKS(25);
#endif
// A permanently missing driver callback otherwise blocks every later reply.
// Allow substantial flash/Wi-Fi latency before recovering the whole radio.
constexpr uint32_t kSendStallRestartMs = 10000;

bool supportedRequest(const uint8_t* data, size_t length) {
  using namespace lil::protocol;
  MessageType type;
  if (length > kStationReceiveCapacity || !readPacketType(data, length, type)) return false;
  if (type == MessageType::kTelemetry) {
    const size_t payloadSize = length - sizeof(PacketHeader);
    return payloadSize == sizeof(TelemetryPayload) || payloadSize == kLegacyTelemetryPayloadSize ||
        payloadSize == kMotionTelemetryPayloadSize || payloadSize == kPrecisionTelemetryPayloadSize ||
        payloadSize == kTimedTelemetryPayloadSize ||
        payloadSize == kMotionFeedbackTelemetryPayloadSize;
  }
  return (type == lil::recording::kStatusMessage && length == sizeof(lil::recording::StatusPacket)) ||
      (type == lil::recording::kRecordMessage && length == sizeof(lil::recording::UploadPacket)) ||
      (type == lil::ota::kMessageType && lil::ota::requestSizeSupported(length));
}
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
  pausedForReset_.store(false, std::memory_order_release);
  if (esp_now_init() != ESP_OK ||
      esp_now_register_recv_cb(receiveCallback) != ESP_OK ||
      esp_now_register_send_cb(sendCallback) != ESP_OK) {
    return false;
  }
#ifdef ESP_PLATFORM
  // Pin flash persistence and archive workers to Core 1 while keeping the
  // latency-sensitive ESP-NOW gateway on Core 0 alongside the Wi-Fi driver.
  if (xTaskCreatePinnedToCore(persistenceTaskEntry, "telemetry-store", 6144, this, 1,
                              &persistenceTask_, 1) != pdPASS) {
    return false;
  }
  if (xTaskCreatePinnedToCore(archiveTaskEntry, "recording-store", 6144, this, 1,
                              &archiveTask_, 1) != pdPASS) {
    return false;
  }
  return xTaskCreatePinnedToCore(taskEntry, "espnow-gateway", 6144, this, 3,
                                 &task_, 0) == pdPASS;
#else
  if (xTaskCreate(persistenceTaskEntry, "telemetry-store", 6144, this, 1,
                  &persistenceTask_) != pdPASS) {
    return false;
  }
  if (xTaskCreate(archiveTaskEntry, "recording-store", 6144, this, 1, &archiveTask_) != pdPASS) return false;
  return xTaskCreate(taskEntry, "espnow-gateway", 6144, this, 3, &task_) ==
         pdPASS;
#endif
}

void EspNowGateway::pauseForReset() {
  pausedForReset_.store(true, std::memory_order_release);
  esp_now_unregister_recv_cb();
  esp_now_unregister_send_cb();
}

void EspNowGateway::receiveCallback(const esp_now_recv_info_t* info,
                                    const uint8_t* data, int length) {
  if (instance_ == nullptr || instance_->pausedForReset_.load(std::memory_order_acquire) ||
      info == nullptr || info->src_addr == nullptr ||
      !supportedRequest(data, length)) {
    if (instance_ != nullptr && !instance_->pausedForReset_.load(std::memory_order_acquire)) {
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
  if (instance_ != nullptr && instance_->task_ != nullptr) {
    if (status != ESP_NOW_SEND_SUCCESS) {
      instance_->replyMacFailures_.fetch_add(1, std::memory_order_relaxed);
    }
    instance_->sendGuard_.complete();
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
#ifdef ESP_PLATFORM
  (void)esp_task_wdt_add(nullptr);
#endif
  EspNowArchiveEvent event{};
  for (;;) {
#ifdef ESP_PLATFORM
    (void)esp_task_wdt_reset();
    const TickType_t wait = pdMS_TO_TICKS(5000);
#else
    const TickType_t wait = portMAX_DELAY;
#endif
    if (xQueueReceive(gateway->archiveQueue_, &event, wait) == pdTRUE) {
      if (!gateway->pausedForReset_.load(std::memory_order_acquire)) {
        gateway->persistArchive(event);
      }
    }
    // Archive replay gets a separate bounded queue and yields between writes;
    // it cannot evict BME telemetry from the normal persistence queue.
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void EspNowGateway::taskLoop() {
#ifdef ESP_PLATFORM
  (void)esp_task_wdt_add(nullptr);
#endif
  EspNowRxEvent event{};
  for (;;) {
#ifdef ESP_PLATFORM
    (void)esp_task_wdt_reset();
    const TickType_t idleWait = pdMS_TO_TICKS(5000);
#else
    const TickType_t idleWait = portMAX_DELAY;
#endif
    const TickType_t wait =
        sendGuard_.pending() ? pdMS_TO_TICKS(250) : idleWait;
    if (xQueueReceive(receiveQueue_, &event, wait) == pdTRUE) {
      if (!pausedForReset_.load(std::memory_order_acquire)) {
        handle(event);
      }
    }
    // There is no safe way to correlate a late callback with a newer frame.
    // Flush buffered history and serialize against flash writes before restart.
    if (sendGuard_.stalled(millis(), kSendStallRestartMs)) {
      Serial.println("[ESP-NOW] Send callback stalled; restarting station");
      registry_->flushAllHistory();
      FilesystemGuard fsGuard;
      ESP.restart();
    }
  }
}

void EspNowGateway::persistenceTaskLoop() {
#ifdef ESP_PLATFORM
  (void)esp_task_wdt_add(nullptr);
#endif
  EspNowPersistenceEvent event{};
  for (;;) {
#ifdef ESP_PLATFORM
    (void)esp_task_wdt_reset();
#endif
    if (xQueueReceive(persistenceQueue_, &event, pdMS_TO_TICKS(250)) == pdTRUE) {
      if (!pausedForReset_.load(std::memory_order_acquire)) {
        persist(event);
      }
    }
    if (!pausedForReset_.load(std::memory_order_acquire)) {
      registry_->flushHistory();
    }
  }
}

void EspNowGateway::persistArchive(const EspNowArchiveEvent& event) {
    if (pausedForReset_.load(std::memory_order_acquire)) return;
    SensorConfig config{};
    uint32_t generation = 0;
    if (!registry_->findConfig(event.sourceMac, config, &generation) ||
        !config.provisioned || generation != event.generation) return;
    if (pausedForReset_.load(std::memory_order_acquire)) return;
    lil::recording::AckPacket ack{};
    const auto& record = event.recording.record;
    ack.payload.session = record.session; ack.payload.sampleMs = record.sampleMs;
    ack.payload.recordCrc = lil::recording::checksum(record);
    ack.payload.stored = recordingArchive.append(event.sourceMac, event.recording);
    if (pausedForReset_.load(std::memory_order_acquire)) return;
    lil::protocol::finalize(ack, lil::recording::kAckMessage, event.sequence);
    EspNowRxEvent reply{}; reply.archiveReply = true;
    reply.generation = event.generation;
    memcpy(reply.sourceMac, event.sourceMac, 6); memcpy(reply.data, &ack, sizeof(ack));
    reply.length = sizeof(ack);
    // Prioritize archive ACKs ahead of queued telemetry so the sensor does not
    // time out and replay an already-durable flash record.
#ifdef ESP_PLATFORM
    if (xQueueSendToFront(receiveQueue_, &reply, pdMS_TO_TICKS(5)) != pdTRUE) {
      droppedPackets_.fetch_add(1, std::memory_order_relaxed);
    }
#else
    if (xQueueSend(receiveQueue_, &reply, 0) != pdTRUE) {
      droppedPackets_.fetch_add(1, std::memory_order_relaxed);
    }
#endif
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
  // The callback has checked the header shape and exact size for this type.
  // Dispatch into out-of-line helpers so mutually exclusive packet buffers do
  // not stack-accumulate on the latency-sensitive gateway task.
  const auto type = static_cast<lil::protocol::MessageType>(
      event.data[offsetof(lil::protocol::PacketHeader, type)]);
  if (event.archiveReply || type == lil::recording::kStatusMessage ||
      type == lil::recording::kRecordMessage) {
    handleRecording(event, type);
    return;
  }
  if (type == lil::ota::kMessageType) {
    handleOta(event);
    return;
  }
  handleTelemetry(event);
}

void EspNowGateway::handleRecording(const EspNowRxEvent& event,
                                    lil::protocol::MessageType type) {
    SensorConfig config{};
    uint32_t generation = 0;
    if (!registry_->findConfig(event.sourceMac, config, &generation) || !config.provisioned ||
        (event.archiveReply && event.generation != generation)) return;
    lil::recording::AckPacket ack{};
    if (event.archiveReply) memcpy(&ack, event.data, sizeof(ack));
    else if (type == lil::recording::kRecordMessage) {
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
      if (now.tv_sec >= 1577836800) {
        ack.payload.stationEpochMs =
            uint64_t(now.tv_sec) * 1000ULL +
            lil::timing::signedUsToMs(now.tv_usec);
      }
      lil::protocol::finalize(ack, lil::recording::kAckMessage, request.header.sequence);
    }
    sendResponse(event.sourceMac, &ack, sizeof(ack));
}

void EspNowGateway::handleOta(const EspNowRxEvent& event) {
    lil::ota::Packet request{};
    SensorConfig config{};
    if (!lil::ota::decodeRequest(event.data, event.length, request) ||
        !registry_->findConfig(event.sourceMac, config) || !config.provisioned) return;
#ifdef ESP_PLATFORM
    if (sendGuard_.pending()) {
      const TickType_t waitStarted = xTaskGetTickCount();
      while (sendGuard_.pending()) {
        const TickType_t elapsed = xTaskGetTickCount() - waitStarted;
        if (elapsed >= kOtaBusyWait) break;
        ulTaskNotifyTake(pdTRUE, kOtaBusyWait - elapsed);
      }
    }
#endif
    if (sendGuard_.pending()) {
      replyBusyDrops_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    if (request.payload.op == lil::ota::Op::Read &&
        request.payload.size == lil::ota::kLargeBlockSize) {
      if (!otaWireBuffer_) {
        otaWireBuffer_.reset(new (std::nothrow) uint8_t[sizeof(lil::ota::LargePacket)]);
      }
      if (!otaWireBuffer_) return;
      const size_t wireLen = otaService.respondWire(
          event.sourceMac, request, otaWireBuffer_.get(), sizeof(lil::ota::LargePacket));
      if (wireLen > 0) {
        sendResponse(event.sourceMac, otaWireBuffer_.get(), wireLen);
      }
      return;
    }
    lil::ota::Packet response{};
    if (otaService.respond(event.sourceMac, request, response)) {
      sendResponse(event.sourceMac, &response, sizeof(response));
    }
}

void EspNowGateway::handleTelemetry(const EspNowRxEvent& event) {
  lil::protocol::TelemetryPacket packet{};
  if (event.length > sizeof(packet) || !lil::protocol::validatePacket(event.data, event.length,
          lil::protocol::MessageType::kTelemetry, event.length - sizeof(lil::protocol::PacketHeader))) {
    invalidPackets_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  memcpy(&packet, event.data, event.length);
  if (!lil::protocol::validTelemetryValues(packet.payload)) {
    invalidPackets_.fetch_add(1, std::memory_order_relaxed);
    return;
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
  response.payload.flags = responseConfig.pendingFlags & (kSensorCommandFlags | lil::protocol::kEnrollmentActive);
  if (wifi_->stationConnected() && !wifi_->networkScanInProgress()) {
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

  sendResponse(event.sourceMac, &response, sizeof(response));

  const bool liveTelemetry = lil::protocol::isLiveSensor(packet.payload.sensorType) &&
      !(packet.payload.flags & lil::protocol::kBatteryProtectionActive) &&
      !(registry_->normalHistory() && usesNormalHistory(packet.payload));
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
    persistence.queueCloudUpload = !lil::protocol::isLiveSensor(packet.payload.sensorType) &&
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

void EspNowGateway::sendResponse(const uint8_t mac[6], const void* packet, size_t size) {
  // If the preceding send outlived its wait, keep receiving and persisting
  // data. Sensors can retry a missing application ACK with the same sequence.
  if (sendGuard_.pending()) {
    replyBusyDrops_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (!ensurePeer(mac)) return;
  ulTaskNotifyTake(pdTRUE, 0);
  sendGuard_.start(millis());
  esp_err_t sendErr = esp_now_send(mac, static_cast<const uint8_t*>(packet), size);
  if (sendErr == ESP_ERR_ESPNOW_NOT_FOUND) {
    // Peer cache became stale; refresh the ESP-NOW peer entry and retry once.
    peerCacheCount_ = 0;
    if (ensurePeer(mac)) {
      sendErr = esp_now_send(mac, static_cast<const uint8_t*>(packet), size);
    }
  }
#ifdef ESP_PLATFORM
  if (sendErr == ESP_ERR_ESPNOW_NO_MEM) {
    vTaskDelay(pdMS_TO_TICKS(5));
    sendErr = esp_now_send(mac, static_cast<const uint8_t*>(packet), size);
  }
#endif
  if (sendErr != ESP_OK) {
    sendGuard_.complete();
    peerCacheCount_ = 0;
    peerFailures_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  // Normally finish the reply before enqueueing flash work. A stale task
  // notification can wake this wait, so completion is always checked itself.
  const TickType_t completionWait = size > lil::protocol::kMaxPacketSize
                                        ? kLargeResponseCompletionWait
                                        : kResponseCompletionWait;
  const TickType_t started = xTaskGetTickCount();
  while (sendGuard_.pending()) {
    const TickType_t elapsed = xTaskGetTickCount() - started;
    if (elapsed >= completionWait) break;
    ulTaskNotifyTake(pdTRUE, completionWait - elapsed);
  }
}

bool EspNowGateway::ensurePeer(const uint8_t mac[6]) {
  uint64_t macKey = 0;
  memcpy(&macKey, mac, 6);
  for (size_t i = 0; i < peerCacheCount_; ++i) {
    if (peerCache_[i] == macKey) return true;
  }
  if (esp_now_is_peer_exist(mac)) {
    if (peerCacheCount_ < sizeof(peerCache_) / sizeof(peerCache_[0])) {
      peerCache_[peerCacheCount_++] = macKey;
    }
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
  if (count > 0) {
    for (size_t i = 0; i < count; ++i) esp_now_del_peer(obsolete[i]);
    peerCacheCount_ = 0;
  }
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;  // Use the AP/STA radio's current channel.
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  const esp_err_t result = esp_now_add_peer(&peer);
  const bool success = result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST;
  if (success) {
    // Keep the default 1 Mbps 802.11b station reply rate so weak-RF sensors
    // that fall back to 1 Mbps always receive config, recording ACK and OTA
    // replies at maximum range.
    if (peerCacheCount_ < sizeof(peerCache_) / sizeof(peerCache_[0])) {
      peerCache_[peerCacheCount_++] = macKey;
    }
  } else {
    peerFailures_.fetch_add(1, std::memory_order_relaxed);
  }
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

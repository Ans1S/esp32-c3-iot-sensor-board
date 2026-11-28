#include "espnow_transport.h"
#include "station_assignment.h"

#include <WiFi.h>
#include <esp_random.h>
#include <esp_wifi.h>
#include <freertos/task.h>

#ifdef ESP_PLATFORM
#include "low_power_wait.h"
#endif
#include "sensor_log.h"
#include "radio_benchmark.h"

namespace sensor {

EspNowTransport* EspNowTransport::instance_ = nullptr;

namespace {
constexpr uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF,
                                      0xFF, 0xFF, 0xFF};
constexpr uint32_t kSendWaitMs = 180;
constexpr uint32_t kResponseWaitMs = 260;
constexpr uint32_t kLiveResponseWaitMs = 60;
constexpr TickType_t kOtaResponseWait = pdMS_TO_TICKS(700);
constexpr TickType_t kOtaMacFailGrace = pdMS_TO_TICKS(60);
constexpr TickType_t kRecordingResponseWait = pdMS_TO_TICKS(500);
constexpr TickType_t kRecordingMacFailGrace = pdMS_TO_TICKS(60);
constexpr int8_t kWeakRssiThresholdDbm = -80;
constexpr int8_t kWeakTxPowerThresholdQuarterDbm = 60;
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
  if (responses_ != nullptr) {
    end();
  }
  responses_ = xQueueCreate(4, sizeof(ResponseEvent));
  if (responses_ == nullptr) return false;
#ifdef ESP_PLATFORM
  if (!lightSleepInhibited_) {
    pushLowPowerWaitInhibit();
    lightSleepInhibited_ = true;
  }
#endif
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
    } else if (previous == SendState::kOta && status != ESP_NOW_SEND_SUCCESS) {
      const auto responses = instance_->otaResponses_.load(std::memory_order_acquire);
      if (responses != nullptr) {
        OtaEvent event{};
        event.completion = true;
        xQueueSend(responses, &event, 0);
      }
    } else if (previous == SendState::kRecording && status != ESP_NOW_SEND_SUCCESS) {
      const auto responses = instance_->recordingResponses_.load(std::memory_order_acquire);
      if (responses != nullptr) {
        RecordingEvent event{};
        event.completion = true;
        xQueueSend(responses, &event, 0);
      }
    }
#ifdef ESP_PLATFORM
    if (previous == SendState::kTelemetry || previous == SendState::kOta ||
        previous == SendState::kRecording || previous == SendState::kAuxiliary) {
      TaskHandle_t waiter = instance_->auxWaiter_.load(std::memory_order_acquire);
      if (waiter != nullptr) {
        xTaskNotifyGive(waiter);
      }
      if (previous == SendState::kAuxiliary && status != ESP_NOW_SEND_SUCCESS) {
        TaskHandle_t largeWaiter =
            instance_->otaLargeNotify_.load(std::memory_order_acquire);
        if (largeWaiter != nullptr) {
          xTaskNotifyGive(largeWaiter);
        }
      }
    }
#endif
  }
}

void EspNowTransport::receiveCallback(const esp_now_recv_info_t* info,
                                      const uint8_t* data, int length) {
  lil::protocol::MessageType type;
  const size_t received = static_cast<size_t>(length);
  if (!instance_ || !info || !info->src_addr || !lil::protocol::readPacketType(data, received, type)) return;
  // Share stack space across mutually exclusive event types inside the Wi-Fi task.
  union EventSlot {
    EventSlot() {}
    RecordingEvent recording;
    OtaEvent ota;
    ResponseEvent response;
  } slot;
  if (type == lil::recording::kAckMessage && received == sizeof(lil::recording::AckPacket)) {
    const auto responses = instance_->recordingResponses_.load(std::memory_order_acquire);
    if (!responses || !lil::protocol::validatePacket(data, received, type, sizeof(lil::recording::Ack))) return;
    slot.recording.completion = false;
    memcpy(slot.recording.mac, info->src_addr, 6);
    memcpy(&slot.recording.packet, data, sizeof(slot.recording.packet));
    xQueueSend(responses, &slot.recording, 0);
    return;
  }
  if (type == lil::ota::kMessageType) {
#ifdef ESP_PLATFORM
    constexpr size_t kLargeHeaderBytes =
        sizeof(lil::protocol::PacketHeader) + offsetof(lil::ota::LargePayload, data);
    if (received >= kLargeHeaderBytes &&
        received <= sizeof(lil::ota::LargePacket) &&
        lil::protocol::validatePacket(data, received, type,
                                      received - sizeof(lil::protocol::PacketHeader))) {
      lil::protocol::PacketHeader hdr{};
      memcpy(&hdr, data, sizeof(hdr));
      uint32_t nonce = 0;
      uint16_t dataSize = 0;
      memcpy(&nonce, data + sizeof(hdr) + offsetof(lil::ota::LargePayload, nonce), sizeof(nonce));
      memcpy(&dataSize, data + sizeof(hdr) + offsetof(lil::ota::LargePayload, size), sizeof(dataSize));
      if (dataSize <= lil::ota::kLargeBlockSize &&
          received >= kLargeHeaderBytes + dataSize) {
        TaskHandle_t waiterToNotify = nullptr;
        portENTER_CRITICAL(&instance_->otaLargeMux_);
        if (instance_->otaLargeWaiter_ != nullptr &&
            instance_->otaLargeTarget_ != nullptr &&
            !instance_->otaLargeReady_ &&
            memcmp(info->src_addr, instance_->otaLargeMac_, 6) == 0 &&
            hdr.sequence == instance_->otaLargeSeq_ &&
            nonce == instance_->otaLargeNonce_) {
          memcpy(instance_->otaLargeTarget_, data, received);
          instance_->otaLargeReady_ = true;
          waiterToNotify = instance_->otaLargeWaiter_;
        }
        portEXIT_CRITICAL(&instance_->otaLargeMux_);
        if (waiterToNotify != nullptr) {
          xTaskNotifyGive(waiterToNotify);
          return;
        }
      }
    }
#endif
    if (received == sizeof(lil::ota::Packet)) {
      const auto responses = instance_->otaResponses_.load(std::memory_order_acquire);
      if (!responses || !lil::protocol::validatePacket(data, received, type, sizeof(lil::ota::Payload))) return;
      slot.ota.completion = false;
      memcpy(slot.ota.mac, info->src_addr, 6);
      memcpy(&slot.ota.packet, data, sizeof(slot.ota.packet));
      xQueueSend(responses, &slot.ota, 0);
    }
    return;
  }
  if (type != lil::protocol::MessageType::kConfigResponse ||
      received != sizeof(lil::protocol::ConfigResponsePacket)) {
    return;
  }

  memset(static_cast<void*>(&slot.response), 0, sizeof(slot.response));
  memcpy(&slot.response.packet, data, sizeof(slot.response.packet));
  memcpy(slot.response.sourceMac, info->src_addr, 6);
  slot.response.rssi = info->rx_ctrl != nullptr ? info->rx_ctrl->rssi : 0;
  xQueueSend(instance_->responses_, &slot.response, 0);
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
        !usableStationMac(event.sourceMac) ||
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

namespace {
bool enrollmentOffer(const ExchangeResult& result) {
  return result.config.provisioned || (result.config.flags & lil::protocol::kFactoryReset);
}
}

void EspNowTransport::preferEnrollmentReply(ExchangeResult& result) {
  if (enrollmentOffer(result)) return;
  // A configured station can reply just after an idle neighbour.
  const TickType_t start = xTaskGetTickCount();
  const TickType_t budget = pdMS_TO_TICKS(25);
  for (;;) {
    const TickType_t elapsed = xTaskGetTickCount() - start;
    if (elapsed >= budget) return;
    ExchangeResult other{};
    if (!receiveResponse(other, kBroadcastMac, budget - elapsed)) continue;
    if (enrollmentOffer(other) || (other.config.flags & lil::protocol::kEnrollmentActive)) {
      result.config = other.config;
      result.stationRssi = other.stationRssi;
      if (enrollmentOffer(result)) return;
    }
  }
}

bool EspNowTransport::sendPending() const {
  const auto state = sendState_.load(std::memory_order_acquire);
  return state == SendState::kTelemetry || state == SendState::kOta ||
         state == SendState::kRecording || state == SendState::kAuxiliary;
}

bool EspNowTransport::sendStalled(uint32_t nowMs, uint32_t timeoutMs) const {
  return sendPending() && static_cast<uint32_t>(nowMs - sendStartedMs_) >= timeoutMs;
}

bool EspNowTransport::waitForAuxiliarySend(uint32_t maxWaitMs,
                                           bool relativeToSendStart) {
  const uint32_t waitStarted = relativeToSendStart ? sendStartedMs_ : millis();
#ifdef ESP_PLATFORM
  const TaskHandle_t current = xTaskGetCurrentTaskHandle();
  auxWaiter_.store(current, std::memory_order_release);
  while (sendPending()) {
    const uint32_t elapsed = millis() - waitStarted;
    if (elapsed >= maxWaitMs) {
      auxWaiter_.store(nullptr, std::memory_order_release);
      return false;
    }
    const uint32_t remaining = maxWaitMs - elapsed;
    const TickType_t ticks = pdMS_TO_TICKS(remaining);
    ulTaskNotifyTake(pdTRUE, ticks > 0 ? ticks : 1);
  }
  auxWaiter_.store(nullptr, std::memory_order_release);
  return true;
#else
  while (sendPending()) {
    if (millis() - waitStarted >= maxWaitMs) return false;
    delay(1);
  }
  return true;
#endif
}

bool EspNowTransport::setChannelCached(uint8_t channel) {
  if (currentChannel_ == channel) return true;
  if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
  currentChannel_ = channel;
  return true;
}

void EspNowTransport::setTxPowerCached(int8_t txPowerQuarterDbm) {
  if (currentTxPower_ == txPowerQuarterDbm) return;
  if (esp_wifi_set_max_tx_power(txPowerQuarterDbm) == ESP_OK) {
    currentTxPower_ = txPowerQuarterDbm;
  } else {
    SENSOR_LOG_PRINTF(
        "[ESP-NOW] Could not set requested TX power %d.%02d dBm\n",
        txPowerQuarterDbm / 4, (abs(txPowerQuarterDbm) % 4) * 25);
  }
}

void EspNowTransport::setPeerRate(const uint8_t mac[6], bool highRate) {
#ifdef ESP_PLATFORM
  if (memcmp(mac, kBroadcastMac, 6) == 0) return;
  if (peerHighRate_ == highRate && memcmp(ratedPeerMac_, mac, 6) == 0) return;
  esp_now_rate_config_t cfg{};
  cfg.phymode = highRate ? WIFI_PHY_MODE_11G : WIFI_PHY_MODE_11B;
  cfg.rate = highRate ? WIFI_PHY_RATE_6M : WIFI_PHY_RATE_1M_L;
  cfg.ersu = false;
  cfg.dcm = false;
  if (esp_now_set_peer_rate_config(mac, &cfg) == ESP_OK) {
    peerHighRate_ = highRate;
    memcpy(ratedPeerMac_, mac, 6);
  }
#else
  (void)mac;
  (void)highRate;
#endif
}

bool EspNowTransport::configureLink(uint8_t channel, int8_t txPowerQuarterDbm) {
  if (!responses_ || channel < 1 || channel > 13) return false;
  if (!waitForAuxiliarySend(kSendWaitMs, true)) return false;
  if (!setChannelCached(channel)) return false;
  const int8_t power = constrain(txPowerQuarterDbm, 8, kMaximumTxPowerQuarterDbm);
  baseTxPower_ = power;
  otaTxPower_ = power;
  setTxPowerCached(power);
#ifdef ESP_PLATFORM
  if (power >= kWeakTxPowerThresholdQuarterDbm) {
    otaLowRate_ = true;
  }
#endif
  return true;
}

bool EspNowTransport::sendFrame(const uint8_t destination[6], const void* data,
                                size_t size, SendState mode) {
  sendStartedMs_ = millis();
  sendState_.store(mode, std::memory_order_release);
  esp_err_t err = esp_now_send(destination, static_cast<const uint8_t*>(data), size);
  if (err == ESP_ERR_ESPNOW_NOT_FOUND && ensurePeer(destination)) {
    err = esp_now_send(destination, static_cast<const uint8_t*>(data), size);
  }
  if (err == ESP_OK) return true;
  sendState_.store(SendState::kIdle, std::memory_order_release);
  return false;
}

void EspNowTransport::waitForTelemetrySend(ExchangeResult& result,
                                           const uint8_t destination[6]) {
  for (;;) {
    const auto state = sendState_.load(std::memory_order_acquire);
    if (state != SendState::kTelemetry && state != SendState::kOta &&
        state != SendState::kRecording && state != SendState::kAuxiliary) return;
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
    const SensorRuntimeConfig& config, bool trialBoot, bool liveStream, bool slowDiscovery) {
  ExchangeResult result{};
  PreparedTelemetry prepared(millis());
  expectedSequence_ = packet.header.sequence;
  if (!hasStationAssignment(config)) {
    discover(packet, config, prepared, result, slowDiscovery);
    return result;
  }
  const bool stream = liveStream && !trialBoot;
  if (tryChannel(config.wifiChannel, config.stationMac, packet, prepared, result,
                 config.txPowerQuarterDbm, stream ? 1 : kKnownChannelAttempts, false,
                 stream ? kLiveResponseWaitMs : 0)) return result;
  // A trial boot requires an application ACK before confirming the image.
  const uint32_t started = millis();
  while (trialBoot && millis() - started < 5000UL && !sendPending()) {
    if (receiveResponse(result, config.stationMac, pdMS_TO_TICKS(50)) ||
        tryChannel(config.wifiChannel, config.stationMac, packet, prepared, result,
                   config.txPowerQuarterDbm, 1)) break;
  }
  return result;
}

void EspNowTransport::discover(const lil::protocol::TelemetryPacket& packet,
                               const SensorRuntimeConfig& config,
                               PreparedTelemetry& prepared, ExchangeResult& result, bool slowDiscovery) {
  const uint8_t preferred = config.wifiChannel >= 1 && config.wifiChannel <= 13 ? config.wifiChannel : 1;
  ExchangeResult pendingReply{};
  const bool rotate = (packet.header.sequence & 3U) == 0;
  const auto selectReply = [&] {
    preferEnrollmentReply(result);
    const bool invited = (result.config.flags & lil::protocol::kEnrollmentActive) != 0;
    if (enrollmentOffer(result) || (!rotate && (invited || !slowDiscovery))) return true;
    // A slow wake must look past an idle station within the operator's six-minute
    // window. During rapid discovery do this only on every fourth sequence.
    if (!pendingReply.configReceived || (invited && !(pendingReply.config.flags & lil::protocol::kEnrollmentActive))) {
      pendingReply = result;
    }
    result = {};
    return false;
  };
  if (tryChannel(preferred, kBroadcastMac, packet, prepared, result,
                 config.txPowerQuarterDbm, kBroadcastAttempts) && selectReply()) return;
  const uint8_t scanStart =
      static_cast<uint8_t>((packet.header.sequence *
                            kDiscoveryChannelStride) % 13U) +
      1U;
  // An unprovisioned sensor scans the complete 2.4 GHz channel range during
  // every discovery wake. Only the wake interval becomes slower after the
  // initial pairing window; channel coverage must not become incomplete.
  uint8_t channel = scanStart;
  for (uint8_t remaining = 13; remaining; --remaining, channel = channel == 13 ? 1 : channel + 1) {
    if (channel == preferred) {
      continue;
    }
    if (tryChannel(channel, kBroadcastMac, packet, prepared, result,
                   config.txPowerQuarterDbm, kBroadcastAttempts, false,
                   pendingReply.configReceived ? kLiveResponseWaitMs : 0) && selectReply()) return;
  }
  if (pendingReply.configReceived) result = pendingReply;
}

ExchangeResult EspNowTransport::acknowledgeReset(
    const lil::protocol::TelemetryPacket& receipt,
    const lil::protocol::ConfigResponsePayload& request) {
  ExchangeResult result{};
  if (!usableStationMac(request.stationMac) || request.wifiChannel < 1 || request.wifiChannel > 13) return result;
  expectedSequence_ = receipt.header.sequence;
  PreparedTelemetry prepared(millis());
  tryChannel(request.wifiChannel, request.stationMac, receipt, prepared, result,
             request.txPowerQuarterDbm, 1);
  return result;
}

ExchangeResult EspNowTransport::exchangeLpChannel(
    const lil::protocol::TelemetryPacket& packet,
    const SensorRuntimeConfig& config, uint8_t channel,
    bool recoveryAttempt) {
  ExchangeResult result{};
  if (!hasStationAssignment(config) || channel < 1 ||
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
    bool forceMaximumPower, uint32_t maxResponseWaitMs) {
  // Never retune or enqueue another frame while its callback is outstanding.
  waitForTelemetrySend(result, destination);
  if (result.configReceived || receiveResponse(result, destination, 0)) return true;
  if (sendPending()) return false;
  if (!setChannelCached(channel) || !ensurePeer(destination)) {
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
  const bool live = optical || lil::protocol::isLiveSensor(packet.payload.sensorType);
  const bool extended = lil::protocol::hasExtendedPayload(packet.payload.sensorType) ||
      (packet.payload.capabilities &
       (lil::protocol::kSpectral | lil::protocol::kDistance | lil::protocol::kAirQualityGas)) != 0;
  const size_t payloadSize = (extended || live) ? sizeof(packet.payload) :
      motion ? lil::protocol::kMotionTelemetryPayloadSize : lil::protocol::kLegacyTelemetryPayloadSize;
  const size_t wireSize = sizeof(lil::protocol::PacketHeader) + payloadSize;
  const bool broadcast = memcmp(destination, kBroadcastMac, 6) == 0;
  const bool weakLinkHint = forceMaximumPower ||
      baseTxPowerQuarterDbm >= kWeakTxPowerThresholdQuarterDbm ||
      (packet.payload.lastStationRssi != 0 &&
       packet.payload.lastStationRssi <= kWeakRssiThresholdDbm);
  baseTxPower_ = constrain(baseTxPowerQuarterDbm, 8, kMaximumTxPowerQuarterDbm);
  auto noteLinkQuality = [&](uint8_t attemptIndex) {
#ifdef ESP_PLATFORM
    if (!broadcast &&
        (attemptIndex > 0 || weakLinkHint ||
         (result.stationRssi != 0 && result.stationRssi <= kWeakRssiThresholdDbm))) {
      otaLowRate_ = true;
      recordingLowRate_ = true;
    }
#else
    (void)attemptIndex;
#endif
  };
  for (uint8_t attempt = 0; attempt < attempts; ++attempt) {
    const int8_t txPower = forceMaximumPower
                               ? kMaximumTxPowerQuarterDbm
                               : txPowerForAttempt(baseTxPowerQuarterDbm,
                                                   attempt, broadcast);
    setTxPowerCached(txPower);
    setPeerRate(destination, !broadcast && attempt == 0 && !weakLinkHint);
    if (attempt > 0) {
      const uint32_t jitterMs =
          static_cast<uint32_t>((static_cast<uint64_t>(esp_random()) * 35ULL) >> 32);
      if (receiveResponse(result, destination,
                          pdMS_TO_TICKS(8U + jitterMs))) {
        noteLinkQuality(attempt);
        return true;
      }
    }
    if (receiveResponse(result, destination, 0)) {
      noteLinkQuality(attempt);
      return true;
    }
    const bool refreshAge = live;
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
    if (!sendFrame(destination, &wirePacket, wireSize, SendState::kTelemetry)) continue;
    WCH_RADIO_BENCHMARK_FRAME();

    waitForTelemetrySend(result, destination);
    const auto state = sendState_.load(std::memory_order_acquire);
    result.delivered = result.delivered || state == SendState::kSuccess;
    if (result.configReceived) {
      noteLinkQuality(attempt);
      return true;
    }
    // A missing MAC ACK can accompany a valid application response. Retain a
    // short grace window after failure, and the full window after delivery.
    const uint32_t configuredWaitMs = maxResponseWaitMs > 0 ? maxResponseWaitMs : kResponseWaitMs;
    const uint32_t responseWaitMs = !broadcast && state == SendState::kFailure ? min<uint32_t>(30, configuredWaitMs) : configuredWaitMs;
    const uint32_t responseStarted = millis();
    for (;;) {
      const uint32_t elapsed = millis() - responseStarted;
      if (elapsed >= responseWaitMs) break;
      const uint32_t remaining = responseWaitMs - elapsed;
      if (receiveResponse(result, destination,
                          pdMS_TO_TICKS(remaining))) {
        noteLinkQuality(attempt);
        return true;
      }
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
#ifdef ESP_PLATFORM
  auxWaiter_.store(nullptr, std::memory_order_release);
  otaLargeNotify_.store(nullptr, std::memory_order_release);
  portENTER_CRITICAL(&otaLargeMux_);
  otaLargeWaiter_ = nullptr;
  otaLargeTarget_ = nullptr;
  otaLargeReady_ = false;
  portEXIT_CRITICAL(&otaLargeMux_);
#endif
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
  currentChannel_ = 0;
  currentTxPower_ = 0;
  baseTxPower_ = 52;
  otaTxPower_ = 0;
  recordingFailures_ = 0;
#ifdef ESP_PLATFORM
  peerHighRate_ = false;
  otaLowRate_ = false;
  recordingLowRate_ = false;
  memset(ratedPeerMac_, 0, sizeof(ratedPeerMac_));
#endif
  if (WiFi.getMode() != WIFI_OFF) {
    WiFi.mode(WIFI_OFF);
  }
#ifdef ESP_PLATFORM
  if (lightSleepInhibited_) {
    popLowPowerWaitInhibit();
    lightSleepInhibited_ = false;
  }
#endif
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
  OtaEvent event{};
#ifdef ESP_PLATFORM
  xQueueReset(responses);
#else
  while (xQueueReceive(responses, &event, 0) == pdTRUE) {}
#endif
  const int8_t startPower = otaTxPower_ > 0 ? otaTxPower_ :
      (currentTxPower_ > 0 ? currentTxPower_ : baseTxPower_);
  for (uint8_t attempt = 0; attempt < 5; ++attempt) {
    const int8_t attemptPower = txPowerForAttempt(startPower, attempt, false);
    setTxPowerCached(attemptPower);
#ifdef ESP_PLATFORM
    if (attempt > 0) otaLowRate_ = true;
    setPeerRate(mac, attempt == 0 && !otaLowRate_);
#else
    setPeerRate(mac, attempt == 0);
#endif
    if (!waitForAuxiliarySend(500, false)) return false;
#ifdef ESP_PLATFORM
    if (attempt > 0) {
      const uint32_t jitterMs =
          10U + static_cast<uint32_t>((static_cast<uint64_t>(esp_random()) * 31ULL) >> 32);
      const TickType_t jitterStart = xTaskGetTickCount();
      const TickType_t jitterTicks = pdMS_TO_TICKS(jitterMs);
      for (;;) {
        const TickType_t elapsed = xTaskGetTickCount() - jitterStart;
        if (elapsed >= jitterTicks ||
            xQueueReceive(responses, &event, jitterTicks - elapsed) != pdTRUE) break;
        if (event.completion) continue;
        if (!memcmp(mac, event.mac, 6) &&
            event.packet.header.sequence == request.header.sequence &&
            event.packet.payload.nonce == request.payload.nonce) {
          response = event.packet;
          otaTxPower_ = attemptPower;
          return true;
        }
      }
    }
#endif
    if (!sendFrame(mac, &request, requestLength, SendState::kOta)) {
      delay(20); continue;
    }
    const TickType_t started = xTaskGetTickCount();
    TickType_t waitLimit = kOtaResponseWait;
    for (;;) {
      const TickType_t elapsed = xTaskGetTickCount() - started;
      if (elapsed >= waitLimit ||
          xQueueReceive(responses, &event, waitLimit - elapsed) != pdTRUE) break;
      if (event.completion) {
        // MAC delivery failed; retain a brief grace window for an already-sent
        // station reply before escalating power and retrying.
        const TickType_t graceEnd = (xTaskGetTickCount() - started) + kOtaMacFailGrace;
        if (graceEnd < waitLimit) waitLimit = graceEnd;
        continue;
      }
      // receiveCallback already verified the packet header, size and CRC32.
      if (!memcmp(mac, event.mac, 6) &&
          event.packet.header.sequence == request.header.sequence &&
          event.packet.payload.nonce == request.payload.nonce) {
        if (attempt > 0) otaTxPower_ = attemptPower;
        response = event.packet; return true;
      }
    }
  }
  return false;
}

bool EspNowTransport::otaExchangeLarge(const uint8_t mac[6],
    const lil::ota::Packet& request, lil::ota::LargePacket& response) {
#ifdef ESP_PLATFORM
  const size_t requestLength = sizeof(lil::protocol::PacketHeader) + request.header.payloadSize;
  if (!lil::ota::requestSizeSupported(requestLength) || !ensurePeer(mac)) return false;
  const int8_t startPower = otaTxPower_ > 0 ? otaTxPower_ :
      (currentTxPower_ > 0 ? currentTxPower_ : baseTxPower_);
  for (uint8_t attempt = 0; attempt < 5; ++attempt) {
    const int8_t attemptPower = txPowerForAttempt(startPower, attempt, false);
    setTxPowerCached(attemptPower);
    if (attempt > 0) otaLowRate_ = true;
    setPeerRate(mac, attempt == 0 && !otaLowRate_);
    if (!waitForAuxiliarySend(500, false)) return false;
    if (attempt > 0) {
      const uint32_t jitterMs =
          10U + static_cast<uint32_t>((static_cast<uint64_t>(esp_random()) * 31ULL) >> 32);
      vTaskDelay(pdMS_TO_TICKS(jitterMs));
    }
    ulTaskNotifyTake(pdTRUE, 0);
    const TaskHandle_t current = xTaskGetCurrentTaskHandle();
    portENTER_CRITICAL(&otaLargeMux_);
    otaLargeTarget_ = &response;
    memcpy(otaLargeMac_, mac, 6);
    otaLargeSeq_ = request.header.sequence;
    otaLargeNonce_ = request.payload.nonce;
    otaLargeReady_ = false;
    otaLargeWaiter_ = current;
    portEXIT_CRITICAL(&otaLargeMux_);
    otaLargeNotify_.store(current, std::memory_order_release);
    if (!sendFrame(mac, &request, requestLength, SendState::kAuxiliary)) {
      otaLargeNotify_.store(nullptr, std::memory_order_release);
      portENTER_CRITICAL(&otaLargeMux_);
      otaLargeWaiter_ = nullptr;
      otaLargeTarget_ = nullptr;
      portEXIT_CRITICAL(&otaLargeMux_);
      delay(20);
      continue;
    }
    const TickType_t started = xTaskGetTickCount();
    TickType_t waitLimit = kOtaResponseWait;
    bool macFailureNoted = false;
    bool ready = false;
    for (;;) {
      portENTER_CRITICAL(&otaLargeMux_);
      ready = otaLargeReady_;
      portEXIT_CRITICAL(&otaLargeMux_);
      if (ready) break;
      const TickType_t elapsed = xTaskGetTickCount() - started;
      if (!macFailureNoted &&
          sendState_.load(std::memory_order_acquire) == SendState::kFailure) {
        macFailureNoted = true;
        const TickType_t graceEnd = elapsed + kOtaMacFailGrace;
        if (graceEnd < waitLimit) waitLimit = graceEnd;
      }
      if (elapsed >= waitLimit) break;
      ulTaskNotifyTake(pdTRUE, waitLimit - elapsed);
    }
    otaLargeNotify_.store(nullptr, std::memory_order_release);
    portENTER_CRITICAL(&otaLargeMux_);
    ready = otaLargeReady_;
    otaLargeWaiter_ = nullptr;
    otaLargeTarget_ = nullptr;
    portEXIT_CRITICAL(&otaLargeMux_);
    if (ready) {
      if (attempt > 0) otaTxPower_ = attemptPower;
      return true;
    }
  }
  return false;
#else
  (void)mac;
  (void)request;
  (void)response;
  return false;
#endif
}

bool EspNowTransport::recordingExchange(const SensorRuntimeConfig& config,
    const void* request, size_t length, uint64_t session, uint32_t sampleMs,
    uint32_t crc, lil::recording::Ack& response, uint32_t responseWaitMs) {
  if (!responses_ || !hasStationAssignment(config) ||
      !request || length < sizeof(lil::protocol::PacketHeader) ||
      length > lil::protocol::kMaxPacketSize) return false;
  // An application ACK can arrive before our MAC callback. Give that send
  // its remaining time before retuning or issuing the next archive request.
  if (!waitForAuxiliarySend(kSendWaitMs, true)) return false;
  lil::protocol::PacketHeader requestHeader{};
  memcpy(&requestHeader, request, sizeof(requestHeader));
  // Environmental wake cycles never allocate an archive-response queue.
  auto responses = recordingResponses_.load(std::memory_order_acquire);
  if (!responses) {
    responses = xQueueCreate(4, sizeof(RecordingEvent));
    recordingResponses_.store(responses, std::memory_order_release);
  }
  if (!responses || !setChannelCached(config.wifiChannel) ||
      !ensurePeer(config.stationMac)) return false;
  const int8_t txPower =
      txPowerForAttempt(config.txPowerQuarterDbm, recordingFailures_, false);
  setTxPowerCached(txPower);
#ifdef ESP_PLATFORM
  setPeerRate(config.stationMac, !recordingLowRate_ && recordingFailures_ == 0 &&
                                     txPower < kWeakTxPowerThresholdQuarterDbm);
#endif
  RecordingEvent event{};
#ifdef ESP_PLATFORM
  xQueueReset(responses);
#else
  while (xQueueReceive(responses, &event, 0) == pdTRUE) {}
#endif
  if (!sendFrame(config.stationMac, request, length, SendState::kRecording)) {
    if (recordingFailures_ < 4) ++recordingFailures_;
#ifdef ESP_PLATFORM
    recordingLowRate_ = true;
#endif
    return false;
  }
  const TickType_t started = xTaskGetTickCount();
  TickType_t waitLimit = responseWaitMs > 0 ? pdMS_TO_TICKS(responseWaitMs) : kRecordingResponseWait;
  for (;;) {
    const TickType_t elapsed = xTaskGetTickCount() - started;
    if (elapsed >= waitLimit ||
        xQueueReceive(responses, &event, waitLimit - elapsed) != pdTRUE) break;
    if (event.completion) {
      const TickType_t graceEnd = (xTaskGetTickCount() - started) + kRecordingMacFailGrace;
      if (graceEnd < waitLimit) waitLimit = graceEnd;
      continue;
    }
    const auto& ack = event.packet.payload;
    if (!memcmp(event.mac, config.stationMac, 6) &&
        event.packet.header.sequence == requestHeader.sequence && ack.session == session &&
        ack.sampleMs == sampleMs && ack.recordCrc == crc) {
      recordingFailures_ = 0;
      response = ack; return true;
    }
  }
  if (recordingFailures_ < 4) ++recordingFailures_;
#ifdef ESP_PLATFORM
  recordingLowRate_ = true;
#endif
  return false;
}

}  // namespace sensor

#include <cassert>
#include <memory>
#include <vector>

// Drive one event at a time through the production callback and task handler.
#define private public
#include "espnow_gateway.h"
#undef private
#include "ota_service.h"
#include "recording_archive.h"
#include "../station/src/espnow_gateway.cpp"

namespace {
uint8_t source[] = {2, 1, 2, 3, 4, 5};
uint8_t stationMac[] = {2, 6, 7, 8, 9, 10};
uint32_t lastRegistered = 0;
uint32_t notifications = 0;
unsigned sends = 0;
unsigned otaRequests = 0;
bool driverBusy = false;
std::vector<uint32_t> persisted;
}

void testRadioTick() { ++testMillis; }
void xTaskNotifyGive(TaskHandle_t) { ++notifications; }
uint32_t ulTaskNotifyTake(bool clear, TickType_t ticks) {
  while (!notifications && ticks--) testRadioTick();
  const uint32_t result = notifications;
  if (clear) notifications = 0;
  return result;
}
esp_err_t esp_now_send(const uint8_t* mac, const uint8_t* bytes, size_t size) {
  assert(!driverBusy && !memcmp(mac, source, 6));
  assert(size == sizeof(lil::protocol::ConfigResponsePacket));
  lil::protocol::ConfigResponsePacket reply;
  memcpy(&reply, bytes, size);
  assert(lil::protocol::validate(reply, size, lil::protocol::MessageType::kConfigResponse));
  assert(reply.payload.requestSequence == lastRegistered);
  ++sends;
  driverBusy = true;
  return ESP_OK;
}

namespace station {
OtaService otaService;
RecordingArchive recordingArchive;
bool RecordingArchive::begin() { return true; }
bool RecordingArchive::append(const uint8_t*, const lil::recording::Upload&) { return false; }
bool OtaService::respond(const uint8_t*, const lil::ota::Packet& request, lil::ota::Packet&) {
  assert(request.payload.op == lil::ota::Op::Read && request.payload.offset == 192);
  ++otaRequests;
  return false;
}
bool WifiService::stationConnected() const { return true; }
uint8_t WifiService::channel() const { return 6; }
const uint8_t* WifiService::stationMac() const { return ::stationMac; }
bool SensorRegistry::findConfig(const uint8_t* mac, SensorConfig& config, uint32_t*) const {
  config.provisioned = memcmp(mac, source, 6) == 0;
  return config.provisioned;
}
bool SensorRegistry::registerTelemetry(const uint8_t*, uint32_t sequence,
    const lil::protocol::TelemetryPayload&, int8_t, SensorConfig&, int8_t&, bool& duplicate,
    uint32_t& generation) {
  generation = 1;
  duplicate = lastRegistered == sequence;
  lastRegistered = sequence;
  return true;
}
bool SensorRegistry::persistTelemetry(const uint8_t*, uint32_t sequence,
    const lil::protocol::TelemetryPayload&, int8_t, uint32_t generation, uint32_t) {
  assert(generation == 1);
  persisted.push_back(sequence);
  return true;
}
bool SensorRegistry::needsPersistence(const uint8_t*) const { return false; }
bool SensorRegistry::generationMatches(const uint8_t*, uint32_t) const { return true; }
bool SensorRegistry::channelShared(uint32_t) const { return false; }
void SensorRegistry::updateRecording(const uint8_t*, const lil::recording::Status&) {}
void PagedHistory::flushDue() {}
bool ThingSpeakService::queue(const SensorConfig&, const lil::protocol::TelemetryPayload&,
    int8_t, uint32_t, uint32_t, bool, uint32_t) { return true; }
}

int main() {
  station::SensorRegistry registry;
  station::WifiService wifi;
  station::ThingSpeakService cloud;
  station::EspNowGateway gateway;
  assert(gateway.begin(registry, cloud, wifi));
  auto receive = [&](uint32_t sequence, size_t length = sizeof(lil::protocol::TelemetryPacket)) {
    lil::protocol::TelemetryPacket packet{};
    packet.payload.sensorType = lil::protocol::EnvironmentalSensorType::kBme280;
    lil::protocol::finalizePacket(&packet, length, lil::protocol::MessageType::kTelemetry,
                                  sequence, length - sizeof(lil::protocol::PacketHeader));
    RxControl control;
    esp_now_recv_info_t info{source, &control};
    testReceiveCallback(&info, reinterpret_cast<const uint8_t*>(&packet), length);
    station::EspNowRxEvent rx{};
    assert(xQueueReceive(gateway.receiveQueue_, &rx, 0));
    gateway.handle(rx);
    station::EspNowPersistenceEvent event{};
    while (xQueueReceive(gateway.persistenceQueue_, &event, 0)) gateway.persist(event);
  };
  receive(1);
  assert(sends == 1 && driverBusy);
  const auto now = millis();
  receive(2);
  assert((persisted == std::vector<uint32_t>{1, 2}));
  assert(sends == 1 && millis() == now && gateway.replyBusyDrops() == 1);
  assert(gateway.receivedPackets() == 2 && gateway.droppedPackets() == 0);
  driverBusy = false;
  testSendCallback(nullptr, ESP_NOW_SEND_FAIL);
  assert(gateway.replyMacFailures() == 1);
  receive(2);
  assert(sends == 2 && persisted.size() == 2);
  printf("Gateway stall: second packet persisted without an additional wait; retry ACK deduplicated\n");

  driverBusy = false;
  testSendCallback(nullptr, ESP_NOW_SEND_SUCCESS);
  RxControl control;
  esp_now_recv_info_t info{source, &control};
  lil::protocol::TelemetryPacket malformed{};
  lil::protocol::finalize(malformed, lil::protocol::MessageType::kTelemetry, 42);
  for (size_t i = 0; i < 64; ++i) {
    auto invalid = malformed;
    const size_t length = i % 4 == 0 ? 1 : sizeof(invalid);
    if (i % 4 == 1) ++invalid.header.magic;
    if (i % 4 == 2) invalid.header.type = lil::protocol::MessageType::kConfigResponse;
    if (i % 4 == 3) --invalid.header.payloadSize;
    testReceiveCallback(&info, reinterpret_cast<const uint8_t*>(&invalid), length);
  }
  printf("Gateway RX after 64 malformed packets: %zu occupied slots, %zu bytes per slot\n",
         gateway.receiveQueue_->data.size(), gateway.receiveQueue_->itemSize);
  assert(gateway.receiveQueue_->data.empty() && gateway.invalidPackets() == 64);
  uint32_t sequence = 2;
  for (size_t payloadSize : {lil::protocol::kLegacyTelemetryPayloadSize,
                            lil::protocol::kMotionTelemetryPayloadSize,
                            lil::protocol::kPrecisionTelemetryPayloadSize,
                            lil::protocol::kTimedTelemetryPayloadSize,
                            sizeof(lil::protocol::TelemetryPayload)}) {
    receive(++sequence, sizeof(lil::protocol::PacketHeader) + payloadSize);
    assert(persisted.back() == sequence);
    driverBusy = false;
    testSendCallback(nullptr, ESP_NOW_SEND_SUCCESS);
  }
  assert(gateway.droppedPackets() == 0);
  auto queueControl = [&](const auto& packet, size_t length) {
    testReceiveCallback(&info, reinterpret_cast<const uint8_t*>(&packet), length);
    station::EspNowRxEvent event{};
    assert(xQueueReceive(gateway.receiveQueue_, &event, 0));
    assert(event.length == length && !memcmp(event.data, &packet, length));
    return event;
  };
  lil::recording::StatusPacket status{};
  lil::protocol::finalize(status, lil::recording::kStatusMessage, 50);
  queueControl(status, sizeof(status));
  lil::recording::UploadPacket upload{};
  lil::protocol::finalize(upload, lil::recording::kRecordMessage, 51);
  queueControl(upload, sizeof(upload));
  lil::ota::Packet ota{};
  ota.payload.op = lil::ota::Op::Read;
  ota.payload.offset = 192;
  lil::protocol::finalize(ota, lil::ota::kMessageType, 52);
  gateway.handle(queueControl(ota, sizeof(ota)));
  const auto compactLength = lil::ota::kCompactReadPacketSize;
  lil::protocol::finalizePacket(&ota, compactLength, lil::ota::kMessageType,
                                53, lil::ota::kCompactReadPayloadSize);
  gateway.handle(queueControl(ota, compactLength));
  assert(otaRequests == 2);
  ota.header.crc32 ^= 1;
  gateway.handle(queueControl(ota, compactLength));
  ota.payload.op = lil::ota::Op::Status;
  lil::protocol::finalizePacket(&ota, compactLength, lil::ota::kMessageType,
                                54, lil::ota::kCompactReadPayloadSize);
  gateway.handle(queueControl(ota, compactLength));
  assert(otaRequests == 2);
  // Header admission alone does not authorize telemetry or update the registry.
  malformed.header.crc32 ^= 1;
  gateway.handle(queueControl(malformed, sizeof(malformed)));
  assert(gateway.invalidPackets() == 65 && lastRegistered == sequence);
  vQueueDelete(gateway.receiveQueue_);
  vQueueDelete(gateway.persistenceQueue_);
  vQueueDelete(gateway.archiveQueue_);
}

#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif
#include <assert.h>
#include <functional>
#include <vector>
#include "espnow_transport.h"
#include "WiFi.h"

struct Event { uint32_t due; std::function<void()> callback; };
std::vector<Event> events;
std::vector<uint8_t> channels;
unsigned sends = 0;
bool pending = false;
int completionDelay = 2;
int responseDelay = -1;
bool deliverySuccess = true;
unsigned badOtaReplies = 0;
bool wrongOtaSource = false, legacyStation = false;
size_t lastTelemetrySize = 0;
unsigned droppedTelemetryReplies = 0;
sensor::SensorRuntimeConfig config;

void testRadioTick() {
  ++testMillis;
  for (size_t i = 0; i < events.size();) {
    if (events[i].due <= testMillis) {
      auto event = events[i]; events.erase(events.begin() + i); event.callback();
    } else ++i;
  }
}
esp_err_t esp_wifi_set_channel(uint8_t channel, int) {
  assert(!pending); channels.push_back(channel); return ESP_OK;
}
esp_err_t esp_now_send(const uint8_t*, const uint8_t* bytes, size_t length) {
  assert(!pending); pending = true; ++sends;
  if (completionDelay >= 0) {
    events.push_back({testMillis + uint32_t(completionDelay), [] {
      pending = false;
      testSendCallback(nullptr, deliverySuccess ? ESP_NOW_SEND_SUCCESS : ESP_NOW_SEND_FAIL);
    }});
  }
  if (responseDelay >= 0) {
    lil::protocol::PacketHeader header{}; memcpy(&header, bytes, sizeof(header));
    if (header.type == lil::ota::kMessageType) {
      lil::ota::Packet request{}; memcpy(&request, bytes, sizeof(request));
      events.push_back({testMillis + uint32_t(responseDelay), [request] {
        lil::ota::Packet response{};
        response.payload.nonce = request.payload.nonce;
        response.payload.op = lil::ota::Op::Data;
        response.payload.offset = request.payload.offset;
        if (badOtaReplies) { --badOtaReplies; ++response.payload.nonce; }
        lil::protocol::finalize(response, lil::ota::kMessageType, request.header.sequence);
        uint8_t spoofed[6] = {8, 0, 0, 0, 0, 0};
        RxControl control;
        esp_now_recv_info_t info{wrongOtaSource ? spoofed : config.stationMac, &control};
        testReceiveCallback(&info, reinterpret_cast<const uint8_t*>(&response), sizeof(response));
      }});
      return ESP_OK;
    }
    lil::protocol::TelemetryPacket request{};
    assert(length <= sizeof(request));
    memcpy(&request, bytes, length);
    lastTelemetrySize = length;
    if (droppedTelemetryReplies) { --droppedTelemetryReplies; return ESP_OK; }
    assert(lil::protocol::validatePacket(bytes, length, lil::protocol::MessageType::kTelemetry,
                                         request.header.payloadSize));
    if (legacyStation && length != sizeof(lil::protocol::PacketHeader) + lil::protocol::kLegacyTelemetryPayloadSize)
      return ESP_OK; // 4.0.0 delivers a MAC ACK but ignores an extended packet.
    events.push_back({testMillis + uint32_t(responseDelay), [request] {
      lil::protocol::ConfigResponsePacket response{};
      response.payload.requestSequence = request.header.sequence;
      memcpy(response.payload.stationMac, config.stationMac, 6);
      lil::protocol::finalize(response, lil::protocol::MessageType::kConfigResponse,
                              request.header.sequence);
      RxControl control;
      esp_now_recv_info_t info{config.stationMac, &control};
      testReceiveCallback(&info, reinterpret_cast<const uint8_t*>(&response), sizeof(response));
    }});
  }
  return ESP_OK;
}
void reset() {
  events.clear(); channels.clear(); sends = 0; pending = false; testMillis = 0;
  completionDelay = 2; responseDelay = -1; deliverySuccess = true;
  badOtaReplies = 0; wrongOtaSource = false; legacyStation = false; lastTelemetrySize = 0;
  config = sensor::SensorRuntimeConfig{};
  config.provisioned = config.stationKnown = true;
  config.stationMac[0] = 2; config.wifiChannel = 6;
  WiFi.shutdowns = 0; droppedTelemetryReplies = 0;
}
int main() {
  {
    sensor::EspNowTransport radio;
    lil::ota::Packet q{}, r{};
    q.payload.nonce = 42; q.payload.op = lil::ota::Op::Read; q.payload.offset = 192;
    lil::protocol::finalize(q, lil::ota::kMessageType, 123);
    reset(); responseDelay = 4; badOtaReplies = 2;
    assert(radio.begin());
    assert(radio.otaExchange(config.stationMac, q, r) && sends == 3);
    assert(r.payload.offset == 192 && r.payload.nonce == 42);
    radio.end();
    reset(); responseDelay = 4; wrongOtaSource = true;
    assert(radio.begin());
    assert(!radio.otaExchange(config.stationMac, q, r) && sends == 5);
    radio.end();
    reset(); completionDelay = -1;
    assert(radio.begin());
    assert(!radio.otaExchange(config.stationMac, q, r) && sends == 1);
    radio.end();
  }
  lil::protocol::TelemetryPacket packet{};
  lil::protocol::finalize(packet, lil::protocol::MessageType::kTelemetry, 17);
  sensor::EspNowTransport transport;
  reset(); responseDelay = 4;
  assert(transport.begin());
  assert(transport.exchange(packet, config).configReceived && sends == 1);
  transport.end(); assert(WiFi.shutdowns == 1);

  reset(); deliverySuccess = false; responseDelay = 25;
  assert(transport.begin());
  assert(transport.exchange(packet, config).configReceived && sends == 1);
  transport.end();

  reset(); responseDelay = 275; // Arrives during the eight-ms retry backoff.
  assert(transport.begin());
  assert(transport.exchange(packet, config).configReceived && sends == 1);
  transport.end();

  reset(); completionDelay = -1;
  assert(transport.begin());
  assert(!transport.exchange(packet, config).configReceived && sends == 1);
  assert(!transport.exchangeLpChannel(packet, config, 11, true).configReceived);
  assert(channels.size() == 1 && sends == 1 && testMillis <= 440);
  transport.end();

  reset(); completionDelay = 200; responseDelay = 250;
  assert(transport.begin());
  assert(transport.exchange(packet, config).configReceived && sends == 1);
  transport.end();

  reset(); config.provisioned = config.stationKnown = false;
  assert(transport.begin());
  assert(!transport.exchange(packet, config).configReceived);
  assert(sends == 13 && channels.size() == 13);
  for (uint8_t channel = 1; channel <= 13; ++channel) {
    assert(std::count(channels.begin(), channels.end(), channel) == 1);
  }
  transport.end();
  reset(); responseDelay = 4; droppedTelemetryReplies = 3;
  assert(transport.begin());
  assert(transport.exchange(packet, config, true).configReceived && sends == 4);
  assert(testMillis < 6000);
  transport.end();
  reset(); responseDelay = 4; droppedTelemetryReplies = 100;
  assert(transport.begin());
  assert(!transport.exchange(packet, config, true).configReceived);
  assert(testMillis < 6500); // A missing application ACK never disables rollback.
  transport.end();

  // Reproduce the old station's exact-size receive gate. Both environmental
  // sensors must receive a configuration ACK, which enables boot confirmation.
  for (auto type : {lil::protocol::EnvironmentalSensorType::kBme280,
                    lil::protocol::EnvironmentalSensorType::kBme680,
                    lil::protocol::EnvironmentalSensorType::kDisabled}) {
    reset(); responseDelay = 4; legacyStation = true;
    packet.payload.sensorType = type;
    lil::protocol::finalize(packet, lil::protocol::MessageType::kTelemetry, 42);
    assert(transport.begin());
    assert(transport.exchange(packet, config).configReceived);
    assert(lastTelemetrySize == sizeof(lil::protocol::PacketHeader) + lil::protocol::kLegacyTelemetryPayloadSize);
    transport.end();
  }
  reset(); responseDelay = 4; legacyStation = true;
  packet.payload.sensorType = lil::protocol::EnvironmentalSensorType::kLsm6dsox;
  packet.payload.capabilities = lil::protocol::kMotion;
  lil::protocol::finalize(packet, lil::protocol::MessageType::kTelemetry, 43);
  assert(transport.begin());
  const auto incompatible = transport.exchange(packet, config);
  assert(incompatible.delivered && !incompatible.configReceived);
  assert(lastTelemetrySize == sizeof(packet));
  transport.end();
  reset(); responseDelay = 4;
  assert(transport.begin());
  assert(transport.exchange(packet, config).configReceived);
  transport.end();
  puts("ESP-NOW timing and V5 environmental/IMU wire compatibility tests passed");
}

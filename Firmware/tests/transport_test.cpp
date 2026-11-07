#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif
#include <assert.h>
#include <functional>
#include <vector>
#include "espnow_transport.h"
#include "WiFi.h"
#include "protocol_counter.h"

struct Event { uint32_t due; std::function<void()> callback; };
std::vector<Event> events;
std::vector<uint8_t> channels;
std::vector<std::vector<uint8_t>> telemetryFrames;
std::vector<uint32_t> telemetryFrameTimes;
unsigned sends = 0;
bool pending = false;
int completionDelay = 2;
int responseDelay = -1;
bool deliverySuccess = true;
unsigned badOtaReplies = 0;
bool wrongOtaSource = false, legacyStation = false;
bool wrongRecordingSequence = false;
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
  lil::protocol::PacketHeader wireHeader{};
  memcpy(&wireHeader, bytes, sizeof(wireHeader));
  if (wireHeader.type == lil::protocol::MessageType::kTelemetry) {
    assert(lil::protocol::validatePacket(bytes, length, wireHeader.type, wireHeader.payloadSize));
    telemetryFrames.emplace_back(bytes, bytes + length); telemetryFrameTimes.push_back(testMillis);
  }
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
    if (header.type == lil::recording::kStatusMessage) {
      lil::recording::StatusPacket request{}; memcpy(&request, bytes, sizeof(request));
      events.push_back({testMillis + uint32_t(responseDelay), [request] {
        lil::recording::AckPacket response{};
        response.payload.session = request.payload.session;
        response.payload.sampleMs = request.payload.elapsedMs;
        response.payload.stationEpochMs = 1700000000000ULL;
        lil::protocol::finalize(response, lil::recording::kAckMessage,
            request.header.sequence - (wrongRecordingSequence ? 1 : 0));
        RxControl control; esp_now_recv_info_t info{config.stationMac, &control};
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
  telemetryFrames.clear(); telemetryFrameTimes.clear();
  completionDelay = 2; responseDelay = -1; deliverySuccess = true;
  badOtaReplies = 0; wrongOtaSource = false; legacyStation = false; lastTelemetrySize = 0;
  config = sensor::SensorRuntimeConfig{};
  config.provisioned = config.stationKnown = true;
  config.stationMac[0] = 2; config.wifiChannel = 6;
  WiFi.shutdowns = 0; droppedTelemetryReplies = 0;
  wrongRecordingSequence = false;
}
int main() {
  {
    reset();
    sensor::EspNowTransport radio;
    const auto queues = testQueueCreates;
    const auto queueBytes = testQueueDataBytes;
    assert(radio.begin());
    assert(testQueueCreates - queues == (FLOW_OPTIMIZED ? 1 : 2));
    assert(testQueueDataBytes - queueBytes == (FLOW_OPTIMIZED ? 220 : 1124));
    lil::protocol::TelemetryPacket telemetry{};
    telemetry.header.sequence = 700;
    telemetry.payload.sensorType = lil::protocol::EnvironmentalSensorType::kBme280;
    testTelemetryFinalizations = testTelemetryCrcBytes = 0;
    assert(!radio.exchangeLpChannel(telemetry, config, 6, true).configReceived);
    const size_t wireBytes = sizeof(lil::protocol::PacketHeader) + lil::protocol::kLegacyTelemetryPayloadSize;
    assert(sends == 1 && testTelemetryFinalizations == (FLOW_OPTIMIZED ? 1 : 2));
    assert(testTelemetryCrcBytes == wireBytes * (FLOW_OPTIMIZED ? 1 : 2));
    printf("Radio one-attempt telemetry: %zu CRC finalizations / %zu bytes, %zu queues / %zu queue data bytes at begin\n",
           testTelemetryFinalizations, testTelemetryCrcBytes, testQueueCreates - queues, testQueueDataBytes - queueBytes);
    radio.end();
  }
  if (FLOW_OPTIMIZED) {
    reset();
    sensor::EspNowTransport radio;
    const auto queues = testQueueCreates;
    testQueueCreateFailAt = queues + 2;
    assert(radio.begin()); // OTA memory pressure cannot disable base telemetry.
    lil::ota::Packet request{}, response{};
    request.payload.nonce = 42; request.payload.op = lil::ota::Op::Read; request.payload.offset = 192;
    lil::protocol::finalize(request, lil::ota::kMessageType, 123);
    assert(!radio.otaExchange(config.stationMac, request, response) && sends == 0);
    testQueueCreateFailAt = 0; responseDelay = 4;
    assert(radio.otaExchange(config.stationMac, request, response) && sends == 1);
    const auto allocated = testQueueCreates;
    assert(radio.otaExchange(config.stationMac, request, response) && testQueueCreates == allocated);
    radio.end();
  }
  for (const auto type : {lil::protocol::EnvironmentalSensorType::kBme280,
                         lil::protocol::EnvironmentalSensorType::kAutoDetect,
                         lil::protocol::EnvironmentalSensorType::kMax30102}) {
    reset(); sensor::EspNowTransport radio;
    lil::protocol::TelemetryPacket original{};
    original.header.sequence = 991; original.payload.sensorType = type;
    if (type == lil::protocol::EnvironmentalSensorType::kAutoDetect)
      original.payload.capabilities = lil::protocol::kMotion;
    original.payload.live.count = 2; original.payload.live.acquisitionAgeMs = 100;
    original.payload.live.estimateAgeMs = UINT16_MAX;
    original.payload.live.optical[0] = {12345, 23456, 65530};
    original.payload.live.optical[1] = {34567, 45678, UINT16_MAX};
    original.payload.live.optical[2].ageMs = 42; // An unused slot is immutable.
    assert(radio.begin());
    assert(!radio.exchangeLpChannel(original, config, 6).configReceived && telemetryFrames.size() == 2);
    for (size_t i = 0; i < telemetryFrames.size(); ++i) {
      auto expected = original;
      const auto addAge = [&](uint16_t age) { return age == UINT16_MAX ? age :
          uint16_t(std::min(uint32_t(65534), uint32_t(age) + telemetryFrameTimes[i])); };
      expected.payload.live.acquisitionAgeMs = addAge(original.payload.live.acquisitionAgeMs);
      expected.payload.live.estimateAgeMs = addAge(original.payload.live.estimateAgeMs);
      for (size_t frame = 0; frame < original.payload.live.count; ++frame)
        expected.payload.live.optical[frame].ageMs = addAge(original.payload.live.optical[frame].ageMs);
      const size_t length = telemetryFrames[i].size();
      lil::protocol::finalizePacket(&expected, length, lil::protocol::MessageType::kTelemetry,
                                  original.header.sequence, length - sizeof(lil::protocol::PacketHeader));
      assert(!memcmp(&expected, telemetryFrames[i].data(), length));
    }
    assert(original.payload.live.acquisitionAgeMs == 100 && original.payload.live.optical[0].ageMs == 65530);
    radio.end();
  }
  puts("Radio wire: exact short, motion-prefix and full packet bytes across retries, saturated/unknown ages and unused slots passed");
  {
    sensor::EspNowTransport radio;
    lil::recording::StatusPacket request{};
    request.payload.session = 123; request.payload.elapsedMs = 1000;
    lil::protocol::finalize(request, lil::recording::kStatusMessage, 91);
    lil::recording::Ack ack{};
    reset(); responseDelay = 4; wrongRecordingSequence = true;
    assert(radio.begin());
    // Identical session/sample/checksum fields can recur in status requests.
    // A delayed ACK from an earlier request must not supply its old UTC anchor.
    assert(!radio.recordingExchange(config, &request, sizeof(request),
        123, 1000, 0, ack));
    assert(sends == 1 && testMillis >= 500);
    radio.end();
    reset(); responseDelay = 4;
    assert(radio.begin());
    assert(radio.recordingExchange(config, &request, sizeof(request),
        123, 1000, 0, ack) && ack.stationEpochMs == 1700000000000ULL);
    // Finish the MAC callback before shutting down the substitute.
    delay(5); radio.end();
  }
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
  reset(); responseDelay = 4;
  packet.payload.sensorType = lil::protocol::EnvironmentalSensorType::kMax30102;
  packet.payload.capabilities = lil::protocol::kOptical | lil::protocol::kHeartRate;
  packet.payload.pulse.beatsPerMinute = 72;
  lil::protocol::finalize(packet, lil::protocol::MessageType::kTelemetry, 44);
  assert(transport.begin());
  assert(transport.exchange(packet, config).configReceived);
  assert(lastTelemetrySize == sizeof(packet));
  transport.end();
  puts("ESP-NOW timing and V5 environmental/IMU wire compatibility tests passed");
}

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace lil {
namespace protocol {

constexpr uint32_t kMagic = 0x4C494C32UL;  // "LIL2"
constexpr uint8_t kVersion = 5;
constexpr size_t kMacLength = 6;
constexpr size_t kMaxPacketSize = 250;

enum class MessageType : uint8_t {
  kTelemetry = 1,
  kConfigResponse = 2,
};

enum Capability : uint16_t {
  kTemperature = 1U << 0,
  kHumidity = 1U << 1,
  kPressure = 1U << 2,
  kIaq = 1U << 3,
  kBattery = 1U << 4,
  kPcbV4PowerGates = 1U << 5,
  kGasResistance = 1U << 6,
};

enum class EnvironmentalSensorType : uint8_t {
  kAutoDetect = 0,
  kBme280 = 1,
  kBme680 = 2,
  kDisabled = 255,
};

enum class SensorOperatingMode : uint8_t {
  kUnknown = 0,
  kDiscovery = 1,
  kEnergySaving = 2,
  kChannelRecovery = 3,
};

enum ConfigFlag : uint16_t {
  kConfigNone = 0,
  kFactoryReset = 1U << 0,
  kResetIaqCalibration = 1U << 1,
};

enum TelemetryFlag : uint16_t {
  kTelemetryNone = 0,
  kSensorReadFailed = 1U << 0,
  kBatteryReadFailed = 1U << 1,
  kIaqCalibrating = 1U << 2,
  kSensorTypeMismatch = 1U << 3,
  // Set on every packet while the node is not provisioned. Such packets are
  // pairing/discovery traffic and must never be uploaded as normal telemetry.
  kDiscoveryBeacon = 1U << 4,
  kBme680RawFallback = 1U << 5,
  // Retained so stations can safely interpret packets from older sensors.
  kBme680Commissioning = 1U << 6,
  kCommissioningBlockedLowBattery = 1U << 7,
};

enum class IaqCalibrationPhase : uint8_t {
  kNotApplicable = 0,
  kStabilizing = 1,
  kLearning = 2,
  kReady = 3,
  kHighAccuracy = 4,
};

constexpr uint16_t kCalibrationTimeUnknown = UINT16_MAX;

#pragma pack(push, 1)
struct PacketHeader {
  uint32_t magic;
  uint8_t version;
  MessageType type;
  uint16_t payloadSize;
  uint32_t sequence;
  uint32_t crc32;
};

struct TelemetryPayload {
  uint32_t appliedConfigRevision;
  uint32_t bootCount;
  uint16_t capabilities;
  uint16_t flags;
  float temperatureC;
  float humidityPercent;
  float pressureHpa;
  float iaq;
  float gasResistanceOhms;
  uint16_t batteryMillivolts;
  uint8_t pcbVersion;
  SensorOperatingMode operatingMode;
  int8_t lastStationRssi;
  EnvironmentalSensorType sensorType;
  uint8_t iaqAccuracy;
  IaqCalibrationPhase iaqCalibrationPhase;
  uint16_t iaqCalibrationElapsedMinutes;
  uint16_t iaqCalibrationRemainingMinutes;
};

struct ConfigResponsePayload {
  uint32_t revision;
  uint32_t sleepIntervalSeconds;
  uint32_t requestSequence;
  uint16_t flags;
  uint8_t wifiChannel;
  int8_t txPowerQuarterDbm;
  EnvironmentalSensorType sensorType;
  float temperatureOffsetC;
  float batteryCalibrationFactor;
  uint8_t provisioned;
  uint8_t stationMac[kMacLength];
};

template <typename Payload>
struct Packet {
  PacketHeader header;
  Payload payload;
};
#pragma pack(pop)

using TelemetryPacket = Packet<TelemetryPayload>;
using ConfigResponsePacket = Packet<ConfigResponsePayload>;

static_assert(sizeof(TelemetryPacket) <= kMaxPacketSize,
              "Telemetry packet exceeds the ESP-NOW payload limit");
static_assert(sizeof(ConfigResponsePacket) <= kMaxPacketSize,
              "Config packet exceeds the ESP-NOW payload limit");

uint32_t crc32(const uint8_t* data, size_t length);
void finalizePacket(void* packet, size_t packetSize, MessageType type,
                    uint32_t sequence, uint16_t payloadSize);
bool validatePacket(const void* packet, size_t packetSize,
                    MessageType expectedType, uint16_t expectedPayloadSize);

template <typename Payload>
void finalize(Packet<Payload>& packet, MessageType type, uint32_t sequence) {
  finalizePacket(&packet, sizeof(packet), type, sequence, sizeof(Payload));
}

template <typename Payload>
bool validate(const Packet<Payload>& packet, size_t receivedSize,
              MessageType expectedType) {
  return receivedSize == sizeof(packet) &&
         validatePacket(&packet, sizeof(packet), expectedType, sizeof(Payload));
}

}  // namespace protocol
}  // namespace lil

#pragma once

#include <string.h>
#include "lil_protocol.h"

namespace lil::recording {
// Independent messages: a normal telemetry response never acknowledges storage.
constexpr auto kRecordMessage = static_cast<protocol::MessageType>(20);
constexpr auto kAckMessage = static_cast<protocol::MessageType>(21);
constexpr auto kStatusMessage = static_cast<protocol::MessageType>(22);
constexpr uint32_t kFormat = 1;

#pragma pack(push, 1)
struct Optical {
  protocol::PulseReading pulse{};
  uint16_t estimateAgeMs = UINT16_MAX, warmupMs = 0, processingMs = 0;
  uint8_t count = 0;
  protocol::OpticalFrame frames[protocol::kOpticalBatchSize]{};
};
struct MotionWithFeedback {
  float accelerationG[3], angularRateDps[3];
  float peakAccelerationG, peakAngularRateDps;
  uint16_t sampleCount;
  protocol::MotionFeedback feedback;
};
static_assert(sizeof(MotionWithFeedback) == 38);
// The transmitted / stored length depends on type. Each length ends in CRC32.
// No quantization: retain the driver's float values and all optical ADC bits.
struct Record {
  uint64_t session = 0;
  uint32_t sampleMs = 0;
  protocol::EnvironmentalSensorType type{};
  uint8_t flags = 0;
  uint16_t capabilities = 0, ageMs = UINT16_MAX, windowMs = 0, batteryMv = 0;
  union Values {
    float temperature;
    protocol::MotionReading motion;
    MotionWithFeedback motionFeedback;
    Optical optical;
    uint8_t bytes[106];
    Values() : bytes{} {}
  } values;
};
struct Upload {
  Record record{};
  // UTC anchor is separate from the immutable record. Zero means unknown.
  uint64_t sessionEpochMs = 0;
  uint32_t totalRecords = 0, durationMs = 0;
};
enum class State : uint8_t { Unavailable, Ready, Recording, Pending, Synced, Full, Fault, Preparing };
struct Status {
  uint64_t session = 0;
  uint32_t elapsedMs = 0, pending = 0, capacity = 0, dropped = 0;
  State state = State::Unavailable;
  protocol::EnvironmentalSensorType type{};
};
struct Ack {
  uint64_t session = 0;
  uint32_t sampleMs = 0;
  uint32_t recordCrc = 0;
  uint64_t stationEpochMs = 0;
  uint8_t stored = 0;
};
#pragma pack(pop)
static_assert(sizeof(Record) == 128);
using UploadPacket = protocol::Packet<Upload>;
using AckPacket = protocol::Packet<Ack>;
using StatusPacket = protocol::Packet<Status>;
static_assert(sizeof(UploadPacket) <= protocol::kMaxPacketSize);

constexpr size_t size(protocol::EnvironmentalSensorType type) {
  return type == protocol::EnvironmentalSensorType::kLsm6dsox ? 64 :
      type == protocol::EnvironmentalSensorType::kTmp117 ? 32 :
      type == protocol::EnvironmentalSensorType::kMax30102 ? 128 : 0;
}
inline uint32_t checksum(const Record& record) {
  const size_t length = size(record.type);
  uint32_t crc = 0;
  if (length) memcpy(&crc, reinterpret_cast<const uint8_t*>(&record) + length - 4, 4);
  return crc;
}
inline bool valid(const Record& record) {
  const size_t length = size(record.type);
  return length && record.session && checksum(record) ==
      protocol::crc32(reinterpret_cast<const uint8_t*>(&record), length - 4) &&
      (record.type != protocol::EnvironmentalSensorType::kMax30102 ||
       record.values.optical.count <= protocol::kOpticalBatchSize);
}
inline Record encode(uint64_t session, uint32_t sampleMs,
                     const protocol::TelemetryPayload& source) {
  Record record{};
  record.session = session; record.sampleMs = sampleMs;
  record.type = source.sensorType; record.flags = source.live.flags;
  record.capabilities = source.capabilities;
  if (source.flags & protocol::kSensorReadFailed) {
    record.capabilities &= protocol::kBattery | protocol::kPcbV4PowerGates;
    record.flags |= protocol::kLiveGap;
  }
  record.ageMs = source.live.acquisitionAgeMs;
  record.windowMs = source.live.windowMs; record.batteryMv = source.batteryMillivolts;
  if (record.type == protocol::EnvironmentalSensorType::kLsm6dsox) {
    if (source.live.flags & protocol::kMotionFeedbackPresent) {
      // Reuse the three padding bytes and the redundant overflow byte. The
      // overflow indicator moves to a flag; the 64-byte storage budget is unchanged.
      auto& stored = record.values.motionFeedback;
      memcpy(stored.accelerationG, source.motion.accelerationG, sizeof(stored.accelerationG));
      memcpy(stored.angularRateDps, source.motion.angularRateDps, sizeof(stored.angularRateDps));
      stored.peakAccelerationG = source.motion.peakAccelerationG;
      stored.peakAngularRateDps = source.motion.peakAngularRateDps;
      stored.sampleCount = source.motion.sampleCount;
      record.values.motionFeedback.feedback = source.motionFeedback;
      if (source.motion.fifoOverrun) record.flags |= protocol::kMotionStoredOverrun;
    } else record.values.motion = source.motion;
  }
  else if (record.type == protocol::EnvironmentalSensorType::kTmp117)
    record.values.temperature = source.temperatureC;
  else if (record.type == protocol::EnvironmentalSensorType::kMax30102) {
    auto& optical = record.values.optical;
    optical.pulse = source.pulse; optical.estimateAgeMs = source.live.estimateAgeMs;
    optical.warmupMs = source.live.warmupMs; optical.processingMs = source.live.processingMs;
    optical.count = source.live.count;
    memcpy(optical.frames, source.live.optical, sizeof(optical.frames));
  }
  const size_t length = size(record.type);
  if (length) {
    const uint32_t crc = protocol::crc32(reinterpret_cast<const uint8_t*>(&record), length - 4);
    memcpy(reinterpret_cast<uint8_t*>(&record) + length - 4, &crc, 4);
  }
  return record;
}
inline protocol::TelemetryPayload decode(const Record& record) {
  protocol::TelemetryPayload output{};
  output.sensorType = record.type; output.capabilities = record.capabilities;
  output.batteryMillivolts = record.batteryMv; output.live.flags = record.flags;
  output.live.acquisitionAgeMs = record.ageMs; output.live.windowMs = record.windowMs;
  if (record.type == protocol::EnvironmentalSensorType::kLsm6dsox) {
    if (record.flags & protocol::kMotionFeedbackPresent) {
      const auto& stored = record.values.motionFeedback;
      memcpy(output.motion.accelerationG, stored.accelerationG, sizeof(stored.accelerationG));
      memcpy(output.motion.angularRateDps, stored.angularRateDps, sizeof(stored.angularRateDps));
      output.motion.peakAccelerationG = stored.peakAccelerationG;
      output.motion.peakAngularRateDps = stored.peakAngularRateDps;
      output.motion.sampleCount = stored.sampleCount;
      output.motion.fifoOverrun = bool(record.flags & protocol::kMotionStoredOverrun);
      output.motionFeedback = record.values.motionFeedback.feedback;
    } else output.motion = record.values.motion;
    output.live.samplePeriodUs = 1000000UL / timing::kImuHz;
  } else if (record.type == protocol::EnvironmentalSensorType::kTmp117)
    output.temperatureC = record.values.temperature;
  else if (record.type == protocol::EnvironmentalSensorType::kMax30102) {
    const auto& optical = record.values.optical;
    output.pulse = optical.pulse; output.live.estimateAgeMs = optical.estimateAgeMs;
    output.live.warmupMs = optical.warmupMs; output.live.processingMs = optical.processingMs;
    output.live.count = optical.count;
    output.live.samplePeriodUs = timing::kOpticalSampleMs * 1000;
    memcpy(output.live.optical, optical.frames, sizeof(optical.frames));
  }
  return output;
}
}  // namespace lil::recording

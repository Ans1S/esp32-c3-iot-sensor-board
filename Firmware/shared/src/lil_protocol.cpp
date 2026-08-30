#include "lil_protocol.h"

#include <string.h>

namespace lil {
namespace protocol {

uint32_t crc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

void finalizePacket(void* packet, size_t packetSize, MessageType type,
                    uint32_t sequence, uint16_t payloadSize) {
  if (packet == nullptr || packetSize < sizeof(PacketHeader)) {
    return;
  }

  auto* header = static_cast<PacketHeader*>(packet);
  header->magic = kMagic;
  header->version = kVersion;
  header->type = type;
  header->payloadSize = payloadSize;
  header->sequence = sequence;
  header->crc32 = 0;
  header->crc32 = crc32(static_cast<const uint8_t*>(packet), packetSize);
}

bool validatePacket(const void* packet, size_t packetSize,
                    MessageType expectedType, uint16_t expectedPayloadSize) {
  if (packet == nullptr || packetSize < sizeof(PacketHeader)) {
    return false;
  }

  const auto* header = static_cast<const PacketHeader*>(packet);
  if (header->magic != kMagic || header->version != kVersion ||
      header->type != expectedType || header->payloadSize != expectedPayloadSize ||
      packetSize != sizeof(PacketHeader) + expectedPayloadSize) {
    return false;
  }

  uint8_t copy[kMaxPacketSize];
  if (packetSize > sizeof(copy)) {
    return false;
  }
  memcpy(copy, packet, packetSize);
  auto* copiedHeader = reinterpret_cast<PacketHeader*>(copy);
  const uint32_t receivedCrc = copiedHeader->crc32;
  copiedHeader->crc32 = 0;
  return receivedCrc == crc32(copy, packetSize);
}

}  // namespace protocol
}  // namespace lil

#include "lil_protocol.h"

#include <string.h>
#if defined(ESP_PLATFORM)
#include <esp_rom_crc.h>
#endif

namespace lil {
namespace protocol {

namespace {
uint32_t continueCrc32(uint32_t seed, const uint8_t* data, size_t length) {
#if defined(ESP_PLATFORM)
  return esp_rom_crc32_le(seed, data, length);
#else
  uint32_t crc = ~seed;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
#endif
}
}

uint32_t crc32(const uint8_t* data, size_t length) {
  return continueCrc32(0, data, length);
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

  if (packetSize > kMaxPacketSize) {
    return false;
  }
  PacketHeader copiedHeader;
  memcpy(&copiedHeader, packet, sizeof(copiedHeader));
  const uint32_t receivedCrc = copiedHeader.crc32;
  copiedHeader.crc32 = 0;
  // Chaining the same CRC over header and original payload avoids copying a
  // complete radio frame to the stack. The caller's packet remains immutable.
  const uint32_t headerCrc = continueCrc32(0, reinterpret_cast<const uint8_t*>(&copiedHeader), sizeof(copiedHeader));
  return receivedCrc == continueCrc32(headerCrc,
      static_cast<const uint8_t*>(packet) + sizeof(copiedHeader), packetSize - sizeof(copiedHeader));
}

}  // namespace protocol
}  // namespace lil

#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif
#include "lil_protocol.h"
#include <cassert>
#include <cstdio>
#include <cstring>

int main() {
  using namespace lil::protocol;
  alignas(4) uint8_t packet[kMaxPacketSize + 4]{};
  uint32_t random = 0xC3A531U;
  size_t corrupted = 0, valid = 0;
  for (size_t offset = 0; offset < 4; ++offset) {
    for (size_t length = sizeof(PacketHeader); length <= kMaxPacketSize; ++length) {
      auto* bytes = packet + offset;
      for (size_t i = 0; i < length; ++i) {
        random = random * 1664525U + 1013904223U;
        bytes[i] = uint8_t(random >> 24);
      }
      const auto payload = uint16_t(length - sizeof(PacketHeader));
      finalizePacket(bytes, length, MessageType::kTelemetry, random, payload);
      uint8_t unchanged[kMaxPacketSize];
      memcpy(unchanged, bytes, length);
      assert(validatePacket(bytes, length, MessageType::kTelemetry, payload));
      assert(!memcmp(bytes, unchanged, length));
      ++valid;
      for (size_t i = 0; i < length; ++i) {
        bytes[i] ^= 0x80;
        assert(!validatePacket(bytes, length, MessageType::kTelemetry, payload));
        bytes[i] ^= 0x80;
        ++corrupted;
      }
      assert(!validatePacket(bytes, length, MessageType::kConfigResponse, payload));
      assert(!validatePacket(bytes, length - 1, MessageType::kTelemetry, payload));
    }
  }
  PacketHeader oversized{kMagic, kVersion, MessageType::kTelemetry, uint16_t(kMaxPacketSize), 1, 0};
  assert(!validatePacket(&oversized, sizeof(PacketHeader) + kMaxPacketSize,
                         MessageType::kTelemetry, kMaxPacketSize));
  assert(!validatePacket(nullptr, sizeof(PacketHeader), MessageType::kTelemetry, 0));
  printf("Protocol validation: %zu packets at every length/alignment; %zu single-byte corruptions; immutable input and rejection bounds passed\n", valid, corrupted);
}

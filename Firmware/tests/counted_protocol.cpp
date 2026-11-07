#include "lil_protocol.h"
#include "protocol_counter.h"

// Compile the actual implementation under a private name, then count calls at
// its public boundary. Include the header first so template definitions retain
// the normal public call; no production code or CRC primitive is substituted.
#define finalizePacket countedFinalizeImplementation
#include "../shared/src/lil_protocol.cpp"
#undef finalizePacket

namespace lil::protocol {
void finalizePacket(void* packet, size_t size, MessageType type,
                    uint32_t sequence, uint16_t payloadSize) {
  if (type == MessageType::kTelemetry) {
    ++testTelemetryFinalizations;
    testTelemetryCrcBytes += size;
  }
  countedFinalizeImplementation(packet, size, type, sequence, payloadSize);
}
}

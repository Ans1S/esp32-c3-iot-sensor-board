#pragma once
#include "lil_protocol.h"
#include <string.h>

namespace lil::ota {
constexpr uint32_t kPackageMagic = 0x3141544f; // OTA1
constexpr uint32_t kRelease = 40103;
#define WCH_FIRMWARE_VERSION "4.1.3"
constexpr char kVersion[] = WCH_FIRMWARE_VERSION;
constexpr size_t kBlockSize = 192;
constexpr uint32_t kSlotSize = 0x1e0000;
constexpr uint32_t kSectorSize = 4096;
enum class Op : uint8_t { Hello = 1, Offer, Read, Data, Status, Idle };
enum class State : uint8_t {
  Waiting, Transferring, Verifying, Rebooting, Success, Paused,
  LowBattery, Rejected, RolledBack, Cancelled
};
enum class Failure : uint8_t {
  None, ManifestInvalid, HardwareMismatch, NotNewer, SlotUnavailable,
  LowBattery, FlashBegin, Transport, FlashWrite, DigestMismatch,
  ImageValidation, BootSelection, TrialBootFailed, SessionTimeout
};
#pragma pack(push, 1)
struct Manifest {
  uint32_t magic;
  uint16_t format;
  uint8_t pcb;
  uint8_t protocol;
  uint32_t imageSize;
  uint32_t release;
  char version[24];
  uint8_t sha256[32];
  uint16_t signatureSize;
  uint8_t signature[72];
  uint8_t reserved[14];
};
struct Payload {
  uint32_t nonce;
  uint32_t offset;
  uint16_t size;
  Op op;
  State state;
  uint8_t data[kBlockSize];
};
struct Identity {
  uint32_t release;
  uint8_t pcb;
  uint8_t protocol;
  uint16_t batteryMv;
  char version[24];
  uint8_t build[32];
};
#pragma pack(pop)
using Packet = protocol::Packet<Payload>;
constexpr auto kMessageType = static_cast<protocol::MessageType>(16);
static_assert(sizeof(Manifest) == 160);
static_assert(sizeof(Packet) <= protocol::kMaxPacketSize);
inline bool validManifest(const Manifest& m) {
  return m.magic == kPackageMagic && m.format == 1 &&
    (m.pcb == 3 || m.pcb == 4) && m.protocol == protocol::kVersion &&
    m.imageSize > 256 && m.imageSize <= kSlotSize && m.release > 0 &&
    m.version[0] && memchr(m.version, 0, sizeof(m.version)) &&
    m.signatureSize > 0 && m.signatureSize <= sizeof(m.signature);
}
inline const char* stateName(State s) {
  switch (s) {
    case State::Waiting: return "Waiting for next contact";
    case State::Transferring: return "Transferring";
    case State::Verifying: return "Verifying";
    case State::Rebooting: return "Waiting for boot confirmation";
    case State::Success: return "Successful";
    case State::Paused: return "Interrupted; waiting for contact";
    case State::LowBattery: return "Battery too low";
    case State::Rejected: return "Rejected";
    case State::RolledBack: return "Previous firmware restored";
    case State::Cancelled: return "Cancelled";
  }
  return "Unknown";
}
inline const char* failureName(Failure failure) {
  switch (failure) {
    case Failure::None: return "";
    case Failure::ManifestInvalid: return "The sensor could not validate the signed update manifest.";
    case Failure::HardwareMismatch: return "The package does not match this sensor's PCB revision or protocol.";
    case Failure::NotNewer: return "The package is not newer than the installed sensor software.";
    case Failure::SlotUnavailable: return "The sensor has no compatible inactive OTA slot for this image.";
    case Failure::LowBattery: return "The battery voltage is too low for a reliable flash update.";
    case Failure::FlashBegin: return "The sensor could not prepare its inactive flash slot.";
    case Failure::Transport: return "The ESP-NOW transfer stopped before the next firmware block arrived.";
    case Failure::FlashWrite: return "The sensor could not write a firmware block to flash.";
    case Failure::DigestMismatch: return "The completed firmware does not match its signed SHA-256 digest.";
    case Failure::ImageValidation: return "ESP-IDF rejected the completed application image.";
    case Failure::BootSelection: return "The sensor could not save or select the new boot image.";
    case Failure::SessionTimeout: return "The ten-minute transfer session ended. Saved progress resumes at the next sensor contact.";
    case Failure::TrialBootFailed: return "The new firmware did not complete its trial boot; the previous image was restored.";
  }
  return "The sensor reported an unknown update failure.";
}
bool verifyManifest(const Manifest& manifest);
}

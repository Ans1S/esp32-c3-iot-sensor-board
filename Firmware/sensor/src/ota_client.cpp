#include "ota_client.h"
#include "espnow_transport.h"
#include "hardware_profile.h"
#include <Preferences.h>
#include <esp_app_desc.h>
#include <esp_ota_ops.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <mbedtls/sha256.h>
#include <memory>
#include <new>

// Arduino otherwise accepts the new application before setup() can test it.
extern "C" bool verifyRollbackLater() { return true; }
namespace sensor {
namespace {
#pragma pack(push, 1)
struct FirmwareIdentity { char magic[8]; uint32_t release; uint8_t pcb; uint8_t protocol; char version[24]; };
#pragma pack(pop)
// Volatile reads keep this identity in the application image for the packager.
volatile FirmwareIdentity firmwareIdentity{{'W','C','H','F','W','0','1',0},
    lil::ota::kRelease, PCB_VERSION, lil::protocol::kVersion, WCH_FIRMWARE_VERSION};
constexpr uint32_t kSessionMs = 10UL * 60UL * 1000UL;
constexpr uint32_t kCheckpointBytes = 16384;
bool pendingBoot = false;
esp_timer_handle_t bootTimer = nullptr;
struct Checkpoint {
  uint32_t magic = 0;
  uint32_t offset = 0;
  lil::ota::Manifest manifest{};
  uint8_t prefixHash[32]{};
  uint32_t crc = 0;
};
constexpr uint32_t kCheckpointMagic = 0x3150434f;
bool saveCheckpoint(Preferences& p, Checkpoint& c, mbedtls_sha256_context& hash) {
  mbedtls_sha256_context copy;
  mbedtls_sha256_init(&copy);
  mbedtls_sha256_clone(&copy, &hash);
  const bool ok = mbedtls_sha256_finish(&copy, c.prefixHash) == 0;
  mbedtls_sha256_free(&copy);
  c.crc = lil::protocol::crc32(reinterpret_cast<const uint8_t*>(&c), offsetof(Checkpoint, crc));
  return ok && p.putBytes("checkpoint", &c, sizeof(c)) == sizeof(c);
}
}
void beginOtaBootGuard() {
  esp_ota_img_states_t state;
  pendingBoot = esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) == ESP_OK &&
                state == ESP_OTA_IMG_PENDING_VERIFY;
  if (!pendingBoot) return;
  esp_timer_create_args_t args{};
  args.callback = [](void*) { esp_restart(); }; // Bootloader rolls back an unconfirmed image.
  args.name = "ota-boot-guard";
  if (esp_timer_create(&args, &bootTimer) != ESP_OK ||
      esp_timer_start_once(bootTimer, 60000000) != ESP_OK) esp_restart();
}
void finishOtaBootGuard() {
  // Never enter deep sleep with an untested image. No station contact means
  // that remote manageability has not been proven during this trial boot.
  if (pendingBoot) {
    esp_ota_mark_app_invalid_rollback_and_reboot();
  }
}
bool otaBootPending() { return pendingBoot; }
void checkOta(EspNowTransport& radio, const SensorRuntimeConfig& config,
              AdcReader& adc, uint16_t batteryMv) {
  using namespace lil::ota;
  // Board-specific inclusive OTA limits, in millivolts.
  const uint16_t minimumUpdateMv = kHardware.pcbVersion == 4 ? 2800 : 3350;
  if (!config.provisioned || !config.stationKnown) return;
  Preferences prefs;
  if (!prefs.begin("ota_sensor", false)) return;
  const uint32_t attempted = prefs.getUInt("attempted", 0);
  State bootState = attempted && attempted != kRelease ? State::RolledBack : State::Success;
  if (pendingBoot) {
    if (attempted != kRelease) return;
    if (esp_ota_mark_app_valid_cancel_rollback() != ESP_OK) return;
    pendingBoot = false;
    if (bootTimer) { esp_timer_stop(bootTimer); esp_timer_delete(bootTimer); bootTimer = nullptr; }
  }
  Packet q{}, r{};
  const uint32_t nonce = esp_random();
  uint32_t sequence = esp_random();
  auto exchange = [&]() {
    q.payload.nonce = nonce;
    lil::protocol::finalize(q, kMessageType, ++sequence);
    return radio.otaExchange(config.stationMac, q, r);
  };
  auto status = [&](State state, uint32_t offset, Failure failure = Failure::None) {
    q.payload = {}; q.payload.op = Op::Status; q.payload.state = state; q.payload.offset = offset;
    if (failure != Failure::None) { q.payload.size = 1; q.payload.data[0] = static_cast<uint8_t>(failure); }
    return exchange();
  };
  Identity identity{};
  identity.release = firmwareIdentity.release; identity.pcb = firmwareIdentity.pcb;
  identity.protocol = lil::protocol::kVersion; identity.batteryMv = batteryMv;
  memcpy(identity.version, kVersion, sizeof(kVersion));
  memcpy(identity.build, esp_app_get_description()->app_elf_sha256, sizeof(identity.build));
  q.payload.op = Op::Hello; q.payload.state = bootState; q.payload.size = sizeof(identity);
  memcpy(q.payload.data, &identity, sizeof(identity));
  if (!exchange()) return;
  if (attempted && r.payload.op == Op::Idle) prefs.remove("attempted");
  if (r.payload.op != Op::Offer || r.payload.size != sizeof(Manifest)) return;
  Manifest manifest{}; memcpy(&manifest, r.payload.data, sizeof(manifest));
  const esp_partition_t* target = esp_ota_get_next_update_partition(nullptr);
  if (!verifyManifest(manifest)) { status(State::Rejected, 0, Failure::ManifestInvalid); return; }
  if (manifest.pcb != kHardware.pcbVersion || manifest.protocol != lil::protocol::kVersion) {
    status(State::Rejected, 0, Failure::HardwareMismatch); return;
  }
  if (manifest.release <= kRelease) { status(State::Rejected, 0, Failure::NotNewer); return; }
  if (!target || target->size < manifest.imageSize ||
      ESP.getFlashChipSize() < target->address + target->size) {
    status(State::Rejected, 0, Failure::SlotUnavailable); return;
  }
  if (batteryMv < minimumUpdateMv) { status(State::LowBattery, 0, Failure::LowBattery); return; }
  Checkpoint checkpoint{};
  if (prefs.getBytesLength("checkpoint") == sizeof(checkpoint)) prefs.getBytes("checkpoint", &checkpoint, sizeof(checkpoint));
  bool resume = checkpoint.magic == kCheckpointMagic &&
    checkpoint.crc == lil::protocol::crc32(reinterpret_cast<const uint8_t*>(&checkpoint), offsetof(Checkpoint, crc)) &&
    !memcmp(&checkpoint.manifest, &manifest, sizeof(manifest)) &&
    checkpoint.offset < manifest.imageSize && checkpoint.offset % kCheckpointBytes == 0;
  mbedtls_sha256_context hash;
  mbedtls_sha256_init(&hash); mbedtls_sha256_starts(&hash, 0);
  if (resume) {
    uint8_t buffer[1024];
    for (uint32_t offset = 0; offset < checkpoint.offset;) {
      const size_t size = min(sizeof(buffer), static_cast<size_t>(checkpoint.offset - offset));
      if (esp_partition_read(target, offset, buffer, size) != ESP_OK ||
          mbedtls_sha256_update(&hash, buffer, size) != 0) { resume = false; break; }
      offset += size; delay(1);
    }
    mbedtls_sha256_context copy; mbedtls_sha256_init(&copy); mbedtls_sha256_clone(&copy, &hash);
    uint8_t digest[32];
    resume = resume && mbedtls_sha256_finish(&copy, digest) == 0 && !memcmp(digest, checkpoint.prefixHash, 32);
    mbedtls_sha256_free(&copy);
  }
  // Keep this off the Arduino task stack. Only flushed bytes enter the hash
  // or a persisted checkpoint, so a lost RAM tail is safely retransmitted.
  std::unique_ptr<uint8_t[]> writeBuffer(new (std::nothrow) uint8_t[kSectorSize]);
  if (!writeBuffer) {
    mbedtls_sha256_free(&hash);
    status(State::Paused, checkpoint.offset, Failure::FlashBegin);
    return;
  }
  uint32_t buffered = 0;
  esp_ota_handle_t handle = 0;
  esp_err_t error;
  if (resume) {
    const size_t end = (manifest.imageSize + 4095) & ~4095U;
    error = esp_partition_erase_range(target, checkpoint.offset, end - checkpoint.offset);
    if (error == ESP_OK) error = esp_ota_resume(target, 0, checkpoint.offset, &handle);
  } else {
    checkpoint = {}; checkpoint.magic = kCheckpointMagic; checkpoint.manifest = manifest;
    mbedtls_sha256_starts(&hash, 0);
    error = esp_ota_begin(target, manifest.imageSize, &handle);
  }
  if (error != ESP_OK) { mbedtls_sha256_free(&hash); status(State::Paused, 0, Failure::FlashBegin); return; }
  const uint32_t started = millis();
  uint32_t offset = checkpoint.offset;
  bool ok = true;
  State interruption = State::Paused;
  Failure interruptionFailure = Failure::Transport;
  while (offset < manifest.imageSize && millis() - started < kSessionMs) {
    q.payload = {}; q.payload.op = Op::Read; q.payload.offset = offset;
    if (!exchange() || r.payload.op != Op::Data || r.payload.offset != offset ||
        r.payload.size != min(static_cast<uint32_t>(kBlockSize), manifest.imageSize - offset)) { ok = false; break; }
    // Preserve the 192-byte wire protocol, but write full flash sectors.
    // The final short sector is flushed before verification as well.
    uint32_t consumed = 0;
    while (consumed < r.payload.size) {
      const uint32_t count = min(static_cast<uint32_t>(r.payload.size - consumed),
                                static_cast<uint32_t>(kSectorSize) - buffered);
      memcpy(writeBuffer.get() + buffered, r.payload.data + consumed, count);
      buffered += count; offset += count; consumed += count;
      if (buffered == kSectorSize || offset == manifest.imageSize) {
        if (esp_ota_write(handle, writeBuffer.get(), buffered) != ESP_OK ||
            mbedtls_sha256_update(&hash, writeBuffer.get(), buffered) != 0) {
          interruptionFailure = Failure::FlashWrite; ok = false; break;
        }
        buffered = 0;
        if (offset % kCheckpointBytes == 0 && offset < manifest.imageSize) {
          checkpoint.offset = offset;
          if (!saveCheckpoint(prefs, checkpoint, hash)) {
            interruptionFailure = Failure::FlashWrite; ok = false; break;
          }
        }
        // Radio waits already yield between packets. Yield explicitly only
        // after a flash sector, rather than adding a delay to every packet.
        delay(1);
      }
    }
    if (!ok) break;
    if (offset % 65536 < kBlockSize) {
      const auto battery = adc.readBattery(config.batteryCalibrationFactor);
      if (!battery.valid || battery.millivolts < minimumUpdateMv) {
        interruption = State::LowBattery; ok = false; break;
      }
    }
  }
  if (!ok || offset != manifest.imageSize) {
    if (ok && offset != manifest.imageSize) interruptionFailure = Failure::SessionTimeout;
    if (interruption == State::LowBattery) interruptionFailure = Failure::LowBattery;
    esp_ota_abort(handle); mbedtls_sha256_free(&hash);
    status(interruption, offset, interruptionFailure); return;
  }
  status(State::Verifying, offset);
  uint8_t digest[32];
  ok = mbedtls_sha256_finish(&hash, digest) == 0 && !memcmp(digest, manifest.sha256, 32);
  mbedtls_sha256_free(&hash);
  if (!ok) { esp_ota_abort(handle); prefs.remove("checkpoint"); status(State::Rejected, offset, Failure::DigestMismatch); return; }
  if (esp_ota_end(handle) != ESP_OK) { prefs.remove("checkpoint"); status(State::Rejected, offset, Failure::ImageValidation); return; }
  // Persist the attempted release before selecting it, allowing the old image
  // to report a bootloader rollback after a crash or failed communication test.
  if (prefs.putUInt("attempted", manifest.release) != sizeof(uint32_t) ||
      esp_ota_set_boot_partition(target) != ESP_OK) {
    status(State::Rejected, offset, Failure::BootSelection); return;
  }
  prefs.remove("checkpoint");
  status(State::Rebooting, offset);
  prefs.end();
  ESP.restart();
}
}

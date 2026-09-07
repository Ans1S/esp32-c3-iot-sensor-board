#include "ota_service.h"
#include <mbedtls/sha256.h>

namespace station {
OtaService otaService;
namespace {
constexpr uint32_t kJobMagic = 0x31424f4a;
struct Lock {
  SemaphoreHandle_t mutex;
  bool held;
  explicit Lock(SemaphoreHandle_t m, TickType_t wait = portMAX_DELAY) : mutex(m), held(m && xSemaphoreTake(m, wait) == pdTRUE) {}
  ~Lock() { if (held) xSemaphoreGive(mutex); }
};
bool active(lil::ota::State s) {
  using S = lil::ota::State;
  return s != S::Success && s != S::Cancelled && s != S::Rejected && s != S::RolledBack;
}
String macText(const uint8_t* mac) {
  char text[18];
  snprintf(text, sizeof(text), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return text;
}
}
bool OtaService::begin() {
  mutex_ = xSemaphoreCreateMutex();
  partition_ = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
      static_cast<esp_partition_subtype_t>(0x40), "ota_store");
  if (!mutex_ || !preferences_.begin("ota_station", false)) return false;
  if (preferences_.getBytesLength("nodes") == sizeof(nodes_))
    preferences_.getBytes("nodes", nodes_, sizeof(nodes_));
  if (preferences_.getBytesLength("job") == sizeof(job_))
    preferences_.getBytes("job", &job_, sizeof(job_));
  if (job_.magic != kJobMagic || !lil::ota::verifyManifest(job_.manifest)) job_ = Job{};
  return partition_ != nullptr;
}
bool OtaService::save() {
  return preferences_.putBytes("job", &job_, sizeof(job_)) == sizeof(job_);
}
bool OtaService::startUpload(const uint8_t mac[6], String& error) {
  Lock lock(mutex_);
  if (!partition_) { error = "OTA staging partition missing; install the station base image by USB"; return false; }
  if (uploading_ || (job_.magic == kJobMagic && active(job_.state))) {
    error = "Cancel or finish the existing update first"; return false;
  }
  bool known = false;
  for (const auto& n : nodes_) if (!memcmp(mac, n.mac, 6) && n.identity.release) known = true;
  if (!known) { error = "Node has not reported OTA support; install the sensor base image by USB first"; return false; }
  // Invalidate the old job durably before overwriting its package.
  job_ = Job{};
  if (!save()) { error = "Cannot persist update state"; return false; }
  memcpy(job_.mac, mac, 6);
  uploading_ = true;
  uploadFailed_ = false;
  written_ = erased_ = 0;
  return true;
}
bool OtaService::upload(const uint8_t* bytes, size_t size) {
  Lock lock(mutex_);
  if (!uploading_ || uploadFailed_ || size > sizeof(lil::ota::Manifest) + lil::ota::kSlotSize - written_) {
    uploadFailed_ = true; return false;
  }
  const uint32_t end = written_ + size;
  while (erased_ < end) {
    if (esp_partition_erase_range(partition_, erased_, 4096) != ESP_OK) { uploadFailed_ = true; return false; }
    erased_ += 4096;
  }
  if (esp_partition_write(partition_, written_, bytes, size) != ESP_OK) { uploadFailed_ = true; return false; }
  written_ = end;
  return true;
}
bool OtaService::finishUpload(String& error) {
  Lock lock(mutex_);
  if (!uploading_) { error = "No firmware upload is active."; return false; }
  uploading_ = false;
  if (uploadFailed_) { job_ = Job{}; error = "The package could not be written to station storage."; return false; }
  if (written_ < sizeof(job_.manifest)) { job_ = Job{}; error = "The selected file is incomplete and contains no OTA manifest."; return false; }
  if (esp_partition_read(partition_, 0, &job_.manifest, sizeof(job_.manifest)) != ESP_OK) {
    job_ = Job{}; error = "The station could not read the uploaded package from flash."; return false;
  }
  if (!lil::ota::validManifest(job_.manifest)) {
    job_ = Job{}; error = "This is not a valid W-Charger sensor .ota package. Select the signed .ota file, not firmware.bin or a factory image."; return false;
  }
  if (written_ != sizeof(job_.manifest) + job_.manifest.imageSize) {
    job_ = Job{}; error = "The firmware package is incomplete or contains unexpected trailing data."; return false;
  }
  if (!lil::ota::verifyManifest(job_.manifest)) {
    job_ = Job{}; error = "The package signature is not trusted by this station. Build and sign it with this installation's OTA key."; return false;
  }
  const Node* selected = nullptr;
  for (const auto& n : nodes_) {
    if (!memcmp(n.mac, job_.mac, 6)) { selected = &n; break; }
  }
  if (!selected || !selected->identity.release) {
    job_ = Job{}; error = "The selected sensor has not reported OTA support. Install the 4.0.0 base firmware by USB first."; return false;
  }
  if (selected->identity.pcb != job_.manifest.pcb) {
    error = "Board mismatch: the selected sensor is PCB V" + String(selected->identity.pcb) +
            ", but the package targets PCB V" + String(job_.manifest.pcb) + ".";
    job_ = Job{}; return false;
  }
  if (selected->identity.protocol != job_.manifest.protocol) {
    error = "Protocol mismatch: the package is incompatible with the selected sensor firmware.";
    job_ = Job{}; return false;
  }
  if (selected->identity.release >= job_.manifest.release) {
    error = "No update available: the sensor already runs " + String(selected->identity.version) +
            " and the package contains " + String(job_.manifest.version) +
            ". Build a newer release before uploading it.";
    job_ = Job{}; return false;
  }
  bool ok = true;
  {
    mbedtls_sha256_context hash;
    mbedtls_sha256_init(&hash);
    mbedtls_sha256_starts(&hash, 0);
    uint8_t buffer[1024], digest[32];
    for (uint32_t offset = 0; offset < job_.manifest.imageSize && ok;) {
      size_t count = min(sizeof(buffer), static_cast<size_t>(job_.manifest.imageSize - offset));
      ok = esp_partition_read(partition_, sizeof(job_.manifest) + offset, buffer, count) == ESP_OK &&
           mbedtls_sha256_update(&hash, buffer, count) == 0;
      offset += count;
      delay(1);
    }
    ok = ok && mbedtls_sha256_finish(&hash, digest) == 0 && !memcmp(digest, job_.manifest.sha256, 32);
    mbedtls_sha256_free(&hash);
  }
  if (!ok) { job_ = Job{}; error = "The uploaded firmware data does not match the signed SHA-256 digest. Upload the package again."; return false; }
  job_.magic = kJobMagic;
  job_.state = lil::ota::State::Waiting;
  job_.failure = lil::ota::Failure::None;
  job_.confirmed = 0;
  if (!save()) { job_.magic = 0; error = "Cannot persist update job"; return false; }
  return true;
}
void OtaService::abortUpload() { Lock lock(mutex_); uploading_ = false; uploadFailed_ = true; }
bool OtaService::cancel() {
  Lock lock(mutex_);
  if (uploading_) return false;
  job_.state = lil::ota::State::Cancelled;
  return save();
}
void OtaService::forget(const uint8_t* mac) {
  Lock lock(mutex_);
  for (auto& node : nodes_) if (!mac || !memcmp(node.mac, mac, 6)) node = Node{};
  preferences_.putBytes("nodes", nodes_, sizeof(nodes_));
  if (!mac || !memcmp(job_.mac, mac, 6)) { job_ = Job{}; save(); activeNonce_ = 0; }
}
bool OtaService::respond(const uint8_t mac[6], const lil::ota::Packet& request,
                        lil::ota::Packet& response) {
  using namespace lil::ota;
  Lock lock(mutex_, 0);
  // Upload erases and package verification must not stall normal telemetry.
  // The node retries its pull request when the staging lock is available.
  if (!lock.held) return false;
  const auto& q = request.payload;
  auto& r = response.payload;
  r.nonce = q.nonce;
  r.offset = q.offset;
  r.op = Op::Idle;
  const bool target = job_.magic == kJobMagic && !memcmp(mac, job_.mac, 6);
  if (q.op == Op::Hello && q.size == sizeof(Identity)) {
    Identity identity{};
    memcpy(&identity, q.data, sizeof(identity));
    if (!memchr(identity.version, 0, sizeof(identity.version)) ||
        (identity.pcb != 3 && identity.pcb != 4) || identity.protocol != lil::protocol::kVersion) return false;
    Node* slot = nullptr;
    for (auto& n : nodes_) {
      if (!memcmp(n.mac, mac, 6)) { slot = &n; break; }
      if (!n.identity.release && !slot) slot = &n;
    }
    if (slot) {
      // Battery changes are live diagnostics, not reasons to wear NVS.
      const bool changed = slot->identity.release != identity.release ||
                           memcmp(slot->identity.build, identity.build, 32) != 0;
      memcpy(slot->mac, mac, 6); slot->identity = identity;
      if (changed) preferences_.putBytes("nodes", nodes_, sizeof(nodes_));
    }
    if (target && active(job_.state)) {
      activeNonce_ = q.nonce;
      if (identity.pcb != job_.manifest.pcb) { job_.state = State::Rejected; save(); }
      else if (identity.release == job_.manifest.release && q.state == State::Success) {
        job_.state = State::Success; job_.confirmed = job_.manifest.imageSize; save();
      } else if (q.state == State::RolledBack) {
        job_.state = State::RolledBack;
        job_.failure = Failure::TrialBootFailed;
        save();
      }
      else if (identity.release < job_.manifest.release) {
        r.op = Op::Offer; r.size = sizeof(Manifest);
        memcpy(r.data, &job_.manifest, sizeof(Manifest));
      }
    }
  } else if (target && q.nonce == activeNonce_ && active(job_.state) && !uploading_ && partition_) {
    if (q.op == Op::Read && q.offset < job_.manifest.imageSize) {
      r.size = min(static_cast<uint32_t>(kBlockSize), job_.manifest.imageSize - q.offset);
      if (esp_partition_read(partition_, sizeof(Manifest) + q.offset, r.data, r.size) != ESP_OK) return false;
      r.op = Op::Data;
      job_.confirmed = q.offset; // The next request acknowledges the previous write.
      job_.state = State::Transferring;
    } else if (q.op == Op::Status && (q.state == State::Verifying || q.state == State::Rebooting ||
               q.state == State::Paused || q.state == State::LowBattery || q.state == State::Rejected)) {
      const Failure failure = q.size == 1 ? static_cast<Failure>(q.data[0]) : Failure::None;
      if (job_.state != q.state || job_.failure != failure) {
        job_.state = q.state; job_.failure = failure; save();
      }
      job_.confirmed = min(q.offset, job_.manifest.imageSize);
    }
  }
  if (target) lastRequestMs_ = millis();
  lil::protocol::finalize(response, kMessageType, request.header.sequence);
  return true;
}
void OtaService::json(JsonObject output) {
  Lock lock(mutex_);
  output["available"] = partition_ != nullptr;
  output["uploading"] = uploading_;
  JsonArray nodes = output["nodes"].to<JsonArray>();
  for (const auto& n : nodes_) if (n.identity.release) {
    JsonObject node = nodes.add<JsonObject>();
    node["mac"] = macText(n.mac); node["pcb"] = n.identity.pcb;
    node["version"] = n.identity.version; node["release"] = n.identity.release;
    char build[17]; for (size_t i = 0; i < 8; ++i) snprintf(build + i * 2, 3, "%02x", n.identity.build[i]);
    node["build"] = build;
  }
  if (job_.magic == kJobMagic) {
    output["mac"] = macText(job_.mac); output["targetVersion"] = job_.manifest.version;
    auto state = job_.state;
    if (state == lil::ota::State::Transferring && millis() - lastRequestMs_ > 15000) state = lil::ota::State::Paused;
    output["state"] = lil::ota::stateName(state);
    output["reason"] = lil::ota::failureName(job_.failure);
    output["bytes"] = job_.confirmed; output["total"] = job_.manifest.imageSize;
  }
}
}

#include "ota_service.h"
#include "sensor_timing.h"
#ifdef ESP_PLATFORM
#include <esp_heap_caps.h>
#include <esp_timer.h>
#endif
#include <mbedtls/sha256.h>
#include <new>

namespace station {
OtaService otaService;
namespace {
constexpr uint32_t kJobMagic = 0x31424f4a;
uint64_t monotonicNowMs() {
#ifdef ESP_PLATFORM
  return lil::timing::signedUsToMs(esp_timer_get_time());
#else
  return static_cast<uint64_t>(millis());
#endif
}
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
void formatMacText(const uint8_t* mac, char (&text)[18]) {
  snprintf(text, sizeof(text), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
}
void OtaService::clearReadCache() {
#ifdef ESP_PLATFORM
  if (readCache_) {
    if (readCachePsram_) heap_caps_free(readCache_);
    else delete[] readCache_;
    readCache_ = nullptr;
  }
  readCacheCapacity_ = 0;
  readCachePsram_ = false;
#else
  readCache_.reset();
#endif
  readCacheBytes_ = 0;
}
bool OtaService::begin() {
  clearReadCache();
  mutex_ = xSemaphoreCreateMutex();
  partition_ = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
      static_cast<esp_partition_subtype_t>(0x40), "ota_store");
  if (!mutex_ || !preferences_.begin("ota_station", false)) return false;
  if (preferences_.getBytesLength("nodes") == sizeof(nodes_)) {
    if (preferences_.getBytes("nodes", nodes_, sizeof(nodes_)) != sizeof(nodes_)) {
      for (auto& node : nodes_) node = Node{};
    } else {
      // Persisted bytes may be corrupt even when their length is correct.
      // Never pass an unterminated stored version to JSON or String.
      for (auto& node : nodes_)
        if (node.identity.release && !lil::ota::validIdentity(node.identity)) node = Node{};
    }
  }
  if (preferences_.getBytesLength("job") == sizeof(job_))
    preferences_.getBytes("job", &job_, sizeof(job_));
  if (job_.magic != kJobMagic || !lil::ota::verifyManifest(job_.manifest)) job_ = Job{};
  activeNonce_ = 0;
  hasActiveNonce_ = false;
  lastRequestMs_ = 0;
  return partition_ != nullptr;
}
bool OtaService::save() {
  return preferences_.putBytes("job", &job_, sizeof(job_)) == sizeof(job_);
}
bool OtaService::startUpload(const uint8_t mac[6], String& error) {
  Lock lock(mutex_);
  if (!lock.held) { error = "OTA service unavailable"; return false; }
  if (!partition_) { error = "OTA staging partition missing; install the station base image by USB"; return false; }
  if (uploading_.load(std::memory_order_acquire) || (job_.magic == kJobMagic && active(job_.state))) {
    error = "Cancel or finish the existing update first"; return false;
  }
  bool known = false;
  for (const auto& n : nodes_) if (!memcmp(mac, n.mac, 6) && n.identity.release) known = true;
  if (!known) { error = "Node has not reported OTA support; install the sensor base image by USB first"; return false; }
  // Invalidate the old job durably before overwriting its package.
  const Job previous = job_;
  job_ = Job{};
  if (!save()) { job_ = previous; error = "Cannot persist update state"; return false; }
  clearReadCache();
  activeNonce_ = 0;
  hasActiveNonce_ = false;
  lastRequestMs_ = 0;
  memcpy(job_.mac, mac, 6);
  uploading_.store(true, std::memory_order_release);
  uploadFailed_ = false;
  written_ = erased_ = 0;
  return true;
}
bool OtaService::upload(const uint8_t* bytes, size_t size) {
  Lock lock(mutex_);
  if (!lock.held) return false;
  const size_t capacity = partition_ ? min(size_t(partition_->size), sizeof(lil::ota::Manifest) + size_t(lil::ota::kSlotSize)) : 0;
  if (!uploading_.load(std::memory_order_acquire) || uploadFailed_ || written_ > capacity || size > capacity - written_) {
    uploadFailed_ = true; return false;
  }
  const uint32_t end = written_ + size;
  while (erased_ < end) {
    if (erased_ > partition_->size || partition_->size - erased_ < 4096U ||
        esp_partition_erase_range(partition_, erased_, 4096) != ESP_OK) {
      uploadFailed_ = true; return false;
    }
    erased_ += 4096;
  }
  if (esp_partition_write(partition_, written_, bytes, size) != ESP_OK) { uploadFailed_ = true; return false; }
  written_ = end;
  return true;
}
bool OtaService::finishUpload(String& error) {
  Lock lock(mutex_);
  if (!lock.held) { error = "OTA service unavailable."; return false; }
  if (!uploading_.load(std::memory_order_acquire)) { error = "No firmware upload is active."; return false; }
  // Keep uploading_ asserted throughout flash read and SHA-256 verification so
  // radio pull requests decline immediately without waiting on mutex_.
  struct ClearUpload {
    std::atomic<bool>& flag;
    ~ClearUpload() { flag.store(false, std::memory_order_release); }
  } clearUpload{uploading_};
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
    constexpr size_t kVerifyChunkSize = 8192;
    std::unique_ptr<uint8_t[]> chunkBuffer(new (std::nothrow) uint8_t[kVerifyChunkSize]);
    uint8_t stackBuffer[1024];
    uint8_t* buffer = chunkBuffer ? chunkBuffer.get() : stackBuffer;
    const size_t chunkSize = chunkBuffer ? kVerifyChunkSize : sizeof(stackBuffer);
    uint8_t digest[32];
    uint32_t sinceYield = 0;
    for (uint32_t offset = 0; offset < job_.manifest.imageSize && ok;) {
      const size_t count = min(chunkSize, static_cast<size_t>(job_.manifest.imageSize - offset));
      ok = esp_partition_read(partition_, sizeof(job_.manifest) + offset, buffer, count) == ESP_OK &&
           mbedtls_sha256_update(&hash, buffer, count) == 0;
      offset += count;
      sinceYield += count;
      if (sinceYield >= 65536U) {
        sinceYield = 0;
        delay(1);
      }
    }
    ok = ok && mbedtls_sha256_finish(&hash, digest) == 0 && !memcmp(digest, job_.manifest.sha256, 32);
    mbedtls_sha256_free(&hash);
  }
  if (!ok) { job_ = Job{}; error = "The uploaded firmware data does not match the signed SHA-256 digest. Upload the package again."; return false; }
  job_.magic = kJobMagic;
  job_.state = lil::ota::State::Waiting;
  job_.failure = lil::ota::Failure::None;
  job_.confirmed = 0;
  activeNonce_ = 0;
  hasActiveNonce_ = false;
  lastRequestMs_ = 0;
  if (!save()) { job_.magic = 0; error = "Cannot persist update job"; return false; }
  return true;
}
void OtaService::abortUpload() {
  Lock lock(mutex_);
  if (!lock.held) return;
  uploading_.store(false, std::memory_order_release); uploadFailed_ = true; written_ = erased_ = 0; clearReadCache();
}
bool OtaService::cancel() {
  Lock lock(mutex_);
  if (!lock.held || uploading_.load(std::memory_order_acquire) || job_.magic != kJobMagic) return false;
  const Job previous = job_;
  job_.state = lil::ota::State::Cancelled;
  const bool saved = save();
  if (!saved) job_ = previous;
  else {
    activeNonce_ = 0;
    hasActiveNonce_ = false;
    lastRequestMs_ = 0;
    clearReadCache();
  }
  return saved;
}
void OtaService::forget(const uint8_t* mac) {
  Lock lock(mutex_);
  if (!lock.held || uploading_.load(std::memory_order_acquire)) return;
  for (auto& node : nodes_) if (!mac || !memcmp(node.mac, mac, 6)) node = Node{};
  preferences_.putBytes("nodes", nodes_, sizeof(nodes_));
  if (!mac || !memcmp(job_.mac, mac, 6)) {
    job_ = Job{};
    save();
    activeNonce_ = 0;
    hasActiveNonce_ = false;
    lastRequestMs_ = 0;
    clearReadCache();
  }
}
bool OtaService::readImage(uint32_t offset, uint8_t* output, size_t size) {
  // Called only with the job lock held and after the usual target/nonce checks.
  // A failed allocation or speculative read must retain the direct-read path.
#ifdef ESP_PLATFORM
  if (!readCache_) {
    readCache_ = static_cast<uint8_t*>(
        heap_caps_malloc(kPsramReadCacheSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (readCache_) {
      readCacheCapacity_ = kPsramReadCacheSize;
      readCachePsram_ = true;
    } else {
      readCache_ = new (std::nothrow) uint8_t[kReadCacheSize];
      readCacheCapacity_ = readCache_ ? kReadCacheSize : 0;
      readCachePsram_ = false;
    }
  }
  if (readCache_ && readCacheCapacity_ >= size) {
    bool hit = offset >= readCacheOffset_ && readCacheBytes_ >= size &&
        offset - readCacheOffset_ <= readCacheBytes_ - size;
    if (!hit) {
      readCacheBytes_ = 0;
      const size_t count = min(readCacheCapacity_, size_t(job_.manifest.imageSize - offset));
      if (esp_partition_read(partition_, sizeof(lil::ota::Manifest) + offset,
                             readCache_, count) == ESP_OK) {
        readCacheOffset_ = offset; readCacheBytes_ = count; hit = true;
      }
    }
    if (hit) { memcpy(output, readCache_ + (offset - readCacheOffset_), size); return true; }
  }
#else
  if (!readCache_) readCache_.reset(new (std::nothrow) uint8_t[kReadCacheSize]);
  if (readCache_) {
    bool hit = offset >= readCacheOffset_ && readCacheBytes_ >= size &&
        offset - readCacheOffset_ <= readCacheBytes_ - size;
    if (!hit) {
      readCacheBytes_ = 0;
      const size_t count = min(kReadCacheSize, size_t(job_.manifest.imageSize - offset));
      if (esp_partition_read(partition_, sizeof(lil::ota::Manifest) + offset,
                             readCache_.get(), count) == ESP_OK) {
        readCacheOffset_ = offset; readCacheBytes_ = count; hit = true;
      }
    }
    if (hit) { memcpy(output, readCache_.get() + (offset - readCacheOffset_), size); return true; }
  }
#endif
  return esp_partition_read(partition_, sizeof(lil::ota::Manifest) + offset, output, size) == ESP_OK;
}
bool OtaService::respond(const uint8_t mac[6], const lil::ota::Packet& request,
                        lil::ota::Packet& response) {
  using namespace lil::ota;
  // Upload erases and package verification must not stall normal telemetry.
  // Allow a brief bounded wait when no upload is active so short /api/ota web
  // polls cannot drop in-flight ESP-NOW OTA blocks.
  if (uploading_.load(std::memory_order_acquire)) return false;
#ifdef ESP_PLATFORM
  Lock lock(mutex_, pdMS_TO_TICKS(5));
#else
  Lock lock(mutex_, 0);
#endif
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
    if (!validIdentity(identity) || !identity.release) return false;
    Node* slot = nullptr;
    for (auto& n : nodes_) {
      if (!memcmp(n.mac, mac, 6)) { slot = &n; break; }
      if (!n.identity.release && !slot) slot = &n;
    }
    if (slot) {
      // Battery changes are live diagnostics, not reasons to wear NVS.
      const bool changed = memcmp(slot->mac, mac, 6) != 0 ||
                           slot->identity.release != identity.release ||
                           slot->identity.pcb != identity.pcb ||
                           strncmp(slot->identity.version, identity.version,
                                   sizeof(identity.version)) != 0 ||
                           memcmp(slot->identity.build, identity.build, 32) != 0;
      memcpy(slot->mac, mac, 6); slot->identity = identity;
      if (changed) preferences_.putBytes("nodes", nodes_, sizeof(nodes_));
    }
    if (target && active(job_.state)) {
      activeNonce_ = q.nonce;
      hasActiveNonce_ = true;
      if (identity.pcb != job_.manifest.pcb) { job_.state = State::Rejected; save(); }
      else if (identity.release >= job_.manifest.release && q.state == State::Success) {
        job_.state = State::Success; job_.confirmed = job_.manifest.imageSize; save();
      } else if (q.state == State::RolledBack) {
        job_.state = State::RolledBack;
        job_.failure = Failure::TrialBootFailed;
        save();
      }
      else if (identity.release < job_.manifest.release) {
        r.op = Op::Offer; r.size = sizeof(Manifest);
#ifdef ESP_PLATFORM
        r.offset = kOfferCompactRead | kOfferLargeBlock;
#else
        r.offset = kOfferCompactRead;
#endif
        memcpy(r.data, &job_.manifest, sizeof(Manifest));
      }
    }
  } else if (target && hasActiveNonce_ && q.nonce == activeNonce_ && active(job_.state) && !uploading_.load(std::memory_order_acquire) && partition_) {
    if (q.op == Op::Read && q.offset < job_.manifest.imageSize) {
      r.size = min(static_cast<uint32_t>(kBlockSize), job_.manifest.imageSize - q.offset);
      if (!readImage(q.offset, r.data, r.size)) return false;
      r.op = Op::Data;
      job_.confirmed = q.offset; // The next request confirms receipt of the previous block.
      job_.state = State::Transferring;
    } else if (q.op == Op::Status && (q.state == State::Verifying || q.state == State::Rebooting ||
               q.state == State::Paused || q.state == State::LowBattery || q.state == State::Rejected)) {
      const Failure failure = q.size == 1 ? static_cast<Failure>(q.data[0]) : Failure::None;
      const uint32_t confirmed = min(q.offset, job_.manifest.imageSize);
      if (job_.state != q.state || job_.failure != failure || job_.confirmed != confirmed) {
        job_.state = q.state; job_.failure = failure; job_.confirmed = confirmed;
        save();
      }
      clearReadCache();
    }
  }
  if (target) {
    if (!active(job_.state)) clearReadCache();
    lastRequestMs_ = monotonicNowMs();
  }
  lil::protocol::finalize(response, kMessageType, request.header.sequence);
  return true;
}

size_t OtaService::respondWire(const uint8_t mac[6],
                               const lil::ota::Packet& request,
                               uint8_t* wireOut, size_t wireCap) {
  using namespace lil::ota;
  if (request.payload.op == Op::Read &&
      request.payload.size == kLargeBlockSize &&
      wireCap >= sizeof(LargePacket)) {
    bool fallthroughToIdle = false;
    {
      if (uploading_.load(std::memory_order_acquire)) return 0;
#ifdef ESP_PLATFORM
      Lock lock(mutex_, pdMS_TO_TICKS(5));
#else
      Lock lock(mutex_, 0);
#endif
      if (!lock.held) return 0;
      const auto& q = request.payload;
      const bool target = job_.magic == kJobMagic && !memcmp(mac, job_.mac, 6);
      if (!target || !hasActiveNonce_ || q.nonce != activeNonce_ || !active(job_.state) ||
          uploading_.load(std::memory_order_acquire) || !partition_ || q.offset >= job_.manifest.imageSize) {
        fallthroughToIdle = true;
      } else {
        auto* large = reinterpret_cast<LargePacket*>(wireOut);
        memset(large, 0, sizeof(LargePacket));
        large->payload.nonce = q.nonce;
        large->payload.offset = q.offset;
        large->payload.op = Op::Data;
        large->payload.size = static_cast<uint16_t>(
            min(static_cast<uint32_t>(kLargeBlockSize),
                job_.manifest.imageSize - q.offset));
        if (!readImage(q.offset, large->payload.data, large->payload.size)) {
          return 0;
        }
        job_.confirmed = q.offset;
        job_.state = State::Transferring;
        lastRequestMs_ = monotonicNowMs();
        const size_t payloadLen = offsetof(LargePayload, data) + large->payload.size;
        const size_t wireLen = sizeof(lil::protocol::PacketHeader) + payloadLen;
        lil::protocol::finalizePacket(large, wireLen, kMessageType,
                                      request.header.sequence, payloadLen);
        return wireLen;
      }
    }
    if (!fallthroughToIdle) return 0;
  }
  if (wireCap < sizeof(Packet)) return 0;
  Packet response{};
  if (!respond(mac, request, response)) return 0;
  memcpy(wireOut, &response, sizeof(Packet));
  return sizeof(Packet);
}
void OtaService::json(JsonObject output) {
  Lock lock(mutex_);
  if (!lock.held) {
    output["available"] = false;
    output["uploading"] = false;
    return;
  }
  output["available"] = partition_ != nullptr;
  output["uploading"] = uploading_.load(std::memory_order_acquire);
  JsonArray nodes = output["nodes"].to<JsonArray>();
  char macBuf[18];
  for (const auto& n : nodes_) if (n.identity.release) {
    JsonObject node = nodes.add<JsonObject>();
    formatMacText(n.mac, macBuf);
    node["mac"] = macBuf; node["pcb"] = n.identity.pcb;
    node["version"] = n.identity.version; node["release"] = uint32_t(n.identity.release);
    char build[17]; for (size_t i = 0; i < 8; ++i) snprintf(build + i * 2, 3, "%02x", n.identity.build[i]);
    node["build"] = build;
  }
  if (job_.magic == kJobMagic) {
    formatMacText(job_.mac, macBuf);
    output["mac"] = macBuf; output["targetVersion"] = job_.manifest.version;
    auto state = job_.state;
    const uint64_t nowMs = monotonicNowMs();
    if (state == lil::ota::State::Transferring &&
        nowMs >= lastRequestMs_ && nowMs - lastRequestMs_ > 15000ULL) {
      state = lil::ota::State::Paused;
    }
    output["state"] = lil::ota::stateName(state);
    output["reason"] = lil::ota::failureName(job_.failure);
    output["bytes"] = job_.confirmed; output["total"] = uint32_t(job_.manifest.imageSize);
  }
}
}

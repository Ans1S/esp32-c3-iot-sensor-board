#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif
#include "ota_service.h"
#include "protocol_counter.h"
#include <mbedtls/sha256.h>
#include <cassert>
#include <cstdlib>
#include <new>
#include <vector>

bool failCacheAllocation = false, signatureAccepted = true;
size_t cacheAllocations = 0, signatureChecks = 0;
void* operator new[](size_t size, const std::nothrow_t&) noexcept {
  if (size == 8 * lil::ota::kBlockSize) {
    ++cacheAllocations;
    if (failCacheAllocation) return nullptr;
  }
  return std::malloc(size);
}
void operator delete[](void* pointer) noexcept { std::free(pointer); }
void testRadioTick() { ++testMillis; }
bool lil::ota::verifyManifest(const Manifest& manifest) {
  ++signatureChecks; return signatureAccepted && validManifest(manifest);
}
namespace {
const uint8_t mac[6] = {2, 3, 4, 5, 6, 7};
using namespace lil::ota;
Packet hello(uint32_t nonce) {
  Packet request{}; request.header.sequence = 10;
  request.payload.nonce = nonce; request.payload.op = Op::Hello;
  Identity identity{}; identity.release = kRelease; identity.pcb = 4;
  identity.protocol = lil::protocol::kVersion; strcpy(identity.version, kVersion);
  request.payload.size = sizeof(identity); memcpy(request.payload.data, &identity, sizeof(identity));
  return request;
}
Packet send(station::OtaService& service, Packet request, bool success = true) {
  lil::protocol::finalize(request, kMessageType, request.header.sequence);
  Packet response{};
  assert(service.respond(mac, request, response) == success);
  if (success) assert(lil::protocol::validate(response, sizeof(response), kMessageType));
  return response;
}
void stage(station::OtaService& service, const std::vector<uint8_t>& image, uint32_t release,
           bool expected = true, bool damagePayload = false) {
  Manifest manifest{}; manifest.magic = kPackageMagic; manifest.format = 1;
  manifest.pcb = 4; manifest.protocol = lil::protocol::kVersion;
  manifest.imageSize = image.size(); manifest.release = release;
  manifest.signatureSize = 1; strcpy(manifest.version, "transfer-test");
  mbedtls_sha256_context hash;
  mbedtls_sha256_update(&hash, image.data(), image.size()); mbedtls_sha256_finish(&hash, manifest.sha256);
  std::vector<uint8_t> package(sizeof(manifest) + image.size());
  memcpy(package.data(), &manifest, sizeof(manifest)); memcpy(package.data() + sizeof(manifest), image.data(), image.size());
  if (damagePayload) package.back() ^= 1;
  String error;
  assert(service.startUpload(mac, error));
  for (size_t offset = 0; offset < package.size(); offset += 700)
    assert(service.upload(package.data() + offset, std::min(size_t(700), package.size() - offset)));
  const auto checked = signatureChecks;
  assert(service.finishUpload(error) == expected && signatureChecks > checked);
}
void checkData(const Packet& response, const std::vector<uint8_t>& image, size_t offset) {
  const size_t size = std::min(size_t(kBlockSize), image.size() - offset);
  assert(response.payload.op == Op::Data && response.payload.offset == offset && response.payload.size == size);
  assert(!memcmp(response.payload.data, image.data() + offset, size));
}
}
int main() {
  using namespace lil::ota;
  stationOtaPreferences.clear();
  station::OtaService service;
  assert(service.begin()); send(service, hello(91));
  std::vector<uint8_t> image(70000);
  for (size_t i = 0; i < image.size(); ++i) image[i] = uint8_t(i * 17 + 3);
  stage(service, image, kRelease + 1);
  assert(send(service, hello(91)).payload.op == Op::Offer);
  stagingReads.clear(); cacheAllocations = 0;
  Packet request{}; request.payload.nonce = 91; request.payload.op = Op::Read;
  size_t packets = 0;
  for (size_t offset = 0; offset < image.size(); offset += kBlockSize) {
    request.payload.offset = offset; ++request.header.sequence;
    checkData(send(service, request), image, offset); ++packets;
  }
  size_t bytes = 0; for (const auto& read : stagingReads) bytes += read.size;
  assert(bytes == image.size());
  assert(stagingReads.size() == (FLOW_OPTIMIZED ? (packets + 7) / 8 : packets));
  assert(cacheAllocations == (FLOW_OPTIMIZED ? 1 : 0));
  printf("Station OTA 70000-byte image: %zu packets, %zu flash reads, %zu flash bytes, %zu cache allocations\n",
         packets, stagingReads.size(), bytes, cacheAllocations);
  const auto reads = stagingReads.size();
  checkData(send(service, request), image, request.payload.offset);
  assert(stagingReads.size() - reads == (FLOW_OPTIMIZED ? 0 : 1));
  request.payload.offset = 1; checkData(send(service, request), image, 1);
  request.payload.offset = 16384; checkData(send(service, request), image, 16384);
  request.payload.offset = 4000; failLargeStagingReadOnce = true;
  checkData(send(service, request), image, 4000); failLargeStagingReadOnce = false;
  request.payload.offset = 5000; failAllStagingReads = true;
  // Force the next request outside either cache to exercise both read errors.
  request.payload.offset = 30000;
  send(service, request, false); failAllStagingReads = false;
  checkData(send(service, request), image, 30000);
  Packet status{}; status.payload.nonce = 91; status.payload.op = Op::Status; status.payload.state = State::Paused;
  send(service, status);
  failCacheAllocation = true; stagingReads.clear();
  for (size_t offset : {5000U, 5192U, 5384U}) {
    request.payload.offset = offset; checkData(send(service, request), image, offset);
  }
  assert(stagingReads.size() == 3); failCacheAllocation = false;
  bool declinedBusy = false;
  stagingReadHook = [&] { Packet response{}; declinedBusy = !service.respond(mac, request, response); };
  request.payload.offset = 60000; checkData(send(service, request), image, 60000);
  assert(declinedBusy);
  failNextOtaPreferenceWrite = true;
  assert(!service.cancel()); checkData(send(service, request), image, 60000);
  assert(service.cancel());
  for (auto& byte : image) byte ^= 0x7A;
  stage(service, image, kRelease + 2); assert(send(service, hello(102)).payload.op == Op::Offer);
  request.payload.nonce = 102; request.payload.offset = 0;
  checkData(send(service, request), image, 0);
  request.payload.nonce = 91; assert(send(service, request).payload.op == Op::Idle);
  station::OtaService reboot;
  assert(reboot.begin()); assert(send(reboot, hello(103)).payload.op == Op::Offer);
  request.payload.nonce = 103; request.payload.offset = 16384;
  checkData(send(reboot, request), image, 16384);
  assert(reboot.cancel());
  signatureAccepted = false; stage(reboot, image, kRelease + 3, false);
  signatureAccepted = true; assert(send(reboot, hello(104)).payload.op == Op::Idle);
  stage(reboot, image, kRelease + 3, false, true);
  assert(send(reboot, hello(105)).payload.op == Op::Idle);
  image.resize(kSlotSize);
  for (size_t i = 0; i < image.size(); ++i) image[i] = uint8_t(i * 23 + 11);
  stage(reboot, image, kRelease + 4); assert(send(reboot, hello(106)).payload.op == Op::Offer);
  request.payload.nonce = 106; stagingReads.clear(); cacheAllocations = 0; packets = bytes = 0;
  for (size_t offset = 0; offset < image.size(); offset += kBlockSize) {
    request.payload.offset = offset; ++request.header.sequence;
    checkData(send(reboot, request), image, offset); ++packets;
  }
  for (const auto& read : stagingReads) bytes += read.size;
  assert(bytes == image.size() && stagingReads.size() == (FLOW_OPTIMIZED ? (packets + 7) / 8 : packets));
  printf("Station OTA full slot: %zu bytes, %zu packets, %zu flash reads, %zu flash bytes\n",
         image.size(), packets, stagingReads.size(), bytes);
  const auto endReads = stagingReads.size();
  request.payload.offset = image.size(); assert(send(reboot, request).payload.op == Op::Idle);
  request.payload.offset = UINT32_MAX; assert(send(reboot, request).payload.op == Op::Idle);
  assert(stagingReads.size() == endReads);
  puts("Station OTA: exact blocks/CRCs, tail/retry/resume, job replacement, nonce, busy lock, read/allocation fallbacks, durable cancel and signature/digest guards passed");
}

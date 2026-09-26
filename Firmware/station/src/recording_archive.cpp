#include "recording_archive.h"
#include <LittleFS.h>
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#ifndef RECORDING_MOUNT_PATH
#define RECORDING_MOUNT_PATH "/littlefs"
#endif

namespace station {
RecordingArchive recordingArchive;
namespace {
constexpr uint32_t kMagic = 0x41524331;
constexpr size_t kReserveBytes = 256U * 1024U;
#pragma pack(push, 1)
struct Header {
  uint32_t magic = kMagic;
  uint16_t version = 1, width = 0;
  uint64_t session = 0, epochMs = 0;
  uint32_t expected = 0, durationMs = 0;
  lil::protocol::EnvironmentalSensorType type{};
  uint8_t reserved[27]{};
  uint32_t crc = 0;
};
#pragma pack(pop)
static_assert(sizeof(Header) == 64);
struct Guard {
  SemaphoreHandle_t mutex;
  explicit Guard(SemaphoreHandle_t value) : mutex(value) { xSemaphoreTake(mutex, portMAX_DELAY); }
  ~Guard() { xSemaphoreGive(mutex); }
};
void prefix(const uint8_t mac[6], char* output, size_t capacity) {
  snprintf(output, capacity, "rec-%02x%02x%02x%02x%02x%02x-", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}
void pathFor(const uint8_t mac[6], uint64_t session, char* output, size_t capacity) {
  char pre[24]; prefix(mac, pre, sizeof(pre));
  snprintf(output, capacity, RECORDING_MOUNT_PATH "/%s%016llx.bin", pre, static_cast<unsigned long long>(session));
}
bool headerValid(const Header& h) {
  return h.magic == kMagic && h.version == 1 && h.session && h.width == lil::recording::size(h.type) &&
      h.width && h.crc == lil::protocol::crc32(reinterpret_cast<const uint8_t*>(&h), sizeof(h)-4);
}
bool load(FILE* file, Header& header, uint32_t& count) {
  if (fseek(file, 0, SEEK_SET) || fread(&header, 1, sizeof(header), file) != sizeof(header) || !headerValid(header)) return false;
  if (fseek(file, 0, SEEK_END)) return false;
  const long length = ftell(file);
  if (length < long(sizeof(header))) return false;
  count = (length - sizeof(header)) / header.width;
  return count <= header.expected;
}
void infoFrom(const Header& h, uint32_t count, RecordingInfo& info) {
  info = {h.session,h.epochMs,h.expected,count,h.durationMs,h.type};
}
bool readAt(FILE* file, const Header& h, uint32_t index, lil::recording::Record& record) {
  record = {};
  return !fseek(file, sizeof(Header) + size_t(index)*h.width, SEEK_SET) &&
      fread(&record, 1, h.width, file) == h.width && lil::recording::valid(record) &&
      record.session == h.session && record.type == h.type;
}
}
bool RecordingArchive::begin() { mutex_ = xSemaphoreCreateMutex(); return mutex_ != nullptr; }
size_t RecordingArchive::freeBytes() const {
  const size_t total = LittleFS.totalBytes(), used = LittleFS.usedBytes();
  return total > used + kReserveBytes ? total - used - kReserveBytes : 0;
}
bool RecordingArchive::append(const uint8_t mac[6], const lil::recording::Upload& upload) {
  const auto& record = upload.record;
  if (!mutex_ || !lil::recording::valid(record) || !upload.totalRecords || upload.totalRecords > 200000 ||
      record.sampleMs > upload.durationMs) return false;
  Guard guard(mutex_);
  char path[320]; pathFor(mac, record.session, path, sizeof(path));
  FILE* file = fopen(path, "r+b");
  Header h{}; uint32_t count = 0;
  if (!file) {
    if (freeBytes() < sizeof(Header) + lil::recording::size(record.type) + 4096) return false;
    h.width = lil::recording::size(record.type); h.session = record.session;
    h.epochMs = upload.sessionEpochMs; h.expected = upload.totalRecords;
    h.durationMs = upload.durationMs; h.type = record.type;
    h.crc = lil::protocol::crc32(reinterpret_cast<const uint8_t*>(&h), sizeof(h)-4);
    char temporary[336]; snprintf(temporary, sizeof(temporary), "%s.tmp", path);
    file = fopen(temporary, "w+b");
    if (!file) return false;
    const bool written = fwrite(&h, 1, sizeof(h), file) == sizeof(h) && !fflush(file) && !fsync(fileno(file));
    const bool closed = fclose(file) == 0;
    if (!written || !closed || rename(temporary, path)) return false;
    file = fopen(path, "r+b");
    if (!file || !load(file, h, count)) { if (file) fclose(file); return false; }
  } else if (!load(file, h, count)) { fclose(file); return false; }
  if (h.session != record.session || h.type != record.type || h.expected != upload.totalRecords || h.durationMs != upload.durationMs) {
    fclose(file); return false;
  }
  // A reset may leave a torn, unacknowledged tail. CRC-valid records remain.
  lil::recording::Record last{};
  while (count && !readAt(file, h, count-1, last)) --count;
  if (fseek(file, 0, SEEK_END)) { fclose(file); return false; }
  if (ftell(file) != long(sizeof(Header) + size_t(count)*h.width) &&
      ftruncate(fileno(file), sizeof(Header) + size_t(count)*h.width)) { fclose(file); return false; }
  if (count && record.sampleMs <= last.sampleMs) {
    uint32_t low = 0, high = count;
    while (low < high) {
      const uint32_t middle = low + (high-low)/2;
      lil::recording::Record candidate{};
      if (!readAt(file, h, middle, candidate)) { fclose(file); return false; }
      if (candidate.sampleMs < record.sampleMs) low = middle+1; else high = middle;
    }
    lil::recording::Record existing{};
    const bool duplicate = low < count && readAt(file, h, low, existing) && !memcmp(&existing, &record, h.width);
    fclose(file); return duplicate;
  }
  if (count >= h.expected || freeBytes() < h.width + 4096 || fseek(file, 0, SEEK_END) ||
      fwrite(&record, 1, h.width, file) != h.width || fflush(file) || fsync(fileno(file))) { fclose(file); return false; }
  lil::recording::Record check{};
  const bool ok = readAt(file, h, count, check) && !memcmp(&check, &record, h.width);
  const bool closed = fclose(file) == 0;
  return ok && closed; // Only this result permits the durable radio ACK.
}
std::vector<RecordingInfo> RecordingArchive::list(const uint8_t mac[6]) {
  std::vector<RecordingInfo> result;
  if (!mutex_) return result;
  Guard guard(mutex_); char pre[24]; prefix(mac, pre, sizeof(pre));
  DIR* directory = opendir(RECORDING_MOUNT_PATH); if (!directory) return result;
  while (auto* entry = readdir(directory)) {
    if (strncmp(entry->d_name, pre, strlen(pre)) || strlen(entry->d_name) != strlen(pre)+20 ||
        strcmp(entry->d_name + strlen(entry->d_name)-4, ".bin")) continue;
    char path[320]; snprintf(path, sizeof(path), RECORDING_MOUNT_PATH "/%s", entry->d_name);
    FILE* file = fopen(path, "rb"); if (!file) continue;
    Header header{}; uint32_t count;
    if (load(file, header, count)) { RecordingInfo info; infoFrom(header, count, info); result.push_back(info); }
    fclose(file);
    if (result.size() >= 128) break;
  }
  closedir(directory); return result;
}
size_t RecordingArchive::read(const uint8_t mac[6], uint64_t session, uint32_t& offset,
    lil::recording::Record* output, size_t capacity, RecordingInfo& info, uint32_t fromMs) {
  if (!mutex_ || !output) return 0;
  Guard guard(mutex_); char path[320]; pathFor(mac, session, path, sizeof(path));
  FILE* file = fopen(path, "rb"); if (!file) return 0;
  Header header{}; uint32_t count;
  if (!load(file, header, count)) { fclose(file); return 0; }
  infoFrom(header, count, info);
  if (fromMs != UINT32_MAX) {
    uint32_t low = 0, high = count;
    while (low < high) {
      const uint32_t middle = low + (high-low)/2;
      lil::recording::Record record{};
      if (!readAt(file, header, middle, record)) { fclose(file); return 0; }
      if (record.sampleMs < fromMs) low = middle+1; else high = middle;
    }
    offset = low;
  }
  size_t copied = 0;
  while (offset < count && copied < capacity) {
    if (!readAt(file, header, offset++, output[copied])) break;
    ++copied;
  }
  fclose(file); return copied;
}
bool RecordingArchive::remove(const uint8_t mac[6], uint64_t session) {
  if (!mutex_) return false;
  Guard guard(mutex_); char path[320]; pathFor(mac, session, path, sizeof(path));
  return ::remove(path) == 0;
}
}  // namespace station

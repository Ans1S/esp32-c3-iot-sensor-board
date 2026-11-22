#include "paged_history.h"
#include "sensor_registry.h"
#include "filesystem_guard.h"
#include "telemetry_validation.h"
#include <LittleFS.h>
#include <esp_heap_caps.h>
#ifdef ESP_PLATFORM
#include <esp_littlefs.h>
#endif
#include <sys/stat.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <unistd.h>

#ifndef NORMAL_HISTORY_MOUNT_PATH
#define NORMAL_HISTORY_MOUNT_PATH "/littlefs"
#endif
namespace station {
PagedHistory pagedHistory;
namespace {
constexpr uint32_t kMagic = 0x32545348;
constexpr uint8_t kEnvironment = 1, kTemperature = 2, kMotion = 3, kExtended = 4;
constexpr uint16_t kMaxRowWidth = 64;
constexpr size_t kReadBatchRows = 8;
constexpr size_t kHistoryFilenameLength = sizeof("hist2-000000000000-00000000.bin") - 1;
constexpr size_t kHistoryPathCapacity = sizeof(NORMAL_HISTORY_MOUNT_PATH) + 1 + kHistoryFilenameLength;
#pragma pack(push, 1)
struct Header {
  uint32_t magic = kMagic;
  uint16_t version = 1, width = 0;
  uint8_t mac[6]{};
  uint8_t kind = 0, reservedByte = 0;
  uint32_t generation = 0, interval = 0, crc = 0;
  uint8_t reserved[36]{};
};
struct EnvironmentRow { HistorySample sample{}; uint32_t crc = 0; };
struct ExtendedRow {
  uint32_t timestamp = 0;
  uint16_t capabilities = 0, flags = 0, batteryMillivolts = 0;
  uint8_t sensorType = 0, pcbVersion = 0;
  lil::protocol::ExtendedReading extended{};
  uint32_t crc = 0;
};
#pragma pack(pop)
static_assert(sizeof(Header) == 64 && sizeof(EnvironmentRow) == 44 && sizeof(ExtendedRow) == 40);
struct Guard {
  SemaphoreHandle_t mutex;
  explicit Guard(SemaphoreHandle_t value) : mutex(value) { xSemaphoreTake(mutex, portMAX_DELAY); }
  ~Guard() { xSemaphoreGive(mutex); }
};
class PageRows {
 public:
  PageRows(FILE* file, uint16_t width, size_t rows)
      : file_(file), remaining_(rows), width_(width) {}

  const uint8_t* next() {
    if (offset_ == available_) {
      if (remaining_ == 0) return nullptr;
      const size_t requested = std::min(remaining_, kReadBatchRows);
      available_ = fread(bytes_, width_, requested, file_);
      offset_ = 0;
      // Consume the complete prefix of a short batch without retrying a torn
      // row. The caller decides whether an early end is a read error.
      remaining_ = available_ == requested ? remaining_ - requested : 0;
      if (available_ == 0) return nullptr;
    }
    return bytes_ + offset_++ * width_;
  }

 private:
  FILE* file_;
  size_t remaining_, offset_ = 0, available_ = 0;
  uint16_t width_;
  uint8_t bytes_[kReadBatchRows * kMaxRowWidth];
};
uint8_t kindFor(lil::protocol::EnvironmentalSensorType type) {
  return type == lil::protocol::EnvironmentalSensorType::kTmp117 ? kTemperature :
    type == lil::protocol::EnvironmentalSensorType::kLsm6dsox ? kMotion :
    lil::protocol::hasExtendedPayload(type) ? kExtended : kEnvironment;
}
uint16_t widthFor(uint8_t kind) {
  return kind == kEnvironment ? 44 : kind == kTemperature ? 32 :
    kind == kMotion ? kMaxRowWidth : kind == kExtended ? sizeof(ExtendedRow) : 0;
}
void pathFor(const uint8_t mac[6], uint32_t generation, char (&path)[kHistoryPathCapacity]) {
  static constexpr char kPrefix[] = NORMAL_HISTORY_MOUNT_PATH "/hist2-";
  static constexpr char kHex[] = "0123456789abcdef";
  char* p = path;
  memcpy(p, kPrefix, sizeof(kPrefix) - 1);
  p += sizeof(kPrefix) - 1;
  for (size_t i = 0; i < 6; ++i) {
    *p++ = kHex[mac[i] >> 4];
    *p++ = kHex[mac[i] & 0x0FU];
  }
  *p++ = '-';
  for (int shift = 28; shift >= 0; shift -= 4) {
    *p++ = kHex[(generation >> shift) & 0x0FUL];
  }
  memcpy(p, ".bin", 5);
}
bool validHeader(const Header& h) {
  return h.magic == kMagic && h.version == 1 && h.generation && h.interval && h.interval <= 86400 &&
    h.width == widthFor(h.kind) && h.width &&
    h.crc == lil::protocol::crc32(reinterpret_cast<const uint8_t*>(&h), offsetof(Header, crc));
}
uint16_t boundedU16(float value, float scale) {
  if (!std::isfinite(value) || value <= 0.0F) return 0;
  const float scaled = value * scale;
  if (scaled >= static_cast<float>(UINT16_MAX)) return UINT16_MAX;
  return static_cast<uint16_t>(lroundf(scaled));
}
uint32_t boundedU32(float value) {
  if (!std::isfinite(value) || value <= 0.0F) return 0;
  // Largest exact IEEE-754 float strictly below 2^32 avoids float-to-uint32 UB.
  if (value >= 4294967040.0F) return UINT32_MAX;
  return static_cast<uint32_t>(std::roundf(value));
}
void encodeEnvironment(EnvironmentRow& row, uint32_t timestamp, const lil::protocol::TelemetryPayload& t) {
  auto& s = row.sample;
  s.timestamp = timestamp; s.capabilities = t.capabilities;
  if (t.flags & lil::protocol::kSensorReadFailed) s.capabilities &= lil::protocol::kBattery;
  if (t.flags & lil::protocol::kBatteryReadFailed) s.capabilities &= ~lil::protocol::kBattery;
  if (t.flags & lil::protocol::kBme680RawFallback) s.capabilities &= ~lil::protocol::kIaq;
  s.temperatureCentiC = std::isfinite(t.temperatureC)
                            ? static_cast<int16_t>(lroundf(std::max(-32768.0F, std::min(32767.0F, t.temperatureC * 100.0F))))
                            : 0;
  s.humidityCentiPercent = boundedU16(t.humidityPercent, 100.0F);
  s.pressureDeciHpa = boundedU16(t.pressureHpa, 10.0F);
  s.iaqDeci = boundedU16(t.iaq, 10.0F);
  s.gasResistanceOhms = boundedU32(t.gasResistanceOhms);
  s.batteryMillivolts = t.batteryMillivolts; s.iaqAccuracy = t.iaqAccuracy;
  s.sensorType = static_cast<uint8_t>(t.sensorType); s.pcbVersion = t.pcbVersion;
  row.crc = lil::protocol::crc32(reinterpret_cast<const uint8_t*>(&s),sizeof(s));
}
void encodeExtended(ExtendedRow& row, uint32_t timestamp, const lil::protocol::TelemetryPayload& t) {
  row = {};
  row.timestamp = timestamp;
  row.capabilities = t.capabilities;
  row.flags = t.flags;
  if (t.flags & lil::protocol::kSensorReadFailed) row.capabilities &= lil::protocol::kBattery;
  if (t.flags & lil::protocol::kBatteryReadFailed) row.capabilities &= ~lil::protocol::kBattery;
  row.batteryMillivolts = t.batteryMillivolts;
  row.sensorType = static_cast<uint8_t>(t.sensorType);
  row.pcbVersion = t.pcbVersion;
  row.extended = t.extended;
  row.crc = lil::protocol::crc32(reinterpret_cast<const uint8_t*>(&row), offsetof(ExtendedRow, crc));
}
#if defined(__GNUC__) && !defined(__clang__)
[[gnu::optimize("O2")]]
#endif
bool decode(const uint8_t* bytes, uint8_t kind, NormalHistoryPoint& point) {
  point = {};
  if (kind == kEnvironment) {
    EnvironmentRow row; memcpy(&row,bytes,sizeof(row));
    if (row.crc != lil::protocol::crc32(bytes,sizeof(HistorySample))) return false;
    const auto& s = row.sample; auto& t = point.telemetry;
    point.timestamp = s.timestamp; t.capabilities = s.capabilities;
    t.temperatureC = s.temperatureCentiC * 0.01F; t.humidityPercent = s.humidityCentiPercent * 0.01F;
    t.pressureHpa = s.pressureDeciHpa * 0.1F; t.iaq = s.iaqDeci * 0.1F;
    t.gasResistanceOhms = s.gasResistanceOhms; t.batteryMillivolts = s.batteryMillivolts;
    t.sensorType = static_cast<lil::protocol::EnvironmentalSensorType>(s.sensorType);
    t.pcbVersion = s.pcbVersion; t.iaqAccuracy = s.iaqAccuracy;
  } else if (kind == kExtended) {
    ExtendedRow row; memcpy(&row, bytes, sizeof(row));
    if (row.crc != lil::protocol::crc32(bytes, offsetof(ExtendedRow, crc))) return false;
    auto& t = point.telemetry;
    point.timestamp = row.timestamp;
    t.capabilities = row.capabilities;
    t.flags = row.flags;
    t.batteryMillivolts = row.batteryMillivolts;
    t.sensorType = static_cast<lil::protocol::EnvironmentalSensorType>(row.sensorType);
    t.pcbVersion = row.pcbVersion;
    t.extended = row.extended;
  } else {
    lil::recording::Record record{}; memcpy(&record,bytes,widthFor(kind));
    if (!lil::recording::valid(record) || kindFor(record.type) != kind) return false;
    point.timestamp = record.sampleMs; point.telemetry = lil::recording::decode(record);
  }
  return point.timestamp >= 1577836800UL && lil::protocol::validTelemetryValues(point.telemetry);
}
void filesystemSpace(size_t& total, size_t& used) {
#ifdef ESP_PLATFORM
  total = used = 0;
  if (esp_littlefs_info("spiffs", &total, &used) != ESP_OK) {
    total = LittleFS.totalBytes();
    used = LittleFS.usedBytes();
  }
#else
  total = LittleFS.totalBytes();
  used = LittleFS.usedBytes();
#endif
}
size_t rawFreeBytes() {
  size_t total = 0, used = 0;
  filesystemSpace(total, used);
  return total > used ? total - used : 0;
}
}

bool PagedHistory::begin(bool mounted) {
  mutex_ = xSemaphoreCreateMutex();
  if (!mutex_ || !initializeFilesystemLock()) return false;
  mounted_ = mounted;
  pageCount_ = 0;
  legacyBytes_ = 0;
  cachedQuota_ = 0;
  budgetPages_ = kMaxPages;
  flushCursor_ = 0;
  generation_ = 0;
  noteFilesystemMutation();
#ifdef ESP_PLATFORM
  cachedFreeBytesValid_ = false;
  clearPageCache();
#endif
  if (!mounted_) return true;
  FilesystemGuard fs;
  DIR* directory=opendir(NORMAL_HISTORY_MOUNT_PATH);
  if (!directory) { mounted_=false; return true; }
#ifdef ESP_PLATFORM
  auto* bootPageBuf = static_cast<uint8_t*>(
      heap_caps_malloc(kPageDataBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
#endif
  while (auto* entry=readdir(directory)) {
    if (strncmp(entry->d_name,"hist2-",6) || strlen(entry->d_name)!=kHistoryFilenameLength ||
        strcmp(entry->d_name+kHistoryFilenameLength-4,".bin")) continue;
    char path[kHistoryPathCapacity];
    snprintf(path, sizeof(path), NORMAL_HISTORY_MOUNT_PATH "/%.*s", int(kHistoryFilenameLength), entry->d_name);
    FILE* file=fopen(path,"rb"); if (!file) continue;
    Header h{}; const bool headerOk=fread(&h,1,sizeof(h),file)==sizeof(h)&&validHeader(h);
    char expected[kHistoryPathCapacity]; if (headerOk) pathFor(h.mac,h.generation,expected);
    if (!headerOk || strcmp(expected,path)) {
      fclose(file);
      ::remove(path);
      noteFilesystemMutation();
      continue;
    }
    generation_ = std::max(generation_, h.generation);
    if (pageCount_ == kMaxPages) { fclose(file); continue; }
    Page page{}; memcpy(page.mac,h.mac,6);page.generation=h.generation;page.interval=h.interval;page.width=h.width;page.kind=h.kind;
    NormalHistoryPoint point{};
#ifdef ESP_PLATFORM
    if (bootPageBuf) {
      const size_t maxBytes = ((kPageDataBytes) / h.width) * size_t(h.width);
      const size_t readBytes = fread(bootPageBuf, 1, maxBytes, file);
      const size_t readRows = readBytes / h.width;
      for (size_t row = 0; row < readRows; ++row) {
        if (!decode(bootPageBuf + row * h.width, h.kind, point) ||
            (page.last && point.timestamp <= page.last)) break;
        if (!page.first) page.first = point.timestamp;
        page.last = point.timestamp;
        ++page.count;
      }
      if (page.count > 0) {
        storePageCache(page, bootPageBuf, size_t(page.count) * page.width);
      }
    } else
#endif
    {
      PageRows rows(file, h.width, (kPageBytes - kHeaderBytes) / h.width);
      while (const auto* row = rows.next()) {
        if (!decode(row,h.kind,point) || (page.last&&point.timestamp<=page.last)) break;
        if (!page.first) page.first=point.timestamp;
        page.last=point.timestamp;++page.count;
      }
    }
    fclose(file);pages_[pageCount_++]=page;delay(1);
  }
#ifdef ESP_PLATFORM
  if (bootPageBuf) heap_caps_free(bootPageBuf);
#endif
  closedir(directory);
  std::sort(pages_.begin(),pages_.begin()+pageCount_,[](const Page& a,const Page& b){return a.generation<b.generation;});
  (void)quotaPages();
  return true;
}
void PagedHistory::resetForFactory() {
  if (!mutex_) return;
  Guard lock(mutex_);
  FilesystemGuard fs;
  for (auto& n : nodes_) {
    if (n.buffer) heap_caps_free(n.buffer);
    n = {};
  }
  for (auto& p : pages_) p = {};
  pageCount_ = 0;
  legacyBytes_ = 0;
  cachedQuota_ = 0;
  budgetPages_ = kMaxPages;
  flushCursor_ = 0;
  generation_ = 0;
  noteFilesystemMutation();
#ifdef ESP_PLATFORM
  cachedFreeBytesValid_ = false;
  clearPageCache();
#endif
}
int PagedHistory::nodeIndex(const uint8_t mac[6]) const {
  for (size_t i=0;i<nodes_.size();++i) if(nodes_[i].enabled&&!memcmp(nodes_[i].mac,mac,6))return i;
  return -1;
}
void PagedHistory::configure(const SensorConfig* configs,size_t count) {
  if (!mutex_ || !configs) return;
  Guard lock(mutex_);
  // Configuration changes do not allocate or resize the persistent history.
  for(size_t i=0;i<std::min(count,nodes_.size());++i){
    auto& n=nodes_[i];const auto& c=configs[i];
    const bool enabled=c.occupied&&c.provisioned&&c.environmentalSensorType!=lil::protocol::EnvironmentalSensorType::kMax30102;
    if(n.enabled&&(!enabled||memcmp(n.mac,c.mac,6))){
      if (mounted_ && n.pending) (void)flushNode(n);
      if(n.buffer)heap_caps_free(n.buffer);
      n={};
    }
    if(enabled){
      const bool restored = !n.enabled;
      n.enabled=true;memcpy(n.mac,c.mac,6);
      // Keep the interval associated with buffered rows until they are flushed.
      if (!n.pending && n.interval != c.sleepSeconds) {
        n.interval = c.sleepSeconds;
        if (!restored && ++n.revision == 0) ++n.revision;
      }
      if (restored) {
        rebuildDurable(n);
        n.lastTimestamp = n.durableNewest;
      }
    }
  }
  size_t active=0;for(const auto& n:nodes_)if(n.enabled)++active;
  cachedQuota_=active && budgetPages_ ? std::max<size_t>(1, budgetPages_/active) : 0;
}
size_t PagedHistory::quotaPages() {
  size_t active=0;for(const auto& n:nodes_)if(n.enabled)++active;
  if(!active)return 0;
  // Each history page is <= 4096 bytes and occupies one LittleFS block.
  const size_t historyBytes = pageCount_ * kPageBytes;
  size_t total = 0, used = 0;
  filesystemSpace(total, used);
  const size_t normalBytes=historyBytes+legacyBytes_;
  const size_t other=used>normalBytes?used-normalBytes:0;
  const size_t reserve=historyWorkspaceReserve();
  const size_t room=(total>reserve && other<total-reserve)?(total-reserve-other):0;
  // Include an allowance for file metadata; admission also checks real free blocks.
  budgetPages_=std::min(kMaxPages,room/(kPageBytes+512));
  cachedQuota_=budgetPages_ ? std::max<size_t>(1, budgetPages_/active) : 0;
  return cachedQuota_;
}
void PagedHistory::setLegacyBytes(size_t bytes){if(mutex_){Guard lock(mutex_);legacyBytes_=bytes;}}
bool PagedHistory::appendLegacy(const uint8_t mac[6],const HistorySample& sample,uint32_t interval){
  EnvironmentRow row{};row.sample=sample;row.crc=lil::protocol::crc32(reinterpret_cast<const uint8_t*>(&sample),sizeof(sample));
  NormalHistoryPoint point{};return decode(reinterpret_cast<const uint8_t*>(&row),kEnvironment,point)&&append(mac,point.timestamp,point.telemetry,interval,true);
}
size_t PagedHistory::pagesFor(const uint8_t mac[6]) const {size_t count=0;for(size_t i=0;i<pageCount_;++i)if(!memcmp(pages_[i].mac,mac,6))++count;return count;}
size_t PagedHistory::largestHistoryVictim(size_t excludeIndex) const {
  static_assert(kMaxPages <= size_t(UINT8_MAX) + 1);
  // Sort indices only. The page array stays in generation order and includes
  // files of retired sensors that no longer have a configured node.
  std::array<uint8_t, kMaxPages> byOwner;
  size_t candidateCount = 0;
  for (size_t i = 0; i < pageCount_; ++i) {
    if (i != excludeIndex) byOwner[candidateCount++] = static_cast<uint8_t>(i);
  }
  if (candidateCount == 0) return pageCount_;
  std::sort(byOwner.begin(), byOwner.begin() + candidateCount, [this](uint8_t a, uint8_t b) {
    return memcmp(pages_[a].mac, pages_[b].mac, 6) < 0;
  });

  size_t chosen = pageCount_;
  size_t largest = 0;
  for (size_t first = 0; first < candidateCount;) {
    const auto* mac = pages_[byOwner[first]].mac;
    uint8_t oldest = byOwner[first];
    size_t end = first + 1;
    for (; end < candidateCount && !memcmp(pages_[byOwner[end]].mac, mac, 6); ++end)
      oldest = std::min(oldest, byOwner[end]);
    const size_t count = end - first;
    if (count > largest || (count == largest && oldest < chosen)) {
      largest = count;
      chosen = oldest;
    }
    first = end;
  }
  return chosen;
}

bool PagedHistory::erasePage(size_t index){
  if(index>=pageCount_)return false;
  char path[kHistoryPathCapacity];pathFor(pages_[index].mac,pages_[index].generation,path);
  if(::remove(path))return false;
#ifdef ESP_PLATFORM
  invalidatePageCache(pages_[index].mac, pages_[index].generation);
#endif
  const int owner = nodeIndex(pages_[index].mac);
  if (owner >= 0) {
    auto& node = nodes_[static_cast<size_t>(owner)];
    node.durableStale = true;
    if (++node.revision == 0) ++node.revision;
  }
  for(size_t i=index+1;i<pageCount_;++i)pages_[i-1]=pages_[i];
  --pageCount_;
  noteFilesystemMutation();
#ifdef ESP_PLATFORM
  cachedFreeBytesValid_ = false;
#endif
  return true;
}
size_t PagedHistory::cachedFreeBytes() {
#ifdef ESP_PLATFORM
  const uint32_t epoch = filesystemSpaceEpoch();
  if (!cachedFreeBytesValid_ || cachedSpaceEpoch_ != epoch) {
    cachedFreeBytes_ = rawFreeBytes();
    cachedSpaceEpoch_ = epoch;
    cachedFreeBytesValid_ = true;
  }
  return cachedFreeBytes_;
#else
  return rawFreeBytes();
#endif
}
bool PagedHistory::makeSpace(const uint8_t mac[6],bool newPage){
  const size_t quota=quotaPages();if(!quota)return false;
  while(newPage&&pagesFor(mac)>=quota){size_t i=0;while(i<pageCount_&&memcmp(pages_[i].mac,mac,6))++i;if(!erasePage(i))return false;}
  while(pageCount_>=kMaxPages || rawFreeBytes()<historyWorkspaceReserve()+2*kPageBytes){
    if (!erasePage(largestHistoryVictim())) return false;
  }
  return true;
}
bool PagedHistory::append(const uint8_t mac[6],uint32_t timestamp,const lil::protocol::TelemetryPayload& telemetry,uint32_t interval,bool batchImport){
  if(!mutex_||timestamp<1577836800UL||!interval||interval>86400||
     telemetry.sensorType==lil::protocol::EnvironmentalSensorType::kMax30102||
     !lil::protocol::validTelemetryValues(telemetry))return false;
  Guard lock(mutex_);const int index=nodeIndex(mac);if(index<0||!mounted_)return false;auto& n=nodes_[index];
  if(timestamp<=n.lastTimestamp)return true;
  const uint8_t kind=kindFor(telemetry.sensorType);const uint16_t width=widthFor(kind);
  if(n.pending&&(n.kind!=kind||n.interval!=interval)&&!flushNode(n)){++n.dropped;return false;}
  n.kind=kind;n.width=width;n.interval=interval;
  if(!n.buffer){
    const size_t bytes=kPageBytes-kHeaderBytes;
    if(heap_caps_get_free_size(MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT)>bytes+1024U*1024U)
      n.buffer=static_cast<uint8_t*>(heap_caps_calloc(bytes,1,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
    if(n.buffer)n.bufferBytes=bytes;
    else if(heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT)>128U*1024U+8*64){
      n.buffer=static_cast<uint8_t*>(heap_caps_calloc(8*64,1,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
      if(n.buffer)n.bufferBytes=8*64;
    }
    if(!n.buffer){n.writeFailed=true;++n.dropped;return false;}
  }
  // Recalculate from the allocation so narrower rows recover the full PSRAM
  // capacity after a type change. Keep internal-RAM writes at eight rows.
  const size_t bufferRows = n.bufferBytes == kPageBytes-kHeaderBytes ? n.bufferBytes/width : 8;
  if(n.pending>=bufferRows&&!flushNode(n)){++n.dropped;return false;}
  uint8_t* output=n.buffer+size_t(n.pending)*width;
  if(kind==kEnvironment){EnvironmentRow row{};encodeEnvironment(row,timestamp,telemetry);memcpy(output,&row,sizeof(row));}
  else if(kind==kExtended){ExtendedRow row{};encodeExtended(row,timestamp,telemetry);memcpy(output,&row,sizeof(row));}
  else {
    auto validTelemetry=telemetry;
    if(telemetry.flags&lil::protocol::kBatteryReadFailed)validTelemetry.capabilities&=~lil::protocol::kBattery;
    const auto record=lil::recording::encode(1,timestamp,validTelemetry);
    memcpy(output,&record,width);
  }
  if(!n.pending)n.startedMs=millis();
  ++n.pending;n.lastTimestamp=timestamp;if(++n.revision==0)++n.revision;
  if(n.pending==bufferRows||(!batchImport&&interval>=60))return flushNode(n);
  return true;
}
bool PagedHistory::flushNode(Node& n){
  if(!n.pending)return true;
  FilesystemGuard fs;if(!fs)return false;
  while(n.pending){
    size_t index=pageCount_;
    for(size_t i=pageCount_;i>0;--i)if(!memcmp(pages_[i-1].mac,n.mac,6)){const auto& p=pages_[i-1];
      if(p.kind==n.kind&&p.interval==n.interval&&p.count<(kPageBytes-kHeaderBytes)/n.width)index=i-1;
      break;
    }
    if(index==pageCount_){
      if(!makeSpace(n.mac,true)||generation_==UINT32_MAX){n.writeFailed=true;return false;}
      Page p{};memcpy(p.mac,n.mac,6);p.width=n.width;p.kind=n.kind;p.interval=n.interval;p.generation=++generation_;
      char path[kHistoryPathCapacity];pathFor(p.mac,p.generation,path);
      Header h{};memcpy(h.mac,p.mac,6);h.width=p.width;h.kind=p.kind;h.interval=p.interval;h.generation=p.generation;
      h.crc=lil::protocol::crc32(reinterpret_cast<const uint8_t*>(&h),offsetof(Header,crc));
#ifdef ESP_PLATFORM
      // Write header + initial batch in a single file open and fsync on flash.
      const size_t count = std::min<size_t>(n.pending, (kPageBytes - kHeaderBytes) / n.width);
      FILE* file = fopen(path, "w+b");
      if (!file) { n.writeFailed = true; return false; }
      bool ok = fwrite(&h, 1, sizeof(h), file) == sizeof(h) &&
                fwrite(n.buffer, n.width, count, file) == count &&
                !fflush(file) && !fsync(fileno(file)) &&
                !fseek(file, kHeaderBytes, SEEK_SET);
      NormalHistoryPoint point{};
      uint32_t firstTimestamp = 0;
      PageRows rows(file, n.width, count);
      for (size_t i = 0; ok && i < count; ++i) {
        const auto* row = rows.next();
        ok = row && !memcmp(row, n.buffer + i * n.width, n.width) && decode(row, n.kind, point);
        if (ok && i == 0) firstTimestamp = point.timestamp;
      }
      const bool closed = fclose(file) == 0;
      if (!ok || !closed) { ::remove(path); n.writeFailed = true; return false; }
      p.first = firstTimestamp; p.last = point.timestamp; p.count = static_cast<uint16_t>(count);
      pages_[pageCount_++] = p;
      storePageCache(p, n.buffer, count * n.width);
      noteFilesystemMutation();
      cachedFreeBytesValid_ = false;
      n.pending -= count;
      if (!n.durableStale) includeDurable(n, pages_[pageCount_ - 1], count);
      memmove(n.buffer, n.buffer + count * n.width, size_t(n.pending) * n.width);
      continue;
#else
      FILE* file=fopen(path,"wb");if(!file){n.writeFailed=true;return false;}
      const bool ok=fwrite(&h,1,sizeof(h),file)==sizeof(h)&&!fflush(file)&&!fsync(fileno(file));
      const bool closed=fclose(file)==0;if(!ok||!closed){::remove(path);n.writeFailed=true;return false;}
      noteFilesystemMutation();
      index=pageCount_;pages_[pageCount_++]=p;
#endif
    }else if(cachedFreeBytes()<historyWorkspaceReserve()+2*kPageBytes){
      const size_t victim = largestHistoryVictim(index);
      if(victim>=pageCount_||!erasePage(victim)){n.writeFailed=true;return false;}
      // The page array changed; select the append page again.
      continue;
    }
    auto& page=pages_[index];const size_t count=std::min<size_t>(n.pending,(kPageBytes-kHeaderBytes)/n.width-page.count);
    const size_t position=kHeaderBytes+size_t(page.count)*n.width;char path[kHistoryPathCapacity];pathFor(page.mac,page.generation,path);
    FILE* file=fopen(path,"r+b");if(!file){n.writeFailed=true;return false;}
    bool truncatedOk = true;
    struct stat st{};
    if (fstat(fileno(file), &st) != 0 || static_cast<size_t>(st.st_size) != position) {
      truncatedOk = !ftruncate(fileno(file), position);
    }
    bool ok=truncatedOk&&!fseek(file,position,SEEK_SET)&&
      fwrite(n.buffer,n.width,count,file)==count&&!fflush(file)&&!fsync(fileno(file));
    NormalHistoryPoint point{};
    uint32_t firstTimestamp = 0;
    // Read back the contiguous batch sequentially after the durable flush.
    // Every row still requires exact bytes and successful CRC/value checks.
    if(ok)ok=!fseek(file,position,SEEK_SET);
    PageRows rows(file, n.width, count);
    for (size_t i = 0; ok && i < count; ++i) {
      const auto* row = rows.next();
      ok = row && !memcmp(row, n.buffer + i*n.width, n.width) && decode(row, n.kind, point);
      if (ok && i == 0) firstTimestamp = point.timestamp;
    }
    const bool closed=fclose(file)==0;
    if(!ok||!closed){
#ifdef ESP_PLATFORM
      cachedFreeBytesValid_ = false;
      invalidatePageCache(page.mac, page.generation);
#endif
      n.writeFailed=true;return false;
    }
    const bool firstRows = page.count == 0;
    if(firstRows)page.first=firstTimestamp;
    page.last=point.timestamp;page.count+=count;n.pending-=count;
    if(firstRows)noteFilesystemMutation();
#ifdef ESP_PLATFORM
    if(firstRows)cachedFreeBytesValid_ = false;
    invalidatePageCache(page.mac, page.generation);
#endif
    if (!n.durableStale) includeDurable(n, page, count);
    memmove(n.buffer,n.buffer+count*n.width,size_t(n.pending)*n.width);
  }
  n.writeFailed=false;return true;
}
bool PagedHistory::flush(const uint8_t mac[6]){if(!mutex_)return false;Guard lock(mutex_);const int i=nodeIndex(mac);return i>=0&&flushNode(nodes_[i]);}
void PagedHistory::flushDue(){
  if(!mutex_)return;
  Guard lock(mutex_);
  for(size_t step=0;step<nodes_.size();++step){const size_t index=(flushCursor_+step)%nodes_.size();auto& n=nodes_[index];
    if(n.enabled&&n.pending&&uint32_t(millis()-n.startedMs)>=kFlushMs){flushNode(n);n.startedMs=millis();flushCursor_=(index+1)%nodes_.size();break;}}
}
void PagedHistory::flushAll(){
  if(!mutex_||!mounted_)return;
  Guard lock(mutex_);
  for(auto& n:nodes_)if(n.enabled&&n.pending)(void)flushNode(n);
}
bool PagedHistory::reserveForArchive(){
  if(!mutex_||!mounted_)return false;
  Guard lock(mutex_);FilesystemGuard fs;noteArchiveWork();
  while(rawFreeBytes()<historyWorkspaceReserve()+2*kPageBytes){if(!pageCount_||!erasePage(largestHistoryVictim()))return false;}
  return true;
}
bool PagedHistory::clear(const uint8_t mac[6]){
  if(!mutex_||!mounted_)return false;
  Guard lock(mutex_);FilesystemGuard fs;
  for(size_t i=0;i<pageCount_;)if(!memcmp(pages_[i].mac,mac,6)){if(!erasePage(i))return false;}else ++i;
  const int index=nodeIndex(mac);if(index>=0){auto& n=nodes_[index];n.pending=0;n.lastTimestamp=0;n.writeFailed=false;if(++n.revision==0)++n.revision;}
  return true;
}
void PagedHistory::includeDurable(Node& node, const Page& page, size_t points) {
  if (!points) return;
  node.durablePoints += points;
  if (!node.durableOldest || page.first < node.durableOldest) node.durableOldest = page.first;
  node.durableNewest = std::max(node.durableNewest, page.last);
}

void PagedHistory::rebuildDurable(Node& node) const {
  node.durablePoints = node.durableOldest = node.durableNewest = 0;
  for (size_t i = 0; i < pageCount_; ++i) {
    const auto& page = pages_[i];
    if (!memcmp(page.mac, node.mac, 6)) includeDurable(node, page, page.count);
  }
  node.durableStale = false;
}

NormalHistoryInfo PagedHistory::infoLocked(Node& n)const{
  if (n.durableStale) rebuildDurable(n);
  NormalHistoryInfo info{};info.available=mounted_;info.revision=n.revision;info.pendingPoints=n.pending;info.intervalSeconds=n.interval;
  info.dropped=n.dropped;info.writeFailed=n.writeFailed;
  info.durablePoints = n.durablePoints;
  info.oldest = n.durableOldest;
  info.newest = n.durableNewest;
  if(n.pending){NormalHistoryPoint p{};if(decode(n.buffer,n.kind,p)){if(!info.oldest||p.timestamp<info.oldest)info.oldest=p.timestamp;}info.newest=n.lastTimestamp;}
  info.points=info.durablePoints+n.pending;
  info.capacityPoints=cachedQuota_*((kPageBytes-kHeaderBytes)/(n.width?n.width:44));return info;
}
NormalHistoryInfo PagedHistory::info(const uint8_t mac[6]){if(!mutex_)return {};Guard lock(mutex_);const int i=nodeIndex(mac);return i<0?NormalHistoryInfo{}:infoLocked(nodes_[i]);}
#ifdef ESP_PLATFORM
bool PagedHistory::ensurePageCache() {
  if (pageCacheMeta_ && pageCacheData_) return true;
  constexpr size_t kDataBytes = kPageCacheSlots * kPageDataBytes;
  constexpr size_t kMetaBytes = kPageCacheSlots * sizeof(PageCacheSlot);
  if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) <=
      kDataBytes + kMetaBytes + 2U * 1024U * 1024U) {
    return false;
  }
  if (!pageCacheMeta_) {
    pageCacheMeta_ = static_cast<PageCacheSlot*>(
        heap_caps_calloc(kPageCacheSlots, sizeof(PageCacheSlot),
                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  }
  if (!pageCacheData_) {
    pageCacheData_ = static_cast<uint8_t*>(
        heap_caps_malloc(kDataBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  }
  if (!pageCacheMeta_ || !pageCacheData_) {
    if (pageCacheMeta_) { heap_caps_free(pageCacheMeta_); pageCacheMeta_ = nullptr; }
    if (pageCacheData_) { heap_caps_free(pageCacheData_); pageCacheData_ = nullptr; }
    return false;
  }
  return true;
}

void PagedHistory::clearPageCache() {
  if (!pageCacheMeta_) return;
  for (size_t i = 0; i < kPageCacheSlots; ++i) pageCacheMeta_[i].valid = false;
}

void PagedHistory::invalidatePageCache(const uint8_t mac[6], uint32_t generation) {
  if (!pageCacheMeta_) return;
  auto& slot = pageCacheMeta_[generation & (kPageCacheSlots - 1U)];
  if (slot.valid && slot.generation == generation && memcmp(slot.mac, mac, 6) == 0) {
    slot.valid = false;
  }
}

const uint8_t* PagedHistory::lookupPageCache(const Page& page) const {
  if (!pageCacheMeta_ || !pageCacheData_ || page.count == 0) return nullptr;
  const size_t idx = page.generation & (kPageCacheSlots - 1U);
  const auto& slot = pageCacheMeta_[idx];
  if (!slot.valid || slot.generation != page.generation ||
      slot.count != page.count || slot.width != page.width ||
      slot.kind != page.kind || memcmp(slot.mac, page.mac, 6) != 0) {
    return nullptr;
  }
  return pageCacheData_ + idx * kPageDataBytes;
}

void PagedHistory::storePageCache(const Page& page, const uint8_t* bytes, size_t totalBytes) {
  if (!bytes || page.count == 0 || totalBytes == 0 || totalBytes > kPageDataBytes) return;
  if (!ensurePageCache()) return;
  const size_t idx = page.generation & (kPageCacheSlots - 1U);
  auto& slot = pageCacheMeta_[idx];
  slot.valid = false;
  memcpy(pageCacheData_ + idx * kPageDataBytes, bytes, totalBytes);
  memcpy(slot.mac, page.mac, 6);
  slot.kind = page.kind;
  slot.generation = page.generation;
  slot.count = page.count;
  slot.width = page.width;
  slot.valid = true;
}
#endif

size_t PagedHistory::read(const uint8_t mac[6],uint32_t after,uint32_t until,uint32_t oldest,NormalHistoryPoint* output,size_t capacity,bool& stale,bool& error){
  stale=error=false;if(!mutex_||!output)return 0;Guard lock(mutex_);FilesystemGuard fs;
  const int index=nodeIndex(mac);if(index<0)return 0;auto& n=nodes_[index];const auto info=infoLocked(n);
  if(oldest&&(info.oldest==0||info.oldest>oldest)){stale=true;return 0;}if(!until)until=info.newest;
  size_t copied=0;NormalHistoryPoint p{};
#ifdef ESP_PLATFORM
  // Serve verified 4 KiB pages from bounded PSRAM cache when warm; on a miss,
  // read the full page in one fread(), verify every row CRC, then cache it.
  uint8_t* pageBuf = nullptr;
  for(size_t i=0;i<pageCount_&&copied<capacity;++i){const auto& page=pages_[i];if(memcmp(page.mac,mac,6)||page.last<=after||page.first>until)continue;
    const size_t totalBytes = size_t(page.count) * page.width;
    if (const uint8_t* cached = lookupPageCache(page)) {
      if (totalBytes <= kPageDataBytes) {
        for (size_t row = 0; row < page.count && copied < capacity; ++row) {
          if (!decode(cached + row * page.width, page.kind, p)) {
            invalidatePageCache(page.mac, page.generation);
            error = true;
            break;
          }
          if (p.timestamp > after && p.timestamp <= until) output[copied++] = p;
          if (p.timestamp >= until) break;
        }
        if (error) break;
        continue;
      }
    }
    if (!pageBuf) {
      pageBuf = static_cast<uint8_t*>(
          heap_caps_malloc(kPageDataBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    char path[kHistoryPathCapacity];pathFor(page.mac,page.generation,path);FILE* file=fopen(path,"rb");if(!file){error=true;break;}
    if(fseek(file,kHeaderBytes,SEEK_SET)){fclose(file);error=true;break;}
    if (pageBuf && totalBytes <= kPageDataBytes) {
      if (fread(pageBuf, 1, totalBytes, file) != totalBytes) { error = true; fclose(file); break; }
      fclose(file);
      bool allValid = true;
      size_t verifiedRows = 0;
      for (size_t row = 0; row < page.count && copied < capacity; ++row) {
        if (!decode(pageBuf + row * page.width, page.kind, p)) { error = true; allValid = false; break; }
        ++verifiedRows;
        if (p.timestamp > after && p.timestamp <= until) output[copied++] = p;
        if (p.timestamp >= until) break;
      }
      if (error) break;
      if (allValid && verifiedRows == page.count) storePageCache(page, pageBuf, totalBytes);
    } else {
      PageRows rows(file, page.width, page.count);
      for(size_t row=0;row<page.count&&copied<capacity;++row){
        const auto* bytes = rows.next();
        if (!bytes || !decode(bytes, page.kind, p)) { error=true; break; }
        if(p.timestamp>after&&p.timestamp<=until)output[copied++]=p;
        if(p.timestamp>=until)break;
      }
      fclose(file);if(error)break;
    }
  }
  if (pageBuf) heap_caps_free(pageBuf);
#else
  for(size_t i=0;i<pageCount_&&copied<capacity;++i){const auto& page=pages_[i];if(memcmp(page.mac,mac,6)||page.last<=after||page.first>until)continue;
    char path[kHistoryPathCapacity];pathFor(page.mac,page.generation,path);FILE* file=fopen(path,"rb");if(!file){error=true;break;}
    if(fseek(file,kHeaderBytes,SEEK_SET)){fclose(file);error=true;break;}
    PageRows rows(file, page.width, page.count);
    for(size_t row=0;row<page.count&&copied<capacity;++row){
      const auto* bytes = rows.next();
      if (!bytes || !decode(bytes, page.kind, p)) { error=true; break; }
      if(p.timestamp>after&&p.timestamp<=until)output[copied++]=p;
      // Committed rows within a page are strictly chronological.
      if(p.timestamp>=until)break;
    }fclose(file);if(error)break;
  }
#endif
  for(size_t i=0;!error&&i<n.pending&&copied<capacity;++i){
    if(!decode(n.buffer+i*n.width,n.kind,p)){error=true;break;}
    if(p.timestamp>after&&p.timestamp<=until)output[copied++]=p;
    if(p.timestamp>=until)break;
  }
  return copied;
}
}

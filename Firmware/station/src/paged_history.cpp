#include "paged_history.h"
#include "sensor_registry.h"
#include "filesystem_guard.h"
#include "telemetry_validation.h"
#include <LittleFS.h>
#include <esp_heap_caps.h>
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
constexpr uint8_t kEnvironment = 1, kTemperature = 2, kMotion = 3;
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
#pragma pack(pop)
static_assert(sizeof(Header) == 64 && sizeof(EnvironmentRow) == 44);
struct Guard {
  SemaphoreHandle_t mutex;
  explicit Guard(SemaphoreHandle_t value) : mutex(value) { xSemaphoreTake(mutex, portMAX_DELAY); }
  ~Guard() { xSemaphoreGive(mutex); }
};
uint8_t kindFor(lil::protocol::EnvironmentalSensorType type) {
  return type == lil::protocol::EnvironmentalSensorType::kTmp117 ? kTemperature :
    type == lil::protocol::EnvironmentalSensorType::kLsm6dsox ? kMotion : kEnvironment;
}
uint16_t widthFor(uint8_t kind) { return kind == kEnvironment ? 44 : kind == kTemperature ? 32 : kind == kMotion ? 64 : 0; }
void pathFor(const uint8_t mac[6], uint32_t generation, char* path, size_t length) {
  snprintf(path, length, NORMAL_HISTORY_MOUNT_PATH "/hist2-%02x%02x%02x%02x%02x%02x-%08lx.bin",
    mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],static_cast<unsigned long>(generation));
}
bool validHeader(const Header& h) {
  return h.magic == kMagic && h.version == 1 && h.generation && h.interval && h.interval <= 86400 &&
    h.width == widthFor(h.kind) && h.width &&
    h.crc == lil::protocol::crc32(reinterpret_cast<const uint8_t*>(&h), offsetof(Header, crc));
}
uint32_t bounded(float value, double scale, uint32_t maximum) {
  if (!std::isfinite(value) || value <= 0) return 0;
  return static_cast<uint32_t>(std::min<double>(maximum, std::round(double(value) * scale)));
}
void encodeEnvironment(EnvironmentRow& row, uint32_t timestamp, const lil::protocol::TelemetryPayload& t) {
  auto& s = row.sample;
  s.timestamp = timestamp; s.capabilities = t.capabilities;
  if (t.flags & lil::protocol::kSensorReadFailed) s.capabilities &= lil::protocol::kBattery;
  if (t.flags & lil::protocol::kBatteryReadFailed) s.capabilities &= ~lil::protocol::kBattery;
  if (t.flags & lil::protocol::kBme680RawFallback) s.capabilities &= ~lil::protocol::kIaq;
  s.temperatureCentiC = std::isfinite(t.temperatureC) ? static_cast<int16_t>(std::round(std::max(-32768.0,std::min(32767.0,double(t.temperatureC)*100)))) : 0;
  s.humidityCentiPercent = bounded(t.humidityPercent,100,UINT16_MAX);
  s.pressureDeciHpa = bounded(t.pressureHpa,10,UINT16_MAX);
  s.iaqDeci = bounded(t.iaq,10,UINT16_MAX);
  s.gasResistanceOhms = bounded(t.gasResistanceOhms,1,UINT32_MAX);
  s.batteryMillivolts = t.batteryMillivolts; s.iaqAccuracy = t.iaqAccuracy;
  s.sensorType = static_cast<uint8_t>(t.sensorType); s.pcbVersion = t.pcbVersion;
  row.crc = lil::protocol::crc32(reinterpret_cast<const uint8_t*>(&s),sizeof(s));
}
bool decode(const uint8_t* bytes, uint8_t kind, NormalHistoryPoint& point) {
  point = {};
  if (kind == kEnvironment) {
    EnvironmentRow row; memcpy(&row,bytes,sizeof(row));
    if (row.crc != lil::protocol::crc32(bytes,sizeof(HistorySample))) return false;
    const auto& s = row.sample; auto& t = point.telemetry;
    point.timestamp = s.timestamp; t.capabilities = s.capabilities;
    t.temperatureC = s.temperatureCentiC/100.0F; t.humidityPercent = s.humidityCentiPercent/100.0F;
    t.pressureHpa = s.pressureDeciHpa/10.0F; t.iaq = s.iaqDeci/10.0F;
    t.gasResistanceOhms = s.gasResistanceOhms; t.batteryMillivolts = s.batteryMillivolts;
    t.sensorType = static_cast<lil::protocol::EnvironmentalSensorType>(s.sensorType);
    t.pcbVersion = s.pcbVersion; t.iaqAccuracy = s.iaqAccuracy;
  } else {
    lil::recording::Record record{}; memcpy(&record,bytes,widthFor(kind));
    if (!lil::recording::valid(record) || kindFor(record.type) != kind) return false;
    point.timestamp = record.sampleMs; point.telemetry = lil::recording::decode(record);
  }
  return point.timestamp >= 1577836800UL && lil::protocol::validTelemetryValues(point.telemetry);
}
size_t rawFreeBytes() { const size_t total=LittleFS.totalBytes(),used=LittleFS.usedBytes(); return total>used?total-used:0; }
}

bool PagedHistory::begin(bool mounted) {
  mutex_ = xSemaphoreCreateMutex();
  if (!mutex_ || !initializeFilesystemLock()) return false;
  mounted_ = mounted;
  if (!mounted_) return true;
  FilesystemGuard fs;
  DIR* directory=opendir(NORMAL_HISTORY_MOUNT_PATH);
  if (!directory) { mounted_=false; return true; }
  while (auto* entry=readdir(directory)) {
    if (strncmp(entry->d_name,"hist2-",6) || strlen(entry->d_name)!=31 || strcmp(entry->d_name+27,".bin")) continue;
    char path[384]; snprintf(path,sizeof(path),NORMAL_HISTORY_MOUNT_PATH "/%s",entry->d_name);
    FILE* file=fopen(path,"rb"); if (!file) continue;
    Header h{}; const bool headerOk=fread(&h,1,sizeof(h),file)==sizeof(h)&&validHeader(h);
    char expected[384]; if (headerOk) pathFor(h.mac,h.generation,expected,sizeof(expected));
    if (!headerOk || strcmp(expected,path) || pageCount_==kMaxPages) { fclose(file); continue; }
    Page page{}; memcpy(page.mac,h.mac,6);page.generation=h.generation;page.interval=h.interval;page.width=h.width;page.kind=h.kind;
    uint8_t row[128];NormalHistoryPoint point{};
    const size_t capacity=(kPageBytes-kHeaderBytes)/h.width;
    while (page.count<capacity&&fread(row,1,h.width,file)==h.width) {
      if (!decode(row,h.kind,point) || (page.last&&point.timestamp<=page.last)) break;
      if (!page.first) page.first=point.timestamp;
      page.last=point.timestamp;++page.count;
    }
    fclose(file);pages_[pageCount_++]=page;generation_=std::max(generation_,h.generation);delay(1);
  }
  closedir(directory);
  std::sort(pages_.begin(),pages_.begin()+pageCount_,[](const Page& a,const Page& b){return a.generation<b.generation;});
  return true;
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
    if(n.enabled&&(!enabled||memcmp(n.mac,c.mac,6))){if(n.buffer)heap_caps_free(n.buffer);n={};}
    if(enabled){
      const bool restored = !n.enabled;
      n.enabled=true;memcpy(n.mac,c.mac,6);
      // Keep the interval associated with buffered rows until they are flushed.
      if (!n.pending) n.interval=c.sleepSeconds;
      if (restored) for(size_t j=0;j<pageCount_;++j)
        if(!memcmp(pages_[j].mac,n.mac,6))n.lastTimestamp=std::max(n.lastTimestamp,pages_[j].last);
    }
  }
  size_t active=0;for(const auto& n:nodes_)if(n.enabled)++active;
  cachedQuota_=active?budgetPages_/active:0;
}
size_t PagedHistory::quotaPages() {
  size_t active=0;for(const auto& n:nodes_)if(n.enabled)++active;
  if(!active)return 0;
  size_t historyBytes=0;for(size_t i=0;i<pageCount_;++i)historyBytes+=kHeaderBytes+size_t(pages_[i].count)*pages_[i].width;
  const size_t total=LittleFS.totalBytes(),used=LittleFS.usedBytes();
  const size_t normalBytes=historyBytes+legacyBytes_;
  const size_t other=used>normalBytes?used-normalBytes:0;
  const size_t reserve=historyWorkspaceReserve();
  const size_t room=total>other+reserve?total-other-reserve:0;
  // Include an allowance for file metadata; admission also checks real free blocks.
  budgetPages_=std::min(kMaxPages,room/(kPageBytes+512));
  cachedQuota_=budgetPages_/active;return cachedQuota_;
}
void PagedHistory::setLegacyBytes(size_t bytes){if(mutex_){Guard lock(mutex_);legacyBytes_=bytes;}}
bool PagedHistory::appendLegacy(const uint8_t mac[6],const HistorySample& sample,uint32_t interval){
  EnvironmentRow row{};row.sample=sample;row.crc=lil::protocol::crc32(reinterpret_cast<const uint8_t*>(&sample),sizeof(sample));
  NormalHistoryPoint point{};return decode(reinterpret_cast<const uint8_t*>(&row),kEnvironment,point)&&append(mac,point.timestamp,point.telemetry,interval,true);
}
size_t PagedHistory::pagesFor(const uint8_t mac[6]) const {size_t count=0;for(size_t i=0;i<pageCount_;++i)if(!memcmp(pages_[i].mac,mac,6))++count;return count;}
bool PagedHistory::erasePage(size_t index){
  if(index>=pageCount_)return false;
  char path[384];pathFor(pages_[index].mac,pages_[index].generation,path,sizeof(path));
  if(::remove(path))return false;
  const int owner=nodeIndex(pages_[index].mac);if(owner>=0&&++nodes_[owner].revision==0)++nodes_[owner].revision;
  for(size_t i=index+1;i<pageCount_;++i)pages_[i-1]=pages_[i];
  --pageCount_;return true;
}
bool PagedHistory::makeSpace(const uint8_t mac[6],bool newPage){
  const size_t quota=quotaPages();if(!quota)return false;
  while(newPage&&pagesFor(mac)>=quota){size_t i=0;while(i<pageCount_&&memcmp(pages_[i].mac,mac,6))++i;if(!erasePage(i))return false;}
  while(pageCount_>=kMaxPages || rawFreeBytes()<historyWorkspaceReserve()+2*kPageBytes){
    size_t chosen=pageCount_;size_t largest=0;
    for(size_t i=0;i<pageCount_;++i){const size_t count=pagesFor(pages_[i].mac);if(count>largest){largest=count;chosen=i;}}
    if(chosen==pageCount_||!erasePage(chosen))return false;
  }
  return true;
}
bool PagedHistory::append(const uint8_t mac[6],uint32_t timestamp,const lil::protocol::TelemetryPayload& telemetry,uint32_t interval,bool batchImport){
  if(!mutex_||timestamp<1577836800UL||!interval||interval>86400||!lil::protocol::validTelemetryValues(telemetry))return false;
  Guard lock(mutex_);const int index=nodeIndex(mac);if(index<0||!mounted_)return false;auto& n=nodes_[index];
  if(timestamp<=n.lastTimestamp)return true;
  const uint8_t kind=kindFor(telemetry.sensorType);const uint16_t width=widthFor(kind);
  if(n.pending&&(n.kind!=kind||n.interval!=interval)&&!flushNode(n))return false;
  n.kind=kind;n.width=width;n.interval=interval;
  if(!n.buffer){
    const size_t bytes=kPageBytes-kHeaderBytes;
    if(heap_caps_get_free_size(MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT)>bytes+1024U*1024U)
      n.buffer=static_cast<uint8_t*>(heap_caps_calloc(bytes,1,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
    if(n.buffer)n.bufferRows=bytes/width;
    else if(heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT)>128U*1024U+8*64){
      n.buffer=static_cast<uint8_t*>(heap_caps_calloc(8*64,1,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));n.bufferRows=n.buffer?8:0;
    }
    if(!n.buffer){n.writeFailed=true;++n.dropped;return false;}
  }
  // A type change may need wider rows in an existing buffer.
  n.bufferRows=std::min<uint16_t>(n.bufferRows,(kPageBytes-kHeaderBytes)/width);
  if(n.pending>=n.bufferRows&&!flushNode(n)){++n.dropped;return false;}
  uint8_t* output=n.buffer+size_t(n.pending)*width;
  auto validTelemetry=telemetry;
  if(telemetry.flags&lil::protocol::kBatteryReadFailed)validTelemetry.capabilities&=~lil::protocol::kBattery;
  if(kind==kEnvironment){EnvironmentRow row{};encodeEnvironment(row,timestamp,validTelemetry);memcpy(output,&row,sizeof(row));}
  else {const auto record=lil::recording::encode(1,timestamp,validTelemetry);memcpy(output,&record,width);}
  if(!n.pending)n.startedMs=millis();
  ++n.pending;n.lastTimestamp=timestamp;if(++n.revision==0)++n.revision;
  if(n.pending==n.bufferRows||(!batchImport&&interval>=60))return flushNode(n);
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
      char path[384];pathFor(p.mac,p.generation,path,sizeof(path));FILE* file=fopen(path,"wb");if(!file){n.writeFailed=true;return false;}
      Header h{};memcpy(h.mac,p.mac,6);h.width=p.width;h.kind=p.kind;h.interval=p.interval;h.generation=p.generation;
      h.crc=lil::protocol::crc32(reinterpret_cast<const uint8_t*>(&h),offsetof(Header,crc));
      const bool ok=fwrite(&h,1,sizeof(h),file)==sizeof(h)&&!fflush(file)&&!fsync(fileno(file));
      const bool closed=fclose(file)==0;if(!ok||!closed){::remove(path);n.writeFailed=true;return false;}
      index=pageCount_;pages_[pageCount_++]=p;
    }else if(rawFreeBytes()<historyWorkspaceReserve()+2*kPageBytes){
      size_t victim=0;while(victim<pageCount_&&victim==index)++victim;
      if(victim==pageCount_||!erasePage(victim)){n.writeFailed=true;return false;}
      // The page array changed; select the append page again.
      continue;
    }
    auto& page=pages_[index];const size_t count=std::min<size_t>(n.pending,(kPageBytes-kHeaderBytes)/n.width-page.count);
    const size_t position=kHeaderBytes+size_t(page.count)*n.width;char path[384];pathFor(page.mac,page.generation,path,sizeof(path));
    FILE* file=fopen(path,"r+b");if(!file){n.writeFailed=true;return false;}
    bool ok=!ftruncate(fileno(file),position)&&!fseek(file,position,SEEK_SET)&&
      fwrite(n.buffer,n.width,count,file)==count&&!fflush(file)&&!fsync(fileno(file));
    uint8_t check[128];NormalHistoryPoint point{};
    for(size_t i=0;ok&&i<count;++i)ok=!fseek(file,position+i*n.width,SEEK_SET)&&fread(check,1,n.width,file)==n.width&&
      !memcmp(check,n.buffer+i*n.width,n.width)&&decode(check,n.kind,point);
    const bool closed=fclose(file)==0;if(!ok||!closed){n.writeFailed=true;return false;}
    NormalHistoryPoint first{};decode(n.buffer,n.kind,first);if(!page.count)page.first=first.timestamp;
    page.last=point.timestamp;page.count+=count;n.pending-=count;
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
bool PagedHistory::reserveForArchive(){
  if(!mutex_||!mounted_)return false;
  Guard lock(mutex_);FilesystemGuard fs;noteArchiveWork();
  while(rawFreeBytes()<historyWorkspaceReserve()+2*kPageBytes){if(!pageCount_||!erasePage(0))return false;}
  return true;
}
bool PagedHistory::clear(const uint8_t mac[6]){
  if(!mutex_||!mounted_)return false;
  Guard lock(mutex_);FilesystemGuard fs;
  for(size_t i=0;i<pageCount_;)if(!memcmp(pages_[i].mac,mac,6)){if(!erasePage(i))return false;}else ++i;
  const int index=nodeIndex(mac);if(index>=0){auto& n=nodes_[index];n.pending=0;n.lastTimestamp=0;n.writeFailed=false;if(++n.revision==0)++n.revision;}
  return true;
}
NormalHistoryInfo PagedHistory::infoLocked(const Node& n)const{
  NormalHistoryInfo info{};info.available=mounted_;info.revision=n.revision;info.pendingPoints=n.pending;info.intervalSeconds=n.interval;
  info.dropped=n.dropped;info.writeFailed=n.writeFailed;
  for(size_t i=0;i<pageCount_;++i)if(!memcmp(pages_[i].mac,n.mac,6)){const auto& p=pages_[i];info.durablePoints+=p.count;
    if(p.count){if(!info.oldest||p.first<info.oldest)info.oldest=p.first;info.newest=std::max(info.newest,p.last);}}
  if(n.pending){NormalHistoryPoint p{};if(decode(n.buffer,n.kind,p)){if(!info.oldest||p.timestamp<info.oldest)info.oldest=p.timestamp;}info.newest=n.lastTimestamp;}
  info.points=info.durablePoints+n.pending;
  info.capacityPoints=cachedQuota_*((kPageBytes-kHeaderBytes)/(n.width?n.width:44));return info;
}
NormalHistoryInfo PagedHistory::info(const uint8_t mac[6]){if(!mutex_)return {};Guard lock(mutex_);const int i=nodeIndex(mac);return i<0?NormalHistoryInfo{}:infoLocked(nodes_[i]);}
size_t PagedHistory::read(const uint8_t mac[6],uint32_t after,uint32_t until,uint32_t oldest,NormalHistoryPoint* output,size_t capacity,bool& stale,bool& error){
  stale=error=false;if(!mutex_||!output)return 0;Guard lock(mutex_);FilesystemGuard fs;
  const int index=nodeIndex(mac);if(index<0)return 0;const auto& n=nodes_[index];const auto info=infoLocked(n);
  if(oldest&&info.oldest>oldest){stale=true;return 0;}if(!until)until=info.newest;
  size_t copied=0;uint8_t bytes[128];NormalHistoryPoint p{};
  for(size_t i=0;i<pageCount_&&copied<capacity;++i){const auto& page=pages_[i];if(memcmp(page.mac,mac,6)||page.last<=after||page.first>until)continue;
    char path[384];pathFor(page.mac,page.generation,path,sizeof(path));FILE* file=fopen(path,"rb");if(!file){error=true;break;}
    for(size_t row=0;row<page.count&&copied<capacity;++row){
      if(fseek(file,kHeaderBytes+row*page.width,SEEK_SET)||fread(bytes,1,page.width,file)!=page.width||!decode(bytes,page.kind,p)){error=true;break;}
      if(p.timestamp>after&&p.timestamp<=until)output[copied++]=p;
    }fclose(file);if(error)break;
  }
  for(size_t i=0;i<n.pending&&copied<capacity;++i){if(!decode(n.buffer+i*n.width,n.kind,p)){error=true;break;}if(p.timestamp>after&&p.timestamp<=until)output[copied++]=p;}
  return copied;
}
}

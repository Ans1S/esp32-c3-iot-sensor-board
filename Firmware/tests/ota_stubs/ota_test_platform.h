#pragma once
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>
#include "ota_protocol.h"
using std::min;
#define PCB_VERSION 4
using esp_err_t = int;
constexpr int ESP_OK = 0;
inline uint32_t testTime = 0;
inline uint32_t millis() { return testTime; }
inline void delay(uint32_t ms) { testTime += ms; }
struct Restart {};
inline void esp_restart() { throw Restart{}; }
struct Esp { uint32_t getFlashChipSize() { return 0x400000; } void restart() { esp_restart(); } };
inline Esp ESP;
inline uint32_t esp_random() { return 42; }
inline std::map<std::string, std::vector<uint8_t>> values;
class Preferences {
 public:
  bool begin(const char*, bool) { return true; }
  void end() {}
  uint32_t getUInt(const char* key, uint32_t fallback) { uint32_t v = fallback; if (getBytesLength(key) == 4) getBytes(key, &v, 4); return v; }
  size_t putUInt(const char* key, uint32_t v) { return putBytes(key, &v, 4); }
  size_t getBytesLength(const char* key) { return values[key].size(); }
  size_t getBytes(const char* key, void* out, size_t size) { if (values[key].size() != size) return 0; memcpy(out, values[key].data(), size); return size; }
  size_t putBytes(const char* key, const void* data, size_t size) { auto p = static_cast<const uint8_t*>(data); values[key] = {p,p+size}; return size; }
  bool remove(const char* key) { return values.erase(key); }
};
struct esp_partition_t { uint32_t address, size; };
inline esp_partition_t running{0x10000, 0x1e0000}, target{0x1f0000, 0x1e0000};
inline std::vector<uint8_t> flash(0x1e0000, 0xff);
inline uint32_t cursor = 0, began = 0, resumed = UINT32_MAX;
inline std::vector<size_t> writeSizes;
inline bool selected = false, aborted = false, failWrite = false, failEnd = false;
enum esp_ota_img_states_t { ESP_OTA_IMG_VALID, ESP_OTA_IMG_PENDING_VERIFY };
inline esp_ota_img_states_t bootState = ESP_OTA_IMG_VALID;
inline bool bootConfirmed = false, rolledBack = false;
using esp_ota_handle_t = uint32_t;
inline const esp_partition_t* esp_ota_get_running_partition() { return &running; }
inline const esp_partition_t* esp_ota_get_next_update_partition(void*) { return &target; }
inline int esp_ota_get_state_partition(const esp_partition_t*, esp_ota_img_states_t* s) { *s = bootState; return 0; }
inline int esp_ota_mark_app_valid_cancel_rollback() { bootConfirmed = true; return 0; }
inline int esp_ota_mark_app_invalid_rollback_and_reboot() { rolledBack = true; throw Restart{}; }
inline int esp_partition_read(const esp_partition_t* p, size_t offset, void* output, size_t size) {
  assert(p == &target && offset + size <= flash.size()); memcpy(output, flash.data()+offset, size); return 0;
}
inline int esp_partition_erase_range(const esp_partition_t* p, size_t offset, size_t size) {
  assert(p == &target && offset%4096 == 0 && size%4096 == 0 && offset+size <= flash.size());
  std::fill(flash.begin()+offset, flash.begin()+offset+size, 0xff); return 0;
}
inline int esp_ota_begin(const esp_partition_t* p, size_t size, esp_ota_handle_t* h) {
  assert(p == &target); ++began; cursor=0; *h=1;
  return esp_partition_erase_range(p, 0, (size+4095)&~4095U);
}
inline int esp_ota_resume(const esp_partition_t* p, size_t, size_t offset, esp_ota_handle_t* h) {
  assert(p == &target); resumed=cursor=offset; *h=1; return 0;
}
inline int esp_ota_write(esp_ota_handle_t, const void* data, size_t size) {
  writeSizes.push_back(size);
  if (failWrite) return -1;
  assert(cursor+size <= flash.size()); auto bytes=static_cast<const uint8_t*>(data);
  for(size_t i=0;i<size;++i) { assert((flash[cursor+i]&bytes[i])==bytes[i]); flash[cursor+i]=bytes[i]; }
  cursor+=size; return 0;
}
inline int esp_ota_abort(esp_ota_handle_t) { aborted=true; return 0; }
inline int esp_ota_end(esp_ota_handle_t) { return failEnd ? -1 : 0; }
inline int esp_ota_set_boot_partition(const esp_partition_t* p) { assert(p==&target); selected=true; return 0; }
struct esp_app_desc_t { uint8_t app_elf_sha256[32]{}; };
inline const esp_app_desc_t* esp_app_get_description() { static esp_app_desc_t d; return &d; }
using esp_timer_handle_t=void*;
struct esp_timer_create_args_t { void(*callback)(void*)=nullptr; const char* name=nullptr; };
inline int esp_timer_create(esp_timer_create_args_t*, esp_timer_handle_t* h) { *h=(void*)1; return 0; }
inline int esp_timer_start_once(esp_timer_handle_t,uint64_t) { return 0; }
inline int esp_timer_stop(esp_timer_handle_t) { return 0; }
inline int esp_timer_delete(esp_timer_handle_t) { return 0; }
// Deterministic hash substitute: exercises checkpoint clone/update ordering.
// Actual ECDSA/SHA-256 package verification is covered by test_ota_package.py.
struct mbedtls_sha256_context { uint32_t value=2166136261U; };
inline void mbedtls_sha256_init(mbedtls_sha256_context* h) { h->value=2166136261U; }
inline void mbedtls_sha256_free(mbedtls_sha256_context*) {}
inline int mbedtls_sha256_starts(mbedtls_sha256_context* h,int) { mbedtls_sha256_init(h); return 0; }
inline void mbedtls_sha256_clone(mbedtls_sha256_context* a,const mbedtls_sha256_context* b) { *a=*b; }
inline int mbedtls_sha256_update(mbedtls_sha256_context* h,const uint8_t* b,size_t n) { while(n--) h->value=(h->value^*b++)*16777619U; return 0; }
inline int mbedtls_sha256_finish(mbedtls_sha256_context* h,uint8_t* out) { for(int i=0;i<8;++i) memcpy(out+i*4,&h->value,4); return 0; }
namespace sensor {
struct SensorRuntimeConfig { bool provisioned=true,stationKnown=true; uint8_t stationMac[6]{2}; float batteryCalibrationFactor=1; };
struct BatteryReading { bool valid=true; uint16_t millivolts=4000; };
inline BatteryReading updateBattery;
struct AdcReader { BatteryReading readBattery(float) { return updateBattery; } };
struct Hardware { uint8_t pcbVersion=4; };
inline Hardware kHardware;
inline lil::ota::Manifest offer;
inline std::vector<uint8_t> image;
inline uint32_t breakAt=UINT32_MAX, readDelayMs=0;
inline bool signatureValid=true, offerEnabled=true;
inline std::vector<lil::ota::State> states;
inline std::vector<lil::ota::Failure> failures;
class EspNowTransport {
 public:
  bool otaExchange(const uint8_t*,const lil::ota::Packet& q,lil::ota::Packet& r) {
    using namespace lil::ota;
    testTime+=2; r={}; r.payload.op=Op::Idle;
    if(q.payload.op==Op::Hello && offerEnabled) {
      r.payload.op=Op::Offer; r.payload.size=sizeof(offer); memcpy(r.payload.data,&offer,sizeof(offer));
    } else if(q.payload.op==Op::Read) {
      testTime += readDelayMs;
      if(q.payload.offset >= breakAt) return false;
      r.payload.op=Op::Data; r.payload.offset=q.payload.offset;
      r.payload.size=min(size_t(kBlockSize),image.size()-q.payload.offset);
      memcpy(r.payload.data,image.data()+q.payload.offset,r.payload.size);
    } else if(q.payload.op==Op::Status) {
      states.push_back(q.payload.state);
      failures.push_back(q.payload.size==1?static_cast<Failure>(q.payload.data[0]):Failure::None);
    }
    return true;
  }
};
void beginOtaBootGuard(); bool otaBootPending(); void finishOtaBootGuard();
void checkOta(EspNowTransport&,const SensorRuntimeConfig&,AdcReader&,uint16_t);
}

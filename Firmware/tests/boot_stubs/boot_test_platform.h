#pragma once
#include "../stubs/Arduino.h"
#include <cassert>
#include <deque>
#include <vector>
#include "environmental_reading.h"
#include "recording_protocol.h"

struct Slept { uint32_t seconds; };
struct Restarted {};
inline int sensorStarts = 0, batteryReads = 0, otaChecks = 0, configWrites = 0;
inline int radioStarts = 0, normalExchanges = 0, lowPowerExchanges = 0;
inline bool radioAvailable = true, stationReplies = false, powerOff = true;
inline std::vector<lil::protocol::TelemetryPacket> sentPackets;
inline uint32_t esp_random() { return 123; }
inline int64_t esp_timer_get_time() { return int64_t(testMillis) * 1000; }
inline void esp_restart() { throw Restarted{}; }
constexpr int GPIO_NUM_9 = 9, GPIO_INTR_LOW_LEVEL = 0;
constexpr int ESP_SLEEP_WAKEUP_GPIO = 1, ESP_SLEEP_WAKEUP_TIMER = 2;
inline int gpio_wakeup_enable(int, int) { return 0; }
inline int gpio_wakeup_disable(int) { return 0; }
inline int esp_sleep_enable_gpio_wakeup() { return 0; }
inline int esp_sleep_enable_timer_wakeup(uint64_t) { return 0; }
inline int esp_sleep_disable_wakeup_source(int) { return 0; }
inline int esp_light_sleep_start() { testMillis += 100; return 0; }
#define SENSOR_LOG_PRINTLN(...) ((void)0)

namespace sensor {
struct BatteryReading { bool valid = false; uint16_t millivolts = 0; };
inline std::deque<BatteryReading> batteries;
struct SensorRuntimeConfig {
  uint32_t revision = 7, sleepSeconds = 600;
  uint8_t wifiChannel = 6, stationMac[6]{1,2,3,4,5,6};
  int8_t txPowerQuarterDbm = 52;
  bool provisioned = true, stationKnown = true, bme680QuickStartComplete = true;
  lil::protocol::EnvironmentalSensorType environmentalSensorType = lil::protocol::EnvironmentalSensorType::kBme280;
  float temperatureOffsetC = 0, batteryCalibrationFactor = 1;
};
inline SensorRuntimeConfig savedConfig;
struct Hardware { uint8_t pcbVersion = PCB_VERSION; uint16_t adcSettleMs = 100; };
inline Hardware kHardware;
class PowerController {
 public:
  void begin() { powerOff = true; }
  void prepareForDeepSleep() { powerOff = true; }
};
class AdcReader {
 public:
  void begin(PowerController&) {}
  void startBatteryMeasurement() {}
  BatteryReading readBattery(float) {
    ++batteryReads; assert(!batteries.empty());
    auto b = batteries.front(); batteries.pop_front(); return b;
  }
  BatteryReading finishBatteryMeasurement(float f) { return readBattery(f); }
};
class SensorConfigStore {
 public:
  bool begin() { return true; }
  bool firmwareChanged() { return false; }
  SensorRuntimeConfig load() { return savedConfig; }
  bool saveIfChanged(const SensorRuntimeConfig&) { ++configWrites; return true; }
  void factoryReset() { ++configWrites; }
};
class EnvironmentalSensor {
 public:
  bool begin(PowerController&, lil::protocol::EnvironmentalSensorType, float) { ++sensorStarts; powerOff = false; return true; }
  EnvironmentalReading read() {
    EnvironmentalReading r{}; r.valid = true; r.sensorType = savedConfig.environmentalSensorType;
    r.temperatureC = 25; r.capabilities = lil::protocol::kTemperature; return r;
  }
  void end() { powerOff = true; }
  void clearIaqState() {}
  void prepareForDeepSleep(uint32_t, lil::protocol::EnvironmentalSensorType) {}
  uint32_t bme680RecommendedSleepSeconds(uint32_t fallback) { return fallback; }
  lil::protocol::EnvironmentalSensorType detectedType() { return savedConfig.environmentalSensorType; }
};
struct ExchangeResult {
  bool delivered = false, configReceived = false;
  lil::protocol::ConfigResponsePayload config{};
  int8_t stationRssi = -60;
};
class EspNowTransport {
 public:
  bool begin() { ++radioStarts; return radioAvailable; }
  void end() {}
  ExchangeResult exchange(const lil::protocol::TelemetryPacket&, const SensorRuntimeConfig&, bool) { ++normalExchanges; return {}; }
  ExchangeResult exchangeLpChannel(const lil::protocol::TelemetryPacket& p, const SensorRuntimeConfig&, uint8_t, bool = false) {
    ++lowPowerExchanges; sentPackets.push_back(p);
    ExchangeResult r; r.configReceived = stationReplies;
    // Even a factory-reset command must not shorten the protection sleep.
    r.config.flags = lil::protocol::kFactoryReset; return r;
  }
  bool recordingExchange(const SensorRuntimeConfig&, const void*, size_t, uint64_t, uint32_t, uint32_t, lil::recording::Ack&) { return false; }
};
class SleepController {
 public:
  [[noreturn]] static void deepSleep(uint32_t seconds, PowerController& power, uint32_t = 0) {
    power.prepareForDeepSleep(); throw Slept{seconds};
  }
};
class RecordingStore {
 public:
  bool begin(lil::protocol::EnvironmentalSensorType) { return true; }
  bool recording() { return false; }
  bool start() { return true; }
  void stop() {}
  bool append(uint64_t, const lil::protocol::TelemetryPayload&) { return true; }
  lil::recording::Status status(uint32_t) { return {}; }
  bool next(lil::recording::Upload&) { return false; }
  void acknowledge(const lil::recording::Record&) {}
  void anchor(uint64_t, uint32_t, uint32_t) {}
};
struct LiveCapture { EnvironmentalReading reading; uint64_t capturedMs; };
class LiveAcquisition {
 public:
  bool begin(EnvironmentalSensor& s, PowerController&, lil::protocol::EnvironmentalSensorType, float) { s.end(); return true; }
  bool active() { return false; }
  bool take(LiveCapture&) { return false; }
  unsigned presses() { return 0; }
  unsigned dropped() { return 0; }
  void stop() {}
  void start() {}
};
inline void beginOtaBootGuard() {}
inline void finishOtaBootGuard() {}
inline bool otaBootPending() { return false; }
inline bool confirmOtaBootAfterContact() { return true; }
inline void checkOta(EspNowTransport&, const SensorRuntimeConfig&, AdcReader&, uint16_t) { ++otaChecks; }
inline void beginDiagnosticLogging() {}
inline uint64_t logicalTimeMs() { return testMillis; }
inline void lowPowerSensorWaitUs(uint32_t us) { testMillis += us / 1000; }
}

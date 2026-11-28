#pragma once
#include "../stubs/Arduino.h"
#include <cassert>
#include <deque>
#include <vector>
#include "environmental_reading.h"
#include "recording_protocol.h"

struct Slept { uint32_t seconds; };
struct Restarted {};
inline int sensorStarts = 0, batteryReads = 0, otaChecks = 0, otaConfirmations = 0, configWrites = 0;
inline int radioStarts = 0, normalExchanges = 0, lowPowerExchanges = 0;
inline bool radioAvailable = true, stationReplies = false, powerOff = true;
inline bool configPersistenceSucceeded = true;
inline bool iaqClearSucceeded = true;
inline bool sensorStartSucceeded = true;
inline bool sensorReadSucceeded = true;
inline bool acquisitionStuck = false;
inline int iaqClears = 0;
inline int ccsSleepWaits = 0, ccsPoweredReports = 0;
inline std::vector<lil::protocol::TelemetryPacket> sentPackets;
inline uint32_t esp_random() { return 123; }
inline int64_t esp_timer_get_time() { return int64_t(testMillis) * 1000; }
inline void esp_restart() { throw Restarted{}; }
constexpr int GPIO_NUM_9 = 9, GPIO_INTR_LOW_LEVEL = 0;
constexpr int INPUT_PULLUP = 1, HIGH = 1, LOW = 0;
inline uint32_t buttonPressMs = 0, buttonReleaseMs = 0;
inline int digitalRead(int) { return testMillis >= buttonPressMs && testMillis < buttonReleaseMs ? LOW : HIGH; }
inline void pinMode(int, int) {}
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
  uint8_t wifiChannel = 6, stationMac[6]{2,2,3,4,5,6};
  int8_t txPowerQuarterDbm = 52;
  bool provisioned = true, stationKnown = true, bme680QuickStartComplete = true;
  lil::protocol::EnvironmentalSensorType environmentalSensorType = lil::protocol::EnvironmentalSensorType::kBme280;
  float temperatureOffsetC = 0, batteryCalibrationFactor = 1;
};
inline SensorRuntimeConfig savedConfig;
inline uint16_t lastReportVoltage = 0;
inline uint32_t lastBroadcastRevision = 0, directedResetRevision = 0;
inline uint8_t directedResetMac[6]{};
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
  bool saveIfChanged(const SensorRuntimeConfig& config) {
    ++configWrites;
    if (!configPersistenceSucceeded) return false;
    savedConfig = config; return true;
  }
  bool factoryReset() { ++configWrites; return configPersistenceSucceeded; }
};
class EnvironmentalSensor {
 public:
  bool begin(PowerController&, lil::protocol::EnvironmentalSensorType, float) { ++sensorStarts; powerOff = !sensorStartSucceeded; return sensorStartSucceeded; }
  EnvironmentalReading readForReport(float) { return read(); }
  bool setCcs811Interval(uint32_t) { return !powerOff; }
  EnvironmentalReading readContinuous() { assert(!powerOff); ++ccsPoweredReports; return read(); }
  EnvironmentalReading read() {
    EnvironmentalReading r{}; r.valid = sensorStartSucceeded && sensorReadSucceeded; r.sensorType = detectedType();
    if (r.valid) { r.temperatureC = 25; r.capabilities = lil::protocol::kTemperature; }
    return r;
  }
  void end() { powerOff = true; }
  bool clearIaqState(bool = false) { ++iaqClears; return iaqClearSucceeded; }
  void prepareForDeepSleep(uint32_t, lil::protocol::EnvironmentalSensorType) {}
  uint32_t bme680RecommendedSleepSeconds(uint32_t fallback) { return fallback; }
  lil::protocol::EnvironmentalSensorType detectedType() {
    return savedConfig.environmentalSensorType == lil::protocol::EnvironmentalSensorType::kAutoDetect
        ? lil::protocol::EnvironmentalSensorType::kBme280 : savedConfig.environmentalSensorType;
  }
};
struct ExchangeResult {
  bool delivered = false, configReceived = false;
  lil::protocol::ConfigResponsePayload config{};
  int8_t stationRssi = -60;
};
inline std::deque<ExchangeResult> configurationReplies;
class EspNowTransport {
 public:
  bool begin() { ++radioStarts; return radioAvailable; }
  void end() {}
  ExchangeResult exchange(const lil::protocol::TelemetryPacket& packet, const SensorRuntimeConfig&, bool, bool = false, bool = false) {
    lastReportVoltage = packet.payload.batteryMillivolts;
    if (packet.payload.flags & lil::protocol::kDiscoveryBeacon) lastBroadcastRevision = packet.payload.appliedConfigRevision;
    ++normalExchanges;
    if (configurationReplies.empty()) return {};
    const auto response = configurationReplies.front(); configurationReplies.pop_front(); return response;
  }
  ExchangeResult acknowledgeReset(const lil::protocol::TelemetryPacket& packet,
                                  const lil::protocol::ConfigResponsePayload& request) {
    directedResetRevision = packet.payload.appliedConfigRevision;
    memcpy(directedResetMac, request.stationMac, 6);
    ExchangeResult result; result.configReceived = true; result.config = request; result.config.flags = 0;
    return result;
  }
  ExchangeResult exchangeLpChannel(const lil::protocol::TelemetryPacket& p, const SensorRuntimeConfig&, uint8_t, bool = false) {
    ++lowPowerExchanges;
    // The real radio finalizes its wire copy after the caller has set all
    // payload fields. Wire-size/retry details are tested with real transport.
    auto wire = p;
    lil::protocol::finalize(wire, lil::protocol::MessageType::kTelemetry, p.header.sequence);
    sentPackets.push_back(wire);
    ExchangeResult r; r.configReceived = stationReplies;
    // Even a factory-reset command must not shorten the protection sleep.
    r.config.flags = lil::protocol::kFactoryReset; return r;
  }
  bool recordingExchange(const SensorRuntimeConfig&, const void*, size_t, uint64_t, uint32_t, uint32_t, lil::recording::Ack&, uint32_t = 0) { return false; }
  bool sendStalled(uint32_t, uint32_t = 5000) const { return false; }
  bool configureLink(uint8_t, int8_t) { return true; }
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
  static bool clearForReset() { return resetStorageReady; }
  inline static bool resetStorageReady = true;
  bool append(uint64_t, const lil::protocol::TelemetryPayload&) { return true; }
  lil::recording::Status status(uint32_t) { return {}; }
  bool next(lil::recording::Upload&) { return false; }
  bool acknowledge(const lil::recording::Record&) { return true; }
  void anchor(uint64_t, uint32_t, uint32_t) {}
};
struct LiveCapture { EnvironmentalReading reading; uint64_t capturedMs; bool recording = false; };
class LiveAcquisition {
 public:
  bool begin(EnvironmentalSensor& s, PowerController&, lil::protocol::EnvironmentalSensorType, float) { s.end(); return true; }
  bool active() { return acquisitionStuck; }
  bool take(LiveCapture&) { return false; }
  unsigned presses() { return 0; }
  unsigned dropped() { return 0; }
  void stop() {}
  void start() {}
  void normal(uint32_t) {}
  bool normal() { return false; }
  void setNormalInterval(uint32_t) {}
  void pauseNormal(bool) {}
  uint32_t nextNormalDueMs(uint32_t) const { return 100; }
};
inline void beginOtaBootGuard() {}
inline void finishOtaBootGuard() {}
inline bool otaBootPending() { return false; }
inline bool confirmOtaBootAfterContact() { ++otaConfirmations; return true; }
inline bool otaReplacesBattery = false;
inline BatteryReading otaBattery{};
inline void checkOta(EspNowTransport&, const SensorRuntimeConfig&, AdcReader&, BatteryReading& battery) {
  ++otaChecks; if (otaReplacesBattery) battery = otaBattery;
}
inline void beginDiagnosticLogging() {}
inline uint64_t logicalTimeMs() { return testMillis; }
inline void lowPowerSensorWaitUs(uint32_t us) {
  if (!powerOff && savedConfig.environmentalSensorType == lil::protocol::EnvironmentalSensorType::kCcs811) {
    assert(us <= 10000000);
    ++ccsSleepWaits;
  }
  testMillis += us / 1000;
}
inline void pushLowPowerWaitInhibit() {}
inline void popLowPowerWaitInhibit() {}
}

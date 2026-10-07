#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "app_config.h"
#include "config_store.h"
#include "lil_protocol.h"
#include "recording_protocol.h"

namespace station {

constexpr uint32_t kHistoryWindowSeconds = 24UL * 60UL * 60UL;

#pragma pack(push, 1)
struct HistorySample {
  uint32_t timestamp = 0;
  uint16_t capabilities = 0;
  int16_t temperatureCentiC = 0;
  uint16_t humidityCentiPercent = 0;
  uint16_t pressureDeciHpa = 0;
  uint16_t iaqDeci = 0;
  uint32_t gasResistanceOhms = 0;
  uint16_t batteryMillivolts = 0;
  uint8_t iaqAccuracy = 0;
  uint8_t sensorType = 0;
  uint8_t pcbVersion = 0;
  int16_t accelerationMilliG[3]{};
  int16_t angularRateDeciDps[3]{};
  uint16_t peakAccelerationMilliG = 0;
  uint16_t peakAngularRateDeciDps = 0;
  uint8_t motionOverrun = 0;
};
#pragma pack(pop)

struct LiveSample {
  uint32_t receivedMs = 0;
  lil::protocol::TelemetryPayload telemetry{};
};
constexpr size_t kLiveCapacity = 1201;

struct SensorRuntime {
  lil::recording::Status recording{};
  bool hasRecordingStatus = false;
  bool hasTelemetry = false;
  bool hasPersistedTelemetry = false;
  lil::protocol::TelemetryPayload telemetry{};
  int8_t stationRssi = 0;
  uint32_t lastSeenMs = 0, receivedIntervalMs = 0;
  uint32_t lastSequence = 0;
  bool hasSequence = false;
  uint32_t receivedPackets = 0;
  int8_t sensorTxPowerQuarterDbm = 52;
  uint8_t strongSignalSamples = 0;
  uint8_t weakSignalSamples = 0;
  bool txPowerInitialized = false;
  bool configPersistencePending = false;
  bool storageInitializationPending = false;
};

struct SensorView {
  SensorConfig config{};
  SensorRuntime runtime{};
  MotionReference motionReference{};
};

class SensorRegistry {
 public:
  bool begin(ConfigStore& store, uint32_t defaultSleepSeconds);
  size_t liveHistory(const uint8_t mac[6], LiveSample* output, size_t capacity) const;
  bool registerTelemetry(const uint8_t mac[6], uint32_t sequence,
                         const lil::protocol::TelemetryPayload& telemetry,
                         int8_t rssi, SensorConfig& responseConfig,
                         int8_t& txPowerQuarterDbm, bool& duplicate,
                         uint32_t& generation);
  bool persistTelemetry(const uint8_t mac[6], uint32_t sequence,
                        const lil::protocol::TelemetryPayload& telemetry,
                        int8_t rssi, uint32_t generation, uint32_t receivedAt);
  size_t views(SensorView* output, size_t capacity) const;
  bool updateConfig(const uint8_t mac[6], const String& name,
                    uint32_t sleepSeconds, uint32_t channelId,
                    const String& writeKey, bool uploadEnabled,
                    const ThingSpeakFieldMapping& fields,
                    uint8_t thingSpeakProfileSlot,
                    lil::protocol::EnvironmentalSensorType sensorType,
                    float temperatureOffsetC,
                    float batteryCalibrationFactor);
  bool setThingSpeakChannel(const uint8_t mac[6], uint32_t channelId,
                            const String& writeKey,
                            uint8_t thingSpeakProfileSlot = 0xFF);
  bool syncThingSpeakProfile(uint8_t slot,
                             const ThingSpeakChannelProfile& profile);
  bool removeThingSpeakProfile(uint8_t slot);
  bool requestFactoryReset(const uint8_t mac[6]);
  bool requestIaqCalibrationReset(const uint8_t mac[6]);
  bool deleteSensor(const uint8_t mac[6]);
  bool setMotionReference(const uint8_t mac[6], bool clear);
  bool channelShared(uint32_t channelId) const;
  bool generationMatches(const uint8_t mac[6], uint32_t generation) const;
  bool findConfig(const uint8_t mac[6], SensorConfig& output,
                  uint32_t* generation = nullptr) const;
  bool cloudUploadMatches(const uint8_t mac[6], uint32_t generation,
                          uint32_t channelId, const char* writeKey,
                          const ThingSpeakFieldMapping& fields) const;
  bool findView(const uint8_t mac[6], SensorView& output) const;
  void updateRecording(const uint8_t mac[6], const lil::recording::Status& status);
  bool needsPersistence(const uint8_t mac[6]) const;
  size_t history(const uint8_t mac[6], HistorySample* output,
                 size_t capacity) const;
  size_t historyCapacity(const uint8_t mac[6]) const;
  uint32_t historyBucketSeconds(const uint8_t mac[6]) const;
  uint32_t historyRevision(const uint8_t mac[6]) const;
  static String formatMac(const uint8_t mac[6]);
  static bool parseMac(const String& text, uint8_t output[6]);

 private:
  int findIndexLocked(const uint8_t mac[6]) const;
  int allocateIndexLocked(const uint8_t mac[6]);
  int identityIndexLocked(const uint8_t mac[6]) const;
  int rememberIdentityLocked(const uint8_t mac[6], const char* name,
                             SensorIdentity* previous = nullptr);
  void commitConfigLocked(size_t index, const SensorConfig& previous,
                          const SensorConfig& saved);
  bool recordHistoryLocked(size_t index,
                           const lil::protocol::TelemetryPayload& telemetry,
                           uint32_t bucketSeconds, uint32_t receivedAt);
  bool loadHistoryLocked(size_t index);
  bool configureHistoryLocked(size_t index, uint32_t bucketSeconds,
                              bool preserveSamples,
                              bool persistImmediately = true);
  bool persistHistoryLocked(size_t index);
  bool persistHistorySampleLocked(size_t index, size_t slot);
  bool saveLatestTelemetryLocked(
      size_t index, const lil::protocol::TelemetryPayload& telemetry,
      int8_t stationRssi);
  void updateTxPowerLocked(size_t index, int8_t rssi, uint8_t pcbVersion);

  struct LatestTelemetry {
    uint32_t magic = 0;
    uint8_t protocolVersion = 0;
    lil::protocol::TelemetryPayload telemetry{};
    int8_t stationRssi = 0;
  };

  struct StoredHistory {
    uint32_t revision = 0;
    uint32_t bucketSeconds = 0;
    size_t capacity = 0;
    HistorySample* samples = nullptr;
  };

  ConfigStore* store_ = nullptr;
  uint32_t defaultSleepSeconds_ = 600;
  mutable SemaphoreHandle_t mutex_ = nullptr;
  mutable SemaphoreHandle_t storageMutex_ = nullptr;
  uint32_t generations_[kMaxSensors]{};
  SensorConfig configs_[kMaxSensors]{};
  SensorIdentity identities_[kMaxSensorIdentities]{};
  SensorRuntime runtime_[kMaxSensors]{};
  StoredHistory history_[kMaxSensors]{};
  struct LiveHistory { LiveSample* samples = nullptr; size_t head = 0, count = 0, capacity = 0; };
  LiveHistory live_[kMaxSensors]{};
};

}  // namespace station

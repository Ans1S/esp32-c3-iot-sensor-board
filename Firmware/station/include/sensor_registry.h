#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "app_config.h"
#include "config_store.h"
#include "lil_protocol.h"

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
};
#pragma pack(pop)

struct SensorRuntime {
  bool hasTelemetry = false;
  bool hasPersistedTelemetry = false;
  lil::protocol::TelemetryPayload telemetry{};
  int8_t stationRssi = 0;
  uint32_t lastSeenMs = 0;
  uint32_t lastSequence = 0;
  bool hasSequence = false;
  uint32_t receivedPackets = 0;
};

struct SensorView {
  SensorConfig config{};
  SensorRuntime runtime{};
};

class SensorRegistry {
 public:
  bool begin(ConfigStore& store, uint32_t defaultSleepSeconds);
  bool registerTelemetry(const uint8_t mac[6], uint32_t sequence,
                         const lil::protocol::TelemetryPayload& telemetry,
                         int8_t rssi, SensorConfig& responseConfig,
                         bool& duplicate);
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
  bool findConfig(const uint8_t mac[6], SensorConfig& output) const;
  bool findView(const uint8_t mac[6], SensorView& output) const;
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
  bool recordHistoryLocked(size_t index,
                           const lil::protocol::TelemetryPayload& telemetry);
  bool loadHistoryLocked(size_t index);
  bool configureHistoryLocked(size_t index, uint32_t bucketSeconds,
                              bool preserveSamples,
                              bool persistImmediately = true);
  bool persistHistoryLocked(size_t index);
  bool persistHistorySampleLocked(size_t index, size_t slot);
  bool saveLatestTelemetryLocked(
      size_t index, const lil::protocol::TelemetryPayload& telemetry,
      int8_t stationRssi);

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
  SensorConfig configs_[kMaxSensors]{};
  SensorRuntime runtime_[kMaxSensors]{};
  StoredHistory history_[kMaxSensors]{};
};

}  // namespace station

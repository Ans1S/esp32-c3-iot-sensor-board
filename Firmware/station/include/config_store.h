#pragma once

#include <Preferences.h>

#include "app_config.h"

namespace station {

class ConfigStore {
 public:
  bool begin();
  StationConfig loadStationConfig();
  bool saveStationConfig(const StationConfig& config);
  bool loadSensorConfigs(SensorConfig* configs, size_t count);
  bool saveSensorConfig(size_t index, const SensorConfig& config);
  bool deleteSensorConfig(size_t index);
  size_t historyBytesLength(size_t index);
  bool loadHistory(size_t index, void* output, size_t length);
  bool loadHistoryRange(size_t index, size_t offset, void* output,
                        size_t length);
  bool saveHistory(size_t index, const void* data, size_t length);
  bool saveHistoryParts(size_t index, const void* header,
                        size_t headerLength, const void* samples,
                        size_t samplesLength);
  bool updateHistory(size_t index, size_t expectedLength,
                     const void* header, size_t headerLength,
                     size_t sampleOffset, const void* sample,
                     size_t sampleLength);
  bool deleteHistory(size_t index);
  bool loadLatestTelemetry(size_t index, void* output, size_t length);
  bool saveLatestTelemetry(size_t index, const void* data, size_t length);
  bool deleteLatestTelemetry(size_t index);
  bool factoryReset();

 private:
  void applyDefaults(StationConfig& config);
  Preferences preferences_;
};

}  // namespace station

#pragma once

#include <ArduinoJson.h>
#include "paged_history.h"

namespace station {

// Stable, versioned column names. Null in an array means that the legacy
// object omitted that field; zero and false remain real measurements.
inline constexpr const char* kHistoryColumns[] = {
  "t", "sensorType", "pcbVersion", "iaqAccuracy", "gap", "temperature",
  "humidity", "pressure", "iaq", "gas", "battery", "ax", "ay", "az",
  "gx", "gy", "gz", "peakAcceleration", "peakAngularRate"
};
inline constexpr size_t kHistoryColumnCount = sizeof(kHistoryColumns) / sizeof(kHistoryColumns[0]);

inline uint32_t historyColumnMask(const NormalHistoryPoint* points, size_t count) {
  uint32_t mask = (1U << 0) | (1U << 1) | (1U << 3) | (1U << 4);
  for (size_t i = 0; i < count; ++i) {
    const auto& t = points[i].telemetry;
    if (t.pcbVersion) mask |= 1U << 2;
    if (t.capabilities & lil::protocol::kTemperature) mask |= 1U << 5;
    if (t.capabilities & lil::protocol::kHumidity) mask |= 1U << 6;
    if (t.capabilities & lil::protocol::kPressure) mask |= 1U << 7;
    if ((t.capabilities & lil::protocol::kIaq) && t.iaqAccuracy) mask |= 1U << 8;
    if (t.capabilities & lil::protocol::kGasResistance) mask |= 1U << 9;
    if (t.capabilities & lil::protocol::kBattery) mask |= 1U << 10;
    if (t.capabilities & lil::protocol::kMotion) mask |= 0xffU << 11;
  }
  return mask;
}

inline void historyPointJson(JsonDocument& row, const NormalHistoryPoint& point) {
  const auto& t = point.telemetry;
  row["t"] = uint64_t(point.timestamp) * 1000;
  row["sensorType"] = static_cast<uint8_t>(t.sensorType);
  if (t.pcbVersion) row["pcbVersion"] = t.pcbVersion;
  row["iaqAccuracy"] = t.iaqAccuracy;
  row["gap"] = bool(t.live.flags & lil::protocol::kLiveGap) || t.motion.fifoOverrun;
  if (t.capabilities & lil::protocol::kTemperature) row["temperature"] = float(t.temperatureC);
  if (t.capabilities & lil::protocol::kHumidity) row["humidity"] = float(t.humidityPercent);
  if (t.capabilities & lil::protocol::kPressure) row["pressure"] = float(t.pressureHpa);
  if ((t.capabilities & lil::protocol::kIaq) && t.iaqAccuracy) row["iaq"] = float(t.iaq);
  if (t.capabilities & lil::protocol::kGasResistance) row["gas"] = t.gasResistanceOhms / 1000.0F;
  if (t.capabilities & lil::protocol::kBattery) row["battery"] = t.batteryMillivolts / 1000.0F;
  if (t.capabilities & lil::protocol::kMotion) {
    const char* keys[] = {"ax", "ay", "az", "gx", "gy", "gz"};
    for (size_t axis = 0; axis < 6; ++axis)
      row[keys[axis]] = float(axis < 3 ? t.motion.accelerationG[axis] : t.motion.angularRateDps[axis - 3]);
    row["peakAcceleration"] = float(t.motion.peakAccelerationG);
    row["peakAngularRate"] = float(t.motion.peakAngularRateDps);
  }
}

inline void compactHistoryPointJson(JsonDocument& result, const JsonDocument& row, uint32_t mask) {
  JsonArray values = result.to<JsonArray>();
  for (size_t i = 0; i < kHistoryColumnCount; ++i)
    if (mask & (1UL << i)) values.add(row[kHistoryColumns[i]]);
}

}  // namespace station

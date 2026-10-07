#pragma once

#include <math.h>
#include "lil_protocol.h"

namespace lil::protocol {
inline bool validTelemetryValues(const TelemetryPayload& telemetry) {
  const auto type = telemetry.sensorType;
  if (type != EnvironmentalSensorType::kAutoDetect &&
      type != EnvironmentalSensorType::kBme280 && type != EnvironmentalSensorType::kBme680 &&
      type != EnvironmentalSensorType::kLsm6dsox && type != EnvironmentalSensorType::kTmp117 &&
      type != EnvironmentalSensorType::kMax30102 && type != EnvironmentalSensorType::kDisabled)
    return false;
  if (static_cast<uint8_t>(telemetry.operatingMode) >
          static_cast<uint8_t>(SensorOperatingMode::kBatteryProtection) ||
      telemetry.live.count > kOpticalBatchSize) return false;
  // Failed sensors may legitimately carry unavailable float placeholders.
  // Consumers honor their failure flags instead of treating them as readings.
  if (telemetry.flags & kSensorReadFailed) return true;
  const uint16_t capabilities = telemetry.capabilities;
  if ((capabilities & kTemperature) && !isfinite(telemetry.temperatureC)) return false;
  if ((capabilities & kHumidity) && !isfinite(telemetry.humidityPercent)) return false;
  if ((capabilities & kPressure) && !isfinite(telemetry.pressureHpa)) return false;
  if ((capabilities & kIaq) && (!(telemetry.flags & kBme680RawFallback)) &&
      (!isfinite(telemetry.iaq) || telemetry.iaqAccuracy > 3)) return false;
  if (capabilities & kMotion) {
    for (size_t i = 0; i < 3; ++i)
      if (!isfinite(telemetry.motion.accelerationG[i]) ||
          !isfinite(telemetry.motion.angularRateDps[i])) return false;
    if (!isfinite(telemetry.motion.peakAccelerationG) ||
        !isfinite(telemetry.motion.peakAngularRateDps)) return false;
  }
  if ((capabilities & kHeartRate) && !isfinite(telemetry.pulse.beatsPerMinute)) return false;
  return true;
}
}  // namespace lil::protocol

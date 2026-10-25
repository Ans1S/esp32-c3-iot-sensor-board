#pragma once
#include "lil_protocol.h"
#include "precision_cadence.h"

namespace station {
inline bool usesNormalHistory(const lil::protocol::TelemetryPayload& telemetry) {
  return !lil::protocol::isLiveSensor(telemetry.sensorType) ||
      (telemetry.flags & lil::protocol::kBatteryProtectionActive) ||
      (lil::timing::supportsNormalPrecisionMeasurements(telemetry.sensorType) &&
       telemetry.operatingMode == lil::protocol::SensorOperatingMode::kEnergySaving);
}
}  // namespace station

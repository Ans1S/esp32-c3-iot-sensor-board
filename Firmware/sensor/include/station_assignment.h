#pragma once
#include <stdint.h>

namespace sensor {
inline bool usableStationMac(const uint8_t mac[6]) {
  return !(mac[0] & 1U) &&
         (mac[0] | mac[1] | mac[2] | mac[3] | mac[4] | mac[5]);
}

template <typename Config>
inline bool hasStationAssignment(const Config& config) {
  return config.provisioned && config.stationKnown &&
         config.wifiChannel >= 1 && config.wifiChannel <= 13 &&
         usableStationMac(config.stationMac);
}
}  // namespace sensor

#pragma once

#include <Arduino.h>

#include "lil_protocol.h"

namespace station {

constexpr uint32_t kConfigMagic = 0x53543230UL;  // "ST20"
constexpr uint16_t kConfigVersion = 5;
constexpr size_t kMaxSensors = 16;
constexpr size_t kMaxThingSpeakChannels = 6;
constexpr uint32_t kSensorStorageVersion = 6;
constexpr uint32_t kMinSleepSeconds = 30;
constexpr uint32_t kMaxSleepSeconds = 24UL * 60UL * 60UL;
constexpr char kDefaultSetupPassword[] = "W-Charger-Setup";
// Internal station-only state stored in SensorConfig::pendingFlags. This bit
// is never sent as a sensor command. It closes the race between the user
// saving a new sensor and the node applying that response on its next wake.
constexpr uint16_t kAwaitingProvisioningAck = 1U << 15;
constexpr uint16_t kSensorCommandFlags =
    lil::protocol::kFactoryReset | lil::protocol::kResetIaqCalibration;

struct ThingSpeakChannelProfile {
  bool occupied = false;
  char name[25] = {};
  uint32_t channelId = 0;
  char readApiKey[33] = {};
  char writeApiKey[33] = {};
};

struct StationConfig {
  uint32_t magic = kConfigMagic;
  uint16_t version = kConfigVersion;
  char hostname[33] = "w-charger";
  char wifiSsid[33] = {};
  char wifiPassword[65] = {};
  char accessPointPassword[65] = {};
  // Per-device salted SHA-256 digest of the optional website password.
  char adminPassword[65] = {};
  char thingSpeakUserApiKey[41] = {};
  uint32_t thingSpeakDefaultChannelId = 0;
  char thingSpeakReadApiKey[33] = {};
  char thingSpeakWriteApiKey[33] = {};
  uint32_t defaultSleepSeconds = 600;
  uint8_t fallbackWifiChannel = 1;
  // Kept in the serialized layout for backwards compatibility.  It means
  // that the setup portal still needs to run once; it is cleared after the
  // first successful home-WLAN connection.
  bool setupPortalRequired = true;
  ThingSpeakChannelProfile thingSpeakChannels[kMaxThingSpeakChannels]{};
};

struct ThingSpeakFieldMapping {
  uint8_t temperature = 1;
  uint8_t humidity = 2;
  uint8_t pressure = 3;
  uint8_t iaq = 0;
  uint8_t battery = 5;
  uint8_t gasResistance = 0;
};

struct SensorConfig {
  bool occupied = false;
  uint8_t mac[6] = {};
  char name[25] = {};
  uint32_t revision = 1;
  uint32_t sleepSeconds = 600;
  uint32_t thingSpeakChannelId = 0;
  char thingSpeakWriteKey[33] = {};
  uint16_t pendingFlags = 0;
  bool cloudUploadEnabled = false;
  ThingSpeakFieldMapping thingSpeakFields{};
  bool provisioned = false;
  uint8_t thingSpeakProfileSlot = 0xFF;
  lil::protocol::EnvironmentalSensorType environmentalSensorType =
      lil::protocol::EnvironmentalSensorType::kAutoDetect;
  float temperatureOffsetC = 0.466F;
  float batteryCalibrationFactor = 1.0F;
  // Retained for storage/API compatibility; ULP-only operation has no
  // separate quick-start phase.
  bool bme680QuickStartComplete = true;
  uint32_t storageVersion = kSensorStorageVersion;
};

template <size_t N>
inline void copyText(char (&destination)[N], const String& source) {
  source.substring(0, N - 1).toCharArray(destination, N);
}

}  // namespace station

#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif
#include <cassert>
#include <limits>
#include "config_store.h"
#include <LittleFS.h>
#include <esp_partition.h>

int64_t testTimeUs = 0;
int main() {
  station::ConfigStore clean;
  assert(clean.begin() && clean.storageAvailable() && LittleFS.formats == 0);
  // Fresh erased flash still initializes without requiring a second upload.
  LittleFS.mountWorks = false;
  station::ConfigStore fresh;
  assert(fresh.begin() && fresh.storageAvailable() && LittleFS.formats == 1);
  // Even one old byte far beyond the superblock forbids automatic erasure.
  testFilesystemBytes.back() = 0;
  LittleFS.mountWorks = false;
  station::ConfigStore damaged;
  assert(damaged.begin() && !damaged.storageAvailable() && LittleFS.formats == 1);
  station::StationConfig settings{};
  assert(damaged.saveStationConfig(settings));
  assert(!damaged.deleteHistory(0) && !damaged.deleteLatestTelemetry(0));
  testFilesystemBytes.back() = 0xFF;
  testPartitionReadFails = true;
  station::ConfigStore unreadable;
  assert(unreadable.begin() && !unreadable.storageAvailable() && LittleFS.formats == 1);
  testPartitionReadFails = false; testPartitionPresent = false;
  station::ConfigStore absent;
  assert(absent.begin() && !absent.storageAvailable() && LittleFS.formats == 1);
  testPartitionPresent = true; LittleFS.formatWorks = false;
  station::ConfigStore failedFormat;
  assert(failedFormat.begin() && !failedFormat.storageAvailable() && LittleFS.formats == 2);
  LittleFS.formatWorks = true;
  testPreferencesBegin = false;
  station::ConfigStore failedNvs;
  assert(!failedNvs.begin()); testPreferencesBegin = true;

  // Unterminated stored strings are bounded before Wi-Fi/JSON/string use.
  std::memset(settings.wifiSsid, 'S', sizeof(settings.wifiSsid));
  std::memset(settings.wifiPassword, 'P', sizeof(settings.wifiPassword));
  std::memset(settings.thingSpeakChannels[0].name, 'N', sizeof(settings.thingSpeakChannels[0].name));
  assert(clean.saveStationConfig(settings));
  auto loadedSettings = clean.loadStationConfig();
  assert(loadedSettings.wifiSsid[32] == 0 && loadedSettings.wifiPassword[64] == 0);
  assert(loadedSettings.thingSpeakChannels[0].name[24] == 0);
  testPreferencesShortRead = true;
  assert(clean.loadStationConfig().wifiSsid[0] == 0);
  testPreferencesShortRead = false;

  station::SensorConfig corrupted{}; corrupted.occupied = true;
  std::memset(corrupted.name, 'N', sizeof(corrupted.name));
  std::memset(corrupted.thingSpeakWriteKey, 'K', sizeof(corrupted.thingSpeakWriteKey));
  corrupted.sleepSeconds = UINT32_MAX; corrupted.revision = 0;
  corrupted.environmentalSensorType = static_cast<lil::protocol::EnvironmentalSensorType>(42);
  corrupted.temperatureOffsetC = std::numeric_limits<float>::quiet_NaN();
  corrupted.batteryCalibrationFactor = std::numeric_limits<float>::infinity();
  corrupted.pendingFlags = UINT16_MAX;
  corrupted.thingSpeakFields = {1, 1, 255, 4, 5, 6};
  assert(clean.saveSensorConfig(0, corrupted));
  station::SensorConfig sensor{}; assert(clean.loadSensorConfigs(&sensor, 1));
  assert(sensor.name[24] == 0 && sensor.thingSpeakWriteKey[32] == 0);
  assert(sensor.sleepSeconds == 600 && sensor.revision == 1);
  assert(sensor.environmentalSensorType == lil::protocol::EnvironmentalSensorType::kAutoDetect);
  assert(isfinite(sensor.temperatureOffsetC) && sensor.batteryCalibrationFactor == 1);
  assert(sensor.pendingFlags == (station::kSensorCommandFlags | station::kAwaitingProvisioningAck));
  assert(sensor.thingSpeakFields.temperature == 1 && sensor.thingSpeakFields.humidity == 0 && sensor.thingSpeakFields.pressure == 0);
  station::DashboardLayout layout{};
  assert(clean.loadDashboardLayout().count == 0);
  layout.columns = 2; layout.count = 1; layout.entries[0].mac[0] = 2;
  layout.entries[0].hidden = 1;
  assert(clean.saveDashboardLayout(layout));
  const auto restored = clean.loadDashboardLayout();
  assert(restored.columns == 2 && restored.count == 1 && restored.entries[0].hidden == 1);
  const auto afterLayout = clean.loadStationConfig();
  assert(!memcmp(&afterLayout, &loadedSettings, sizeof(loadedSettings)));
  assert(clean.loadSensorConfigs(&sensor, 1) && sensor.occupied);
  layout.columns = 4; assert(!clean.saveDashboardLayout(layout));
  testPreferences["dashboard1"].back() ^= 1;
  assert(clean.loadDashboardLayout().count == 0);
  // Compact SID2 identity storage keeps NVS blob size small and remains
  // backward-compatible with legacy fixed-array identity blobs.
  station::SensorIdentity identities[station::kMaxSensorIdentities]{};
  identities[0].mac[0] = 0xAA; identities[0].mac[5] = 0x01;
  station::copyText(identities[0].name, "Kitchen");
  identities[0].motion.enabled = true;
  identities[0].motion.accelerationG[2] = 1.0F;
  identities[station::kMaxSensorIdentities - 1].mac[0] = 0xBB;
  identities[station::kMaxSensorIdentities - 1].mac[5] = 0x40;
  station::copyText(identities[station::kMaxSensorIdentities - 1].name, "Roof");
  assert(clean.saveSensorIdentities(identities, station::kMaxSensorIdentities));
  assert(testPreferences["identities1"].size() < 256);
  station::SensorIdentity reloaded[station::kMaxSensorIdentities]{};
  assert(clean.loadSensorIdentities(reloaded, station::kMaxSensorIdentities));
  assert(reloaded[0].mac[0] == 0xAA && !strcmp(reloaded[0].name, "Kitchen") &&
         reloaded[0].motion.enabled &&
         reloaded[0].motion.accelerationG[2] == 1.0F);
  assert(reloaded[1].mac[0] == 0xBB && !strcmp(reloaded[1].name, "Roof") &&
         reloaded[1].mac[5] == 0x40);
  assert(reloaded[2].mac[0] == 0 && reloaded[2].name[0] == '\0');
  // Legacy fixed-array format still loads cleanly.
  testPreferences["identities1"].assign(
      reinterpret_cast<const uint8_t*>(identities),
      reinterpret_cast<const uint8_t*>(identities) + sizeof(identities));
  station::SensorIdentity fromLegacy[station::kMaxSensorIdentities]{};
  assert(clean.loadSensorIdentities(fromLegacy, station::kMaxSensorIdentities));
  assert(fromLegacy[0].mac[0] == 0xAA && !strcmp(fromLegacy[0].name, "Kitchen"));
  assert(fromLegacy[station::kMaxSensorIdentities - 1].mac[0] == 0xBB &&
         !strcmp(fromLegacy[station::kMaxSensorIdentities - 1].name, "Roof"));
  puts("Station configuration: preserved mount failures, erased-flash initialization, bounded strings and sensor settings passed");
}

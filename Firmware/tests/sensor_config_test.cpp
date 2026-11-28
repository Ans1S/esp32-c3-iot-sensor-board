// Include the production store to model RTC retention independently of NVS.
#include "../sensor/src/sensor_config_store.cpp"
#include <cassert>
#include <limits>
#include <type_traits>
void testRadioTick() { ++testMillis; }

void reset() {
  sensor::rtcConfigCache = {};
  sensorConfigPreferences.clear();
  sensorConfigReadFailure.clear(); sensorConfigWriteFailure.clear();
  sensorConfigOpen = sensorConfigClear = true;
  sensorConfigWrites = sensorConfigReads = 0;
}
template<class T> void storeBlob(const T& config) {
  Preferences p; assert(p.putBytes("config", &config, sizeof(config)) == sizeof(config));
  sensorConfigWrites = 0;
}
template<class T> void legacy() {
  reset(); T old{};
  old.sleepSeconds = 42; old.revision = 9; old.stationKnown = true;
  old.stationMac[0] = 2; old.wifiChannel = 6;
  if constexpr (std::is_same_v<T,sensor::LegacySensorRuntimeConfigV4> ||
                std::is_same_v<T,sensor::LegacySensorRuntimeConfigV5>) old.provisioned = true;
  storeBlob(old);
  sensor::SensorConfigStore store; assert(store.begin());
  auto config = store.load();
  assert(config.version == sensor::kSensorConfigVersion && config.sleepSeconds == 42 &&
      config.revision == 9 && config.stationKnown && config.stationMac[0] == 2);
  assert(sensorConfigWrites == 1);
  // A matching byte length does not establish a legacy schema or identity.
  reset(); old.magic = 0; storeBlob(old);
  sensor::SensorConfigStore corrupt; assert(corrupt.begin());
  config = corrupt.load();
  assert(!config.provisioned && !config.stationKnown && config.revision == 0);
  assert(sensorConfigWrites == 0);
  reset(); old.magic = sensor::kSensorConfigMagic; old.version = 99; storeBlob(old);
  sensor::SensorConfigStore future; assert(future.begin());
  assert(!future.load().stationKnown && sensorConfigWrites == 0);
}
int main() {
  legacy<sensor::LegacySensorRuntimeConfigV1>();
  legacy<sensor::LegacySensorRuntimeConfigV2>();
  legacy<sensor::LegacySensorRuntimeConfigV3>();
  legacy<sensor::LegacySensorRuntimeConfigV4>();
  legacy<sensor::LegacySensorRuntimeConfigV5>();
  for (unsigned damage = 0; damage < 5; ++damage) {
    reset(); sensor::SensorRuntimeConfig broken{};
    broken.provisioned = true; broken.stationKnown = damage != 1;
    broken.revision = 23; broken.stationMac[0] = damage == 0 ? 0 : damage == 2 ? 3 : 2;
    if (damage == 4) memset(broken.stationMac, 0xff, 6);
    broken.wifiChannel = damage == 3 ? 17 : 6;
    broken.batteryCalibrationFactor = 1.11F;
    storeBlob(broken);
    sensor::SensorConfigStore store; assert(store.begin());
    const auto repaired = store.load();
    assert(!repaired.provisioned && !repaired.stationKnown && repaired.revision == 0);
    assert(repaired.wifiChannel == (damage == 3 ? 1 : 6) && repaired.batteryCalibrationFactor == 1.11F);
    assert(sensorConfigWrites == 1 && store.saveIfChanged(repaired) && sensorConfigWrites == 1);
  }
  reset(); sensor::SensorRuntimeConfig config{};
  config.provisioned = config.stationKnown = true;
  config.stationMac[0] = 2; config.revision = 17;
  storeBlob(config);
  sensor::SensorConfigStore initial; assert(initial.begin());
  assert(initial.load().revision == 17 && sensorConfigWrites == 0);
  const unsigned reads = sensorConfigReads;
  sensor::SensorConfigStore retained; assert(retained.begin());
  assert(retained.load().revision == 17 && sensorConfigReads == reads);
  assert(retained.saveIfChanged(config) && sensorConfigWrites == 0);
  config.revision++;
  sensorConfigWriteFailure = "config";
  assert(!retained.saveIfChanged(config) && retained.load().revision == 17);
  sensorConfigWriteFailure.clear();
  assert(retained.saveIfChanged(config) && retained.load().revision == 18);
  sensorConfigClear = false;
  assert(!retained.factoryReset() && retained.load().revision == 18);
  sensorConfigClear = true;
  assert(retained.factoryReset() && !retained.load().stationKnown);
  config.batteryCalibrationFactor = std::numeric_limits<float>::quiet_NaN();
  assert(!retained.saveIfChanged(config));
  reset(); config = {}; config.revision = 23; storeBlob(config);
  sensorConfigReadFailure = "config";
  sensor::SensorConfigStore partial; assert(partial.begin());
  assert(partial.load().revision == 0);
  sensorConfigReadFailure.clear();
  sensor::SensorConfigStore recovered; assert(recovered.begin());
  assert(recovered.load().revision == 23); // A failed read must not poison RTC.
  reset(); sensor::LegacySensorRuntimeConfigV5 old{}; old.revision = 31; storeBlob(old);
  sensorConfigWriteFailure = "config";
  sensor::SensorConfigStore migration; assert(migration.begin());
  const auto migrated = migration.load(); assert(migrated.revision == 31);
  sensorConfigWriteFailure.clear();
  sensor::SensorConfigStore retry; assert(retry.begin());
  assert(retry.saveIfChanged(migrated) && sensorConfigWrites == 1);
  reset(); sensor::SensorConfigStore defaults; assert(defaults.begin());
  const auto fresh = defaults.load();
  assert(defaults.saveIfChanged(fresh) && sensorConfigWrites == 1);
  puts("Sensor configuration: V1-V5 migration, schema identity, partial reads, durable revisions, RTC cache and failed reset passed");
}

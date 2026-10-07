// Compile and execute the production setup/live/protection control flow.
#include "boot_test_platform.h"
#include "../sensor/src/main.cpp"
void testRadioTick() { ++testMillis; }
void resetBoot(bool cold = true) {
  sensorStarts = batteryReads = otaChecks = configWrites = 0;
  radioStarts = normalExchanges = lowPowerExchanges = 0;
  radioAvailable = true; stationReplies = false; powerOff = true;
  configPersistenceSucceeded = iaqClearSucceeded = true; iaqClears = 0;
  sensorStartSucceeded = true;
  sentPackets.clear(); sensor::batteries.clear();
  sensor::configurationReplies.clear();
  if (cold) { rtcState = {}; sensor::savedConfig = {}; testMillis = 100; }
  else testMillis += 86400000;
}
uint32_t boot() {
  try { setup(); } catch (const Slept& s) { return s.seconds; }
  assert(false); return 0;
}
void assertProtected() {
  assert(sensorStarts == 0 && batteryReads == 1 && powerOff);
  assert(normalExchanges == 0 && otaChecks == 0 && configWrites == 0);
  assert(lowPowerExchanges == 1 && sentPackets.size() == 1);
  const auto& p = sentPackets[0];
  assert(p.payload.operatingMode == lil::protocol::SensorOperatingMode::kBatteryProtection);
  assert(p.payload.capabilities == (lil::protocol::kPcbV4PowerGates |
      ((p.payload.flags & lil::protocol::kBatteryReadFailed) ? 0 : lil::protocol::kBattery)));
  assert(p.payload.flags & lil::protocol::kBatteryProtectionActive);
  assert(!(p.payload.flags & (lil::protocol::kSensorReadFailed | lil::protocol::kSensorTypeMismatch)));
  assert(p.payload.temperatureC == 0 && p.payload.motion.sampleCount == 0 && p.payload.live.count == 0);
  assert(lil::protocol::validate(p,sizeof(p),lil::protocol::MessageType::kTelemetry));
}
int main() {
  // Execute actual configuration application, including fixed -> auto changes.
  runtimeConfig = sensor::SensorRuntimeConfig{};
  lil::protocol::ConfigResponsePayload response{};
  response.revision = 8; response.sleepIntervalSeconds = 600;
  response.wifiChannel = 6; response.txPowerQuarterDbm = 52;
  response.batteryCalibrationFactor = 1.0F; response.provisioned = 1;
  memcpy(response.stationMac, runtimeConfig.stationMac, 6);
  response.sensorType = lil::protocol::EnvironmentalSensorType::kTmp117;
  rtcState.hasPairingSnapshot = true;
  assert(applyStationConfig(response));
  assert(runtimeConfig.environmentalSensorType == response.sensorType && !rtcState.hasPairingSnapshot);
  assert(rtcState.sensorRecoveryPending && rtcState.sensorStartAttemptsRemaining == 3);
  assert(!applyStationConfig(response));
  response.sensorType = lil::protocol::EnvironmentalSensorType::kAutoDetect;
  assert(applyStationConfig(response));
  response.sleepIntervalSeconds = 10;
  assert(applyStationConfig(response));
  assert(runtimeConfig.sleepSeconds == 10);
  const auto previousConfig = runtimeConfig;
  const int previousIaqClears = iaqClears;
  response.revision++;
  response.sleepIntervalSeconds = 30;
  response.sensorType = lil::protocol::EnvironmentalSensorType::kBme680;
  response.flags = lil::protocol::kResetIaqCalibration;
  configPersistenceSucceeded = false;
  rtcState.hasPairingSnapshot = true;
  assert(!applyStationConfig(response));
  assert(runtimeConfig.revision == previousConfig.revision &&
      runtimeConfig.sleepSeconds == previousConfig.sleepSeconds &&
      runtimeConfig.environmentalSensorType == previousConfig.environmentalSensorType);
  assert(iaqClears == previousIaqClears + 1 && rtcState.hasPairingSnapshot);
  response.flags = lil::protocol::kFactoryReset;
  assert(!applyStationConfig(response) && rtcState.acknowledgedResetRevision == 0);
  assert(iaqClears == previousIaqClears + 2);
  configPersistenceSucceeded = true;
  response.flags = lil::protocol::kResetIaqCalibration;
  iaqClearSucceeded = false;
  const int writesBeforeClearFailure = configWrites;
  assert(!applyStationConfig(response) && runtimeConfig.revision == previousConfig.revision);
  assert(configWrites == writesBeforeClearFailure);
  iaqClearSucceeded = true;
  assert(applyStationConfig(response) && runtimeConfig.revision == response.revision);
  const int clearsAfterSuccess = iaqClears;
  assert(!applyStationConfig(response) && iaqClears == clearsAfterSuccess);
  response.flags = 0;
  response.sensorType = lil::protocol::EnvironmentalSensorType::kAutoDetect;
  assert(applyStationConfig(response));
  response.sensorType = static_cast<lil::protocol::EnvironmentalSensorType>(42);
  assert(!applyStationConfig(response));
  assert(runtimeConfig.environmentalSensorType == lil::protocol::EnvironmentalSensorType::kAutoDetect);
  // A type change received after a BME reading must not wait 600 seconds.
  resetBoot(); sensor::batteries.push_back({true,3500});
  response.sensorType = lil::protocol::EnvironmentalSensorType::kTmp117;
  response.sleepIntervalSeconds = 600;
  sensor::ExchangeResult changed{}; changed.configReceived = true; changed.config = response;
  sensor::configurationReplies.push_back(changed);
  assert(boot() == 1);
  assert(sensor::savedConfig.environmentalSensorType == lil::protocol::EnvironmentalSensorType::kTmp117);
  // The selected module may need another rail startup before it responds.
  resetBoot(false); sensorStartSucceeded = false; sensor::batteries.push_back({true,3500});
  assert(boot() == 5 && sensorStarts == 1 && normalExchanges == 1);
  assert(rtcState.sensorStartAttemptsRemaining == 2 && rtcState.sensorRecoveryPending);
  resetBoot(false); sensorStartSucceeded = false; sensor::batteries.push_back({true,3500});
  assert(boot() == 5 && rtcState.sensorStartAttemptsRemaining == 1);
  resetBoot(false); sensorStartSucceeded = false; sensor::batteries.push_back({true,3500});
  assert(boot() == 600 && !rtcState.sensorRecoveryPending && rtcState.sensorStartAttemptsRemaining == 0);
  resetBoot(false); sensorStartSucceeded = false; sensor::batteries.push_back({true,3500});
  assert(boot() == 600 && !rtcState.sensorRecoveryPending);
  // Recovery stops promptly on success and allows bounded recovery of a later
  // physical module change without an unlimited fast-wake loop.
  resetBoot(); sensorStartSucceeded = false; sensor::batteries.push_back({true,3500});
  assert(boot() == 5);
  resetBoot(false); sensor::batteries.push_back({true,3500});
  assert(boot() == 600 && !rtcState.sensorRecoveryPending && rtcState.sensorStartAttemptsRemaining == 3);
  resetBoot(false); sensorStartSucceeded = false; sensor::batteries.push_back({true,3500});
  assert(boot() == 5);
  // A retry at five seconds must report despite the ten-minute normal deadline.
  resetBoot(false); testMillis -= 86400000 - 5000;
  sensorStartSucceeded = false; sensor::batteries.push_back({true,3500});
  assert(boot() == 5 && normalExchanges == 1);
  resetBoot(false); sensorStartSucceeded = false; sensor::batteries.push_back({true,2799});
  assert(boot() == 86400); assertProtected();
  // Fixed -> auto restarts the active live driver instead of retaining TMP117.
  resetBoot(); sensor::savedConfig.environmentalSensorType = lil::protocol::EnvironmentalSensorType::kTmp117;
  sensor::batteries.push_back({true,3500});
  changed.config.sensorType = lil::protocol::EnvironmentalSensorType::kAutoDetect;
  sensor::configurationReplies.push_back(changed);
  bool restarted = false;
  try { setup(); } catch (const Restarted&) { restarted = true; }
  assert(restarted && sensor::savedConfig.environmentalSensorType == lil::protocol::EnvironmentalSensorType::kAutoDetect);
  resetBoot();
  sensor::EnvironmentalReading unreadable{};
  runtimeConfig.environmentalSensorType = lil::protocol::EnvironmentalSensorType::kBme280;
  const auto failed = makeTelemetryPacket(unreadable, {}, lil::protocol::SensorOperatingMode::kEnergySaving);
  assert(failed.payload.flags & lil::protocol::kSensorReadFailed);
  assert(!(failed.payload.flags & lil::protocol::kSensorTypeMismatch));
  auto batteryPacket = failed;
  updatePacketBattery(batteryPacket, {true,3500});
  assert((batteryPacket.payload.capabilities & lil::protocol::kBattery) &&
      !(batteryPacket.payload.flags & lil::protocol::kBatteryReadFailed));
  updatePacketBattery(batteryPacket, {false,1000});
  assert(!(batteryPacket.payload.capabilities & lil::protocol::kBattery) &&
      (batteryPacket.payload.flags & lil::protocol::kBatteryReadFailed));
  assert(batteryPacket.payload.flags & lil::protocol::kSensorReadFailed);
  resetBoot(); sensor::batteries.push_back({true,2799});
  assert(boot() == 86400); assertProtected();
  resetBoot(false); sensor::batteries.push_back({true,2949});
  assert(boot() == 86400); assertProtected(); // RTC hysteresis across sleep.
  resetBoot(false); sensor::batteries.push_back({true,2950});
  assert(boot() == 600); assert(sensorStarts == 1 && !rtcState.batteryPaused);
  resetBoot(); sensor::batteries.push_back({true,2800});
  assert(boot() == 600 && sensorStarts == 1); // Threshold is strictly below.
  resetBoot(); sensor::batteries.push_back({false,4000});
  assert(boot() == 86400); assertProtected();
  assert(sentPackets[0].payload.flags & lil::protocol::kBatteryReadFailed);
  resetBoot(); radioAvailable = false; sensor::batteries.push_back({true,2700});
  assert(boot() == 86400 && radioStarts == 1 && sensorStarts == 0 && sentPackets.empty());
  resetBoot(); stationReplies = true; sensor::batteries.push_back({true,2700});
  assert(boot() == 86400); assertProtected(); // Commands and OTA cannot bypass it.
  resetBoot(); sensor::savedConfig.provisioned = false; sensor::batteries.push_back({true,2700});
  assert(boot() == 86400 && radioStarts == 0 && sensorStarts == 0); // No discovery scan.
  resetBoot(); sensor::savedConfig.environmentalSensorType = lil::protocol::EnvironmentalSensorType::kTmp117;
  sensor::batteries.push_back({true,3500}); sensor::batteries.push_back({true,2790});
  assert(boot() == 86400 && sensorStarts == 1 && batteryReads == 2 && powerOff);
  assert(sentPackets.back().payload.flags & lil::protocol::kBatteryProtectionActive);
  puts("V4 production boot flow: threshold, RTC hysteresis, invalid ADC, offline/unpaired station, command deferral, battery-only packet and live shutdown passed");
}

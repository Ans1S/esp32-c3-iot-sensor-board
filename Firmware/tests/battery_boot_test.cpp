// Compile and execute the production setup/live/protection control flow.
#include "boot_test_platform.h"
#include "protocol_counter.h"
#include "../sensor/src/main.cpp"
void testRadioTick() { ++testMillis; }
void resetBoot(bool cold = true) {
  sensorStarts = batteryReads = otaChecks = otaConfirmations = configWrites = 0;
  radioStarts = normalExchanges = lowPowerExchanges = 0;
  ccsSleepWaits = ccsPoweredReports = 0;
  radioAvailable = true; stationReplies = false; powerOff = true;
  sensor::otaReplacesBattery = false; sensor::otaBattery = {};
  configPersistenceSucceeded = iaqClearSucceeded = true; iaqClears = 0;
  buttonPressMs = buttonReleaseMs = 0;
  sensor::RecordingStore::resetStorageReady = true;
  sensorStartSucceeded = true;
  sensorReadSucceeded = true;
  acquisitionStuck = false;
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
void testOnboarding() {
  using namespace lil::protocol;
  resetBoot();
  auto& saved = sensor::savedConfig;
  saved.provisioned = saved.stationKnown = false;
  saved.environmentalSensorType = EnvironmentalSensorType::kAutoDetect;
  sensor::ExchangeResult pending{};
  pending.configReceived = true;
  auto& reply = pending.config;
  reply.revision = 12; reply.sleepIntervalSeconds = 3600; reply.wifiChannel = 6;
  reply.txPowerQuarterDbm = 52; reply.stationMac[0] = 2;
  reply.batteryCalibrationFactor = 1; reply.sensorType = EnvironmentalSensorType::kBme688;
  const auto wake = [&](bool hear, uint16_t mv) {
    sensor::batteries.push_back({true,mv});
    if (hear) sensor::configurationReplies.push_back(pending);
    return boot();
  };
  assert(wake(true,3700) == 10 && saved.environmentalSensorType == EnvironmentalSensorType::kAutoDetect);
  testMillis += 10000;
  const int starts = sensorStarts;
  assert(wake(true,3550) == 10 && sensorStarts == starts && sensor::lastReportVoltage == 3550);
  testMillis += 7200000;
  assert(wake(true,3550) == 300);
  reply.flags = kEnrollmentActive;
  for (unsigned expected : {10,10,10,300}) {
    testMillis += 10000;
    assert(wake(true,3550) == expected);
  }
  testMillis += 300000;
  assert(wake(false,3550) == 300);
  assert(wake(true,3550) == 300);
  testMillis += 310000;
  assert(wake(true,3550) == 10);
  testMillis += 10000; assert(wake(false,3550) == 10);
  testMillis += 10000; assert(wake(false,3550) == 10);
  testMillis += 10000; assert(wake(true,3550) == 300);
  // A failed archive erase must not clear config or acknowledge the command.
  reply.flags = kFactoryReset;
  sensor::RecordingStore::resetStorageReady = false;
  const int writes = configWrites;
  assert(!applyStationConfig(reply) && rtcState.acknowledgedResetRevision == 0 && configWrites == writes);
  sensor::RecordingStore::resetStorageReady = true;
  try { applyStationConfig(reply); assert(false); } catch (const Slept& s) { assert(s.seconds == 1); }
  assert(rtcState.acknowledgedResetRevision == 12 && rtcState.discoveryStartedMs == testMillis);
  assert(!applyStationConfig(reply)); // Duplicate reset only needs another receipt.
  assert(makeTelemetryPacket({}, {}, SensorOperatingMode::kDiscovery).payload.appliedConfigRevision == 0);
  reply.flags = 0; reply.revision = 2;
  reply.stationMac[0] = 4; applyStationConfig(reply);
  assert(rtcState.acknowledgedResetRevision == 12);
  reply.stationMac[0] = 2; configPersistenceSucceeded = false; applyStationConfig(reply);
  assert(rtcState.acknowledgedResetRevision == 12);
  configPersistenceSucceeded = true; applyStationConfig(reply);
  assert(rtcState.acknowledgedResetRevision == 0 && runtimeConfig.revision == 2);
  assert(wake(true,3550) == 10);
  reply.revision = 14; reply.flags = kFactoryReset;
  try { applyStationConfig(reply); assert(false); } catch (const Slept& s) { assert(s.seconds == 1); }
  sensor::directedResetRevision = 0;
  assert(wake(true,3550) == 10);
  assert(sensor::lastBroadcastRevision == 0 && sensor::directedResetRevision == 14 &&
         !memcmp(sensor::directedResetMac,reply.stationMac,6) && rtcState.acknowledgedResetRevision == 0);
  for (unsigned hold : {0,1000,5500,10500}) {
    resetBoot(); saved.batteryCalibrationFactor = 1.1F;
    buttonReleaseMs = testMillis + hold;
    assert(wake(false,3550) == (hold == 5500 ? 10U : 600U));
    assert(saved.provisioned == (hold != 5500) && saved.batteryCalibrationFactor == 1.1F);
  }
  resetBoot(); buttonPressMs = 600; buttonReleaseMs = 6800;
  assert(wake(false,3550) == 10 && !saved.provisioned); // Pressed after boot, during the first second.
  resetBoot(); buttonReleaseMs = testMillis + 5500;
  assert(wake(false,2799) == 86400 && saved.provisioned && configWrites == 0);
  puts("Onboarding: old-node sleeps 10/10/10/300 s, bounded intermittent replies, fresh V4 voltage, scoped reset receipt and local transfer passed");
}
int main() {
  testOnboarding();
  resetBoot();
  rtcState.sequence = 1234;
  sensor::EnvironmentalReading prepared{};
  prepared.valid = true; prepared.temperatureC = 21.125F;
  prepared.capabilities = lil::protocol::kTemperature;
  prepared.sensorType = lil::protocol::EnvironmentalSensorType::kBme280;
  testTelemetryFinalizations = testTelemetryCrcBytes = 0;
  const auto preparedPacket = makeTelemetryPacket(prepared, {true,3800},
      lil::protocol::SensorOperatingMode::kEnergySaving);
  assert(preparedPacket.header.sequence == 1234 && preparedPacket.payload.temperatureC == 21.125F);
  assert(testTelemetryFinalizations == (FLOW_OPTIMIZED ? 0 : 1));
  printf("Main telemetry preparation: %zu CRC finalizations, %zu CRC bytes before transport\n",
         testTelemetryFinalizations, testTelemetryCrcBytes);
  // A maintenance-only BSEC wake must preserve its work/cadence without
  // constructing or signing a telemetry envelope that will never be sent.
  resetBoot();
  sensor::savedConfig.environmentalSensorType = lil::protocol::EnvironmentalSensorType::kBme680;
  sensor::savedConfig.sleepSeconds = 3600;
  sensor::batteries.push_back({true,3500});
  assert(boot() == 300);
  resetBoot(false); testMillis -= 86400000 - 300000;
  sensor::batteries.push_back({true,3500});
  testTelemetryFinalizations = testTelemetryCrcBytes = 0;
  assert(boot() == 300 && sensorStarts == 1 && normalExchanges == 0 && radioStarts == 0);
  assert(testTelemetryFinalizations == (FLOW_OPTIMIZED ? 0 : 1));
  printf("Main BME680 maintenance: %zu telemetry CRC finalizations, no radio\n", testTelemetryFinalizations);
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
  // A successful chip/config probe does not establish a successful BME280
  // conversion. Read failures need the same bounded recovery as missing chips.
  resetBoot(); sensorReadSucceeded = false; sensor::batteries.push_back({true,3500});
  assert(boot() == 5 && sensorStarts == 1 && powerOff);
  assert(rtcState.sensorStartAttemptsRemaining == 2 && rtcState.sensorRecoveryPending);
  assert(sentPackets.back().payload.flags & lil::protocol::kSensorReadFailed);
  resetBoot(false); testMillis -= 86400000 - 5000;
  sensorReadSucceeded = false; sensor::batteries.push_back({true,3500});
  assert(boot() == 5 && normalExchanges == 1 && rtcState.sensorStartAttemptsRemaining == 1);
  resetBoot(false); sensorReadSucceeded = false; sensor::batteries.push_back({true,3500});
  assert(boot() == 600 && !rtcState.sensorRecoveryPending && rtcState.sensorStartAttemptsRemaining == 0);
  resetBoot(false); sensorReadSucceeded = false; sensor::batteries.push_back({true,3500});
  assert(boot() == 600 && rtcState.sensorStartAttemptsRemaining == 0);
  resetBoot(false); sensor::batteries.push_back({true,3500});
  assert(boot() == 600 && rtcState.sensorStartAttemptsRemaining == 3);
  assert(!(sentPackets.back().payload.flags & lil::protocol::kSensorReadFailed));
  resetBoot(false); sensorReadSucceeded = false; sensor::batteries.push_back({true,3500});
  assert(boot() == 5 && rtcState.sensorRecoveryPending);
  resetBoot(false); sensorReadSucceeded = false; sensor::batteries.push_back({true,2799});
  assert(boot() == 86400); assertProtected();
  // BME680 can legitimately return no ready reading during BSEC maintenance.
  resetBoot(); sensor::savedConfig.environmentalSensorType = lil::protocol::EnvironmentalSensorType::kBme680;
  sensorReadSucceeded = false; sensor::batteries.push_back({true,3500});
  assert(boot() == 300 && !rtcState.sensorRecoveryPending && rtcState.sensorStartAttemptsRemaining == 3);
  // BMP280 and extended one-shot sensors use the same bounded read-failure recovery as BME280.
  for (auto oneShotType : {lil::protocol::EnvironmentalSensorType::kBmp280,
                           lil::protocol::EnvironmentalSensorType::kVl53l1x,
                           lil::protocol::EnvironmentalSensorType::kAs7341}) {
    resetBoot(); sensor::savedConfig.environmentalSensorType = oneShotType;
    sensorReadSucceeded = false; sensor::batteries.push_back({true,3500});
    assert(boot() == 5 && rtcState.sensorRecoveryPending && rtcState.sensorStartAttemptsRemaining == 2);
  }
  // CCS811 uses bounded 5 s recovery when startup fails, and shuts down its continuous
  // powered loop immediately into 24 h battery protection if V4 voltage drops below 2800 mV.
  resetBoot(); sensor::savedConfig.environmentalSensorType = lil::protocol::EnvironmentalSensorType::kCcs811;
  sensorStartSucceeded = false; sensor::batteries.push_back({true,3500});
  assert(boot() == 5 && rtcState.sensorRecoveryPending && rtcState.sensorStartAttemptsRemaining == 2);
  resetBoot(); sensor::savedConfig.environmentalSensorType = lil::protocol::EnvironmentalSensorType::kCcs811;
  sensor::savedConfig.sleepSeconds = 60;
  // Six 10-second protective checks and one more report while the rail stays on.
  for (unsigned i = 0; i < 7; ++i) sensor::batteries.push_back({true,3500});
  sensor::batteries.push_back({true,2799});
  assert(boot() == 86400 && powerOff && rtcState.batteryPaused);
  assert(sensorStarts == 1 && normalExchanges == 2 && batteryReads == 8);
  assert(ccsSleepWaits == 7 && ccsPoweredReports == 2);
  // Confirmed station contact confirms trial-boot OTA even if deprovisioned.
  resetBoot(); sensor::batteries.push_back({true,3500});
  sensor::ExchangeResult deprov{}; deprov.configReceived = true; deprov.config.provisioned = 0;
  deprov.config.sleepIntervalSeconds = 600; deprov.config.wifiChannel = 6;
  deprov.config.txPowerQuarterDbm = 52; deprov.config.batteryCalibrationFactor = 1.0F;
  sensor::configurationReplies.push_back(deprov);
  assert(boot() > 0 && otaConfirmations == 1 && otaChecks == 0);
  resetBoot(); sensor::savedConfig.environmentalSensorType = lil::protocol::EnvironmentalSensorType::kAutoDetect;
  sensorReadSucceeded = false; sensor::batteries.push_back({true,3500});
  assert(boot() == 5 && rtcState.lastSensorType == lil::protocol::EnvironmentalSensorType::kBme280);
  resetBoot(false); sensor::batteries.push_back({true,3500});
  assert(boot() == 600 && !rtcState.sensorRecoveryPending && rtcState.sensorStartAttemptsRemaining == 3);
  // Fixed -> auto restarts the active live driver instead of retaining TMP117.
  resetBoot(); sensor::savedConfig.environmentalSensorType = lil::protocol::EnvironmentalSensorType::kTmp117;
  sensor::batteries.push_back({true,3500});
  changed.config.sensorType = lil::protocol::EnvironmentalSensorType::kAutoDetect;
  sensor::configurationReplies.push_back(changed);
  bool restarted = false;
  try { setup(); } catch (const Restarted&) { restarted = true; }
  assert(restarted && sensor::savedConfig.environmentalSensorType == lil::protocol::EnvironmentalSensorType::kAutoDetect);
  // Stop the live path at battery protection after its first two contacts.
  // A confirmed MAC delivery with a missing application ACK must stay on
  // the known channel. A missing MAC delivery still permits recovery.
  for (bool delivered : {true, false}) {
    resetBoot(); sensor::savedConfig.environmentalSensorType = lil::protocol::EnvironmentalSensorType::kTmp117;
    sensor::batteries.push_back({true, 3500}); sensor::batteries.push_back({true, 2790});
    sensor::ExchangeResult reply{}; reply.delivered = delivered;
    sensor::configurationReplies.push_back(reply);
    sensor::configurationReplies.push_back(reply);
    sensor::configurationReplies.push_back(reply);
    assert(boot() == 86400 && normalExchanges == 3);
    assert(lowPowerExchanges == (delivered ? 1 : 3)); // Includes the final battery report.
  }
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
  // A fresh voltage measured during OTA must take precedence immediately,
  // including an invalid ADC reading, rather than wait for the next wake.
  for (auto type : {lil::protocol::EnvironmentalSensorType::kBme280,
                    lil::protocol::EnvironmentalSensorType::kTmp117})
  for (auto low : {sensor::BatteryReading{true,2799}, sensor::BatteryReading{false,3500}}) {
    resetBoot(); sensor::batteries.push_back({true,3500});
    sensor::savedConfig.environmentalSensorType = type;
    sensor::otaReplacesBattery = true; sensor::otaBattery = low;
    sensor::ExchangeResult otaReply{}; otaReply.configReceived = true; otaReply.config.provisioned = 1;
    otaReply.config.revision = sensor::savedConfig.revision;
    otaReply.config.sensorType = type;
    otaReply.config.sleepIntervalSeconds = sensor::savedConfig.sleepSeconds;
    otaReply.config.temperatureOffsetC = sensor::savedConfig.temperatureOffsetC;
    otaReply.config.batteryCalibrationFactor = 1;
    otaReply.config.wifiChannel = sensor::savedConfig.wifiChannel;
    memcpy(otaReply.config.stationMac, sensor::savedConfig.stationMac, 6);
    sensor::configurationReplies.push_back(otaReply);
    assert(boot() == 86400 && otaChecks == 1 && powerOff && rtcState.batteryPaused);
    assert(sentPackets.back().payload.flags & lil::protocol::kBatteryProtectionActive);
    if (!low.valid) assert(sentPackets.back().payload.flags & lil::protocol::kBatteryReadFailed);
  }
  resetBoot(); sensor::savedConfig.environmentalSensorType = lil::protocol::EnvironmentalSensorType::kTmp117;
  sensor::batteries.push_back({true,3500}); sensor::batteries.push_back({true,2790});
  assert(boot() == 86400 && sensorStarts == 1 && batteryReads == 2 && powerOff);
  assert(sentPackets.back().payload.flags & lil::protocol::kBatteryProtectionActive);
  // A stuck acquisition task must not trap battery protection forever.
  resetBoot(); sensor::savedConfig.environmentalSensorType = lil::protocol::EnvironmentalSensorType::kTmp117;
  sensor::batteries.push_back({true,3500}); sensor::batteries.push_back({true,2790});
  acquisitionStuck = true; restarted = false;
  try { setup(); } catch (const Restarted&) { restarted = true; }
  assert(restarted && batteryReads == 2 && testMillis >= 20000 && rtcState.batteryPaused);
  resetBoot(false); sensor::batteries.push_back({true,2900});
  assert(boot() == 86400); assertProtected();
  puts("V4 production boot flow: threshold, RTC hysteresis, invalid ADC, offline/unpaired station, command deferral, battery-only packet and live shutdown passed");
}

// Compile and execute the production setup/live/protection control flow.
#include "boot_test_platform.h"
#include "../sensor/src/main.cpp"
void testRadioTick() { ++testMillis; }
void resetBoot(bool cold = true) {
  sensorStarts = batteryReads = otaChecks = configWrites = 0;
  radioStarts = normalExchanges = lowPowerExchanges = 0;
  radioAvailable = true; stationReplies = false; powerOff = true;
  sentPackets.clear(); sensor::batteries.clear();
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
  sensor::EnvironmentalReading unreadable{};
  runtimeConfig.environmentalSensorType = lil::protocol::EnvironmentalSensorType::kBme280;
  const auto failed = makeTelemetryPacket(unreadable, {}, lil::protocol::SensorOperatingMode::kEnergySaving);
  assert(failed.payload.flags & lil::protocol::kSensorReadFailed);
  assert(!(failed.payload.flags & lil::protocol::kSensorTypeMismatch));
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

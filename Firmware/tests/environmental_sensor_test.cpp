#include <cassert>
#include "environmental_sensor.h"
void testRadioTick() { ++testMillis; }
bool railEnabled = false, expired = false;
std::vector<bool> railTransitions;
uint8_t startedAddress = 0;
using Type = lil::protocol::EnvironmentalSensorType;
Type startedType = Type::kAutoDetect;
namespace sensor {
void PowerController::sensorPower(bool enabled) { railEnabled = enabled; railTransitions.push_back(enabled); }
void beginMeasurementBudget(uint32_t) { expired = false; }
void endMeasurementBudget() {}
bool measurementBudgetExpired() { return expired; }
void measurementWaitUs(uint32_t us) { delay((us + 999) / 1000); }
bool Bme280Driver::begin(uint8_t address) { startedAddress = address; startedType = Type::kBme280; return true; }
EnvironmentalReading Bme280Driver::read() { return {}; }
bool Bme680Driver::begin(uint8_t address, float) { startedAddress = address; startedType = Type::kBme680; return true; }
EnvironmentalReading Bme680Driver::read() { return {}; }
void Bme680Driver::prepareForDeepSleep(uint32_t) {}
uint32_t Bme680Driver::recommendedSleepSeconds(uint32_t fallback) const { return fallback; }
bool Bme680Driver::clearPersistentState() { return !Preferences::failClear; }
bool Lsm6dsoxDriver::begin(uint8_t address) { startedAddress = address; startedType = Type::kLsm6dsox; return true; }
EnvironmentalReading Lsm6dsoxDriver::read() { return {}; }
bool PrecisionSensors::begin(Type type, uint8_t address) { startedAddress = address; startedType = type; return true; }
EnvironmentalReading PrecisionSensors::read() { return {}; }
bool PrecisionSensors::probe(Type type, uint8_t& address) {
  const uint8_t probeAddress = type == Type::kTmp117 ? 0x48 : 0x57;
  if (!Wire.hasDevice(probeAddress)) return false;
  address = probeAddress; return true;
}
}
int main() {
  sensor::PowerController power;
  sensor::EnvironmentalSensor sensor;
  Wire.devices = {{0x76,0x61},{0x77,0x60}};
  const uint32_t fastStartMs = millis();
  assert(sensor.begin(power,Type::kBme280,0));
  assert(millis() - fastStartMs == 12 && Wire.clocks.size() == 1);
  assert(startedAddress == 0x77 && startedType == Type::kBme280);
  sensor.end(); assert(!railEnabled);
  assert(sensor.begin(power,Type::kAutoDetect,0));
  assert(startedAddress == 0x76 && startedType == Type::kBme680);
  sensor.end();
  // A slow selected module must be retried even if a different chip responds.
  Wire.clocks.clear();
  Wire.devices = {{0x76,0x60},{0x48,0x01}};
  Wire.readyOnBeginCount = {{0x48,2}};
  assert(sensor.begin(power,Type::kTmp117,0));
  assert(Wire.clocks.size() == 2 && startedType == Type::kTmp117 &&
      sensor.detectedType() == Type::kTmp117);
  Wire.readyOnBeginCount.clear();
  // begin() must shut down an active previous driver before changing types.
  railTransitions.clear();
  const uint32_t transitionStartMs = millis();
  Wire.devices = {{0x57,0x15}};
  assert(sensor.begin(power,Type::kMax30102,0));
  assert(railTransitions == std::vector<bool>({false,true}));
  assert(millis() - transitionStartMs == 112 && startedType == Type::kMax30102);
  sensor.end();
  Wire.devices = {{0x76,0x60},{0x77,0x61}};
  assert(sensor.begin(power,Type::kBme680,0));
  assert(startedAddress == 0x77 && startedType == Type::kBme680);
  sensor.end();
  Wire.devices = {{0x76,0x61}};
  assert(!sensor.begin(power,Type::kBme280,0));
  assert(sensor.detectedType() == Type::kBme680 && !railEnabled);
  sensor.end();
  Wire.devices.clear();
  assert(!sensor.begin(power,Type::kBme280,0));
  assert(!railEnabled && sensor.detectedType() == Type::kAutoDetect &&
      sensor.read().sensorType == Type::kBme280);
  sensor.end();
  Wire.devices = {{0x57,0x15}};
  assert(sensor.begin(power,Type::kMax30102,0));
  assert(Wire.clocks.back() == 100000);
  sensor.end(); assert(!railEnabled && padMode[4] == INPUT && padMode[5] == INPUT);
  Wire.beginFails = true;
  assert(!sensor.begin(power,Type::kMax30102,0) && !railEnabled);
  assert(padMode[4] == INPUT && padMode[5] == INPUT);
  Wire.beginFails = false;
  assert(sensor.begin(power,Type::kDisabled,0) && !railEnabled && sensor.read().valid);
  sensor.end();
  Preferences::failClear = true;
  assert(!sensor.clearIaqState());
  Preferences::failClear = false;
  assert(sensor.clearIaqState());
  puts("Environmental production dispatch: Bosch fast startup/address selection, bounded mismatch recovery, active type changes, MAX30102 standard bus and rail shutdown passed");
}

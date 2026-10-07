#include <cassert>
#include <limits>
#include "bme680_driver.h"
#include "logical_clock.h"

int64_t testTimeUs = 100000;
uint64_t testLogicalMs = 100;
bool testExpired = false;
unsigned rawStarts = 0;
uint8_t rawStatus = BME68X_NEW_DATA_MSK | BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK;
void advance(uint32_t ms) {
  testMillis += ms; testTimeUs += int64_t(ms) * 1000; testLogicalMs += ms;
}
void testRadioTick() { advance(1); }
namespace sensor {
uint64_t logicalTimeUs() { return testLogicalMs * 1000; }
bool measurementBudgetExpired() { return testExpired; }
void measurementWaitUs(uint32_t us) { advance((us + 999) / 1000); }
}
extern "C" {
int8_t bme68x_init(bme68x_dev*) { return BME68X_OK; }
int8_t bme68x_get_conf(bme68x_conf* conf, bme68x_dev*) { *conf = {}; return BME68X_OK; }
int8_t bme68x_set_conf(bme68x_conf* conf, bme68x_dev*) {
  assert(conf->filter == BME68X_FILTER_OFF && conf->odr == BME68X_ODR_NONE);
  return BME68X_OK;
}
int8_t bme68x_set_heatr_conf(uint8_t mode, const bme68x_heatr_conf* conf, bme68x_dev*) {
  assert(mode == BME68X_FORCED_MODE && conf->heatr_dur == 150);
  return BME68X_OK;
}
int8_t bme68x_set_op_mode(uint8_t mode, bme68x_dev*) {
  assert(mode == BME68X_FORCED_MODE); ++rawStarts; return BME68X_OK;
}
uint32_t bme68x_get_meas_dur(uint8_t, bme68x_conf*, bme68x_dev*) { return 20000; }
int8_t bme68x_get_data(uint8_t, bme68x_data* data, uint8_t* count, bme68x_dev*) {
  *count = 1; data->status = rawStatus; data->temperature = 25;
  data->humidity = 50; data->pressure = 100000; data->gas_resistance = 100000;
  return BME68X_OK;
}
}
int main() {
  using Phase = lil::protocol::IaqCalibrationPhase;
  sensor::Bme680Driver driver;
  sensor::BsecStateStore store;
  assert(store.begin());
  auto reset = [&]() {
    Preferences::failClear = false;
    assert(driver.clearPersistentState()); testBsec = {}; rawStarts = 0;
    Preferences::stateWrites = Preferences::calibrationWrites = 0;
    Preferences::failWrite = false; testExpired = false;
    testMillis = 100; testTimeUs = 100000; testLogicalMs = 100;
  };
  reset();
  assert(driver.begin(0x76,0));
  assert(testBsec.subscribedRate == BSEC_SAMPLE_RATE_ULP);
  auto reading = driver.read();
  assert(reading.valid && !reading.bme680RawFallback && !rawStarts);
  assert(Preferences::stateWrites == 0); // Avoid flash writes on every ULP sample.
  advance(6 * 60 * 60 * 1000);
  reading = driver.read();
  assert(reading.valid && Preferences::stateWrites == 1);
  advance(300000); driver.read(); assert(Preferences::stateWrites == 1);

  // Calibration Ready without its matching CRC-checked baseline is false.
  reset(); assert(store.saveCalibration(86400,true));
  assert(driver.begin(0x76,0)); reading = driver.read();
  assert(reading.iaqCalibrationPhase == Phase::kStabilizing);
  uint8_t state[BSEC_MAX_STATE_BLOB_SIZE]{};
  reset(); assert(store.save(state,sizeof(state)) && store.saveCalibration(86400,true));
  testBsec.stateOk = false;
  assert(driver.begin(0x76,0)); reading = driver.read();
  assert(testBsec.restores == 1 && reading.iaqCalibrationPhase == Phase::kStabilizing);
  reset(); assert(store.save(state,sizeof(state)) && store.saveCalibration(86400,true));
  Preferences::values["bsec"][12] ^= 1; // Corrupt state data, not unused padding.
  assert(driver.begin(0x76,0)); reading = driver.read();
  assert(testBsec.restores == 0 && reading.iaqCalibrationPhase == Phase::kStabilizing);

  // Malformed IAQ must not mark calibration complete or trigger accuracy saves.
  reset(); testBsec.iaq = std::numeric_limits<float>::quiet_NaN(); testBsec.accuracy = 3;
  assert(driver.begin(0x76,0)); reading = driver.read();
  assert(reading.valid && !(reading.capabilities & lil::protocol::kIaq) && reading.iaqAccuracy == 0);
  assert(reading.iaqCalibrationPhase == Phase::kStabilizing && Preferences::stateWrites == 0);

  // Repeated initialization past the first 32-bit clock wrap must not add a
  // second wrap to Bosch timestamps, even within the same physical boot.
  reset(); testLogicalMs = 0x100000000ULL + 100;
  assert(driver.begin(0x76,0)); driver.read();
  assert(testBsec.lastRunTimeMs == int64_t(testLogicalMs));
  advance(300000);
  assert(driver.begin(0x76,0)); driver.read();
  assert(testBsec.lastRunTimeMs == int64_t(testLogicalMs));

  // A future ULP slot does not justify keeping the rail awake for five seconds.
  reset(); testBsec.emit = false; testBsec.nextCallNs = (testLogicalMs + 300000) * 1000000;
  assert(driver.begin(0x76,0)); const uint32_t started = millis(); reading = driver.read();
  assert(reading.valid && reading.bme680RawFallback && millis() - started == 180);
  assert(testBsec.runs == 1 && rawStarts == 1 && driver.recommendedSleepSeconds(300) == 300);

  // Repeated stale output is never relabeled as a fresh BSEC measurement.
  reset(); testBsec.forcedTimestampNs = 100000000;
  assert(driver.begin(0x76,0)); assert(!driver.read().bme680RawFallback);
  testBsec.nextCallNs = (testLogicalMs + 300000) * 1000000;
  reading = driver.read(); assert(reading.bme680RawFallback && rawStarts == 1);

  // A failed durable reset leaves both NVS and the active/cached baseline
  // intact. A successful reset cannot let the old BSEC instance restore it.
  reset(); testBsec.accuracy = 1;
  assert(driver.begin(0x76,0)); reading = driver.read();
  assert(reading.iaqCalibrationPhase == Phase::kReady && Preferences::values.count("bsec"));
  const auto savedBaseline = Preferences::values;
  Preferences::failClear = true;
  assert(!driver.clearPersistentState() && Preferences::values == savedBaseline);
  testBsec.failRun = true; reading = driver.read();
  assert(reading.bme680RawFallback && (reading.capabilities & lil::protocol::kIaq) &&
      reading.iaq == 42 && reading.iaqCalibrationPhase == Phase::kReady);
  Preferences::failClear = false;
  assert(driver.clearPersistentState() && Preferences::values.empty());
  testBsec.failRun = false; reading = driver.read();
  assert(reading.bme680RawFallback && !(reading.capabilities & lil::protocol::kIaq));

  // Gas resistance needs both valid gas ADC and the requested heater target.
  for (uint8_t status : {uint8_t(BME68X_NEW_DATA_MSK | BME68X_GASM_VALID_MSK),
                         uint8_t(BME68X_NEW_DATA_MSK | BME68X_HEAT_STAB_MSK),
                         uint8_t(BME68X_NEW_DATA_MSK | BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK)}) {
    reset(); testBsec.beginOk = false; rawStatus = status;
    assert(driver.begin(0x76,0)); reading = driver.read();
    const bool validGas = (status & BME68X_GASM_VALID_MSK) && (status & BME68X_HEAT_STAB_MSK);
    assert(reading.valid && bool(reading.capabilities & lil::protocol::kGasResistance) == validGas);
  }
  testExpired = true; assert(!driver.read().valid);
  puts("BME680 production driver: ULP cadence, periodic state saves, baseline/CRC recovery, honest IAQ, clock wraps, stale outputs, heater stability and deadlines passed");
}

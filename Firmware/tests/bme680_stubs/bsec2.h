#pragma once
#include <Arduino.h>
#include <bme68x.h>
#include <inc/bsec_datatypes.h>
#define BSEC_INSTANCE_SIZE 3272
#define ARRAY_LEN(array) (sizeof(array) / sizeof(array[0]))
using bsecData = bsec_output_t;
using bsecSensor = bsec_virtual_sensor_t;
struct bsecOutputs {
  bsecData output[BSEC_NUMBER_OUTPUTS]{};
  uint8_t nOutputs = 0;
};
struct BsecScript {
  bool beginOk = true, stateOk = true, emit = true, failRun = false;
  bool includeIaq = true;
  float iaq = 42, temperature = 25, humidity = 50, pressure = 1000;
  uint8_t accuracy = 0;
  unsigned runs = 0, restores = 0;
  int64_t nextCallNs = 0, forcedTimestampNs = -1, lastRunTimeMs = 0;
  float subscribedRate = 0;
};
inline BsecScript testBsec;
// Reproduce Bosch wrapper clock semantics, including begin() retaining its
// overflow counter. The proprietary algorithm itself remains a substitute.
class Bsec2 {
 public:
  struct { int8_t status = BME68X_OK; } sensor;
  bsec_version_t version{};
  bsec_library_return_t status = BSEC_OK;
  void allocateMemory(uint8_t (&)[BSEC_INSTANCE_SIZE]) {}
  bool begin(bme68x_intf, bme68x_read_fptr_t, bme68x_write_fptr_t,
             bme68x_delay_us_fptr_t, void*, unsigned long (*clock)()) {
    clock_ = clock; outputs_ = {}; return testBsec.beginOk;
  }
  bool setConfig(const uint8_t*) { return true; }
  void setTemperatureOffset(float) {}
  bool setState(uint8_t*) { ++testBsec.restores; return testBsec.stateOk; }
  bool getState(uint8_t* bytes) { memset(bytes, 0x2A, BSEC_MAX_STATE_BLOB_SIZE); return true; }
  bool updateSubscription(bsecSensor*, uint8_t, float rate) {
    testBsec.subscribedRate = rate; return true;
  }
  int64_t getTimeMs() {
    const int64_t now = clock_();
    if (lastMillis_ > now) ++wraps_;
    lastMillis_ = now; return now + int64_t(wraps_) * 0x100000000LL;
  }
  int64_t getNextCallNs() const { return testBsec.nextCallNs; }
  const bsecOutputs* getOutputs() const { return outputs_.nOutputs ? &outputs_ : nullptr; }
  bool run() {
    ++testBsec.runs; testBsec.lastRunTimeMs = getTimeMs();
    if (testBsec.failRun) return false;
    if (!testBsec.emit) return true;
    outputs_ = {};
    const int64_t stamp = testBsec.forcedTimestampNs >= 0 ?
        testBsec.forcedTimestampNs : testBsec.lastRunTimeMs * 1000000;
    auto add = [&](bsecSensor id, float signal, uint8_t accuracy = 0) {
      auto& output = outputs_.output[outputs_.nOutputs++];
      output.sensor_id = id; output.signal = signal;
      output.time_stamp = stamp; output.accuracy = accuracy;
    };
    add(BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE, testBsec.temperature);
    add(BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY, testBsec.humidity);
    add(BSEC_OUTPUT_RAW_PRESSURE, testBsec.pressure);
    if (testBsec.includeIaq) add(BSEC_OUTPUT_STATIC_IAQ, testBsec.iaq, testBsec.accuracy);
    add(BSEC_OUTPUT_RAW_GAS, 100000);
    return true;
  }
 private:
  unsigned long (*clock_)() = nullptr;
  int64_t lastMillis_ = 0;
  uint32_t wraps_ = 0;
  bsecOutputs outputs_{};
};

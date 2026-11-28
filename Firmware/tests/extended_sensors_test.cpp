#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <vector>

#include <Arduino.h>
#include <Wire.h>

#include "bme280_driver.h"
#include "extended_sensors.h"
#include "measurement_budget.h"

void testRadioTick() { ++testMillis; }

namespace {
bool budgetExpired = false;

// Minimal deterministic I2C bus simulator supporting 8-bit and 16-bit registers
// for BMP280 (0x76), AS7341 (0x39), TOF400C-VL53L1X (0x29), and CCS811 (0x5A).
struct SimBus {
  uint8_t addr = 0;
  std::vector<uint8_t> tx;
  std::vector<uint8_t> rx;
  size_t rxPos = 0;
  uint32_t clockHz = 100000;

  // Device register maps
  uint8_t bmpRegs[256]{};
  uint8_t asRegs[256]{};
  uint8_t asSmux[20]{};
  bool asSatOnce = false;
  bool asClipDigitalOnce = false;
  uint8_t asPassCount = 0;

  std::map<uint16_t, uint8_t> vlRegs;
  uint16_t vlDistanceMm = 1234;
  uint16_t vlRawSignal = 50;      // 50 * 8 = 400 kcps
  uint16_t vlRawAmbient = 5;      // 5 * 8 = 40 kcps
  uint8_t vlRawStatus = 9;  // ST ULD LUT[9] == 0 (Valid)

  bool ccsPresent = true;
  uint8_t ccsStatus = 0x10;  // APP_VALID=1, FW_MODE=0 (bootloader)
  uint8_t ccsMeasMode = 0;
  uint32_t ccsCycleStarted = 0;
  bool ccsStalled = false;
  bool ccsShortFrame = false;
  unsigned ccsModeWrites = 0;
  unsigned ccsFrames = 0;
  uint16_t ccsEco2 = 612;
  uint16_t ccsTvoc = 42;
  uint8_t ccsError = 0;
  uint8_t ccsBaseline[2] = {0x9A, 0x34};
  uint8_t ccsRestoredBaseline[2] = {0, 0};
  bool ccsBaselineWritten = false;

  void initBmp280() {
    memset(bmpRegs, 0, sizeof(bmpRegs));
    bmpRegs[0xD0] = 0x58;  // BMP280 chip ID
    // Realistic calibration trims
    const uint16_t t1 = 27504;
    const int16_t t2 = 26435, t3 = -1000;
    const uint16_t p1 = 36477;
    const int16_t p2 = -10685, p3 = 3024, p4 = 2855, p5 = 140, p6 = -7, p7 = 15500, p8 = -14600, p9 = 6000;
    auto put16 = [&](uint8_t r, uint16_t v) { bmpRegs[r] = v & 0xFF; bmpRegs[r + 1] = v >> 8; };
    put16(0x88, t1); put16(0x8A, uint16_t(t2)); put16(0x8C, uint16_t(t3));
    put16(0x8E, p1); put16(0x90, uint16_t(p2)); put16(0x92, uint16_t(p3));
    put16(0x94, uint16_t(p4)); put16(0x96, uint16_t(p5)); put16(0x98, uint16_t(p6));
    put16(0x9A, uint16_t(p7)); put16(0x9C, uint16_t(p8)); put16(0x9E, uint16_t(p9));
    // Raw P=415148, T=519888 (~25.08 C, ~1006.53 hPa)
    const uint32_t adcP = 415148, adcT = 519888;
    bmpRegs[0xF7] = (adcP >> 12) & 0xFF; bmpRegs[0xF8] = (adcP >> 4) & 0xFF; bmpRegs[0xF9] = (adcP << 4) & 0xF0;
    bmpRegs[0xFA] = (adcT >> 12) & 0xFF; bmpRegs[0xFB] = (adcT >> 4) & 0xFF; bmpRegs[0xFC] = (adcT << 4) & 0xF0;
  }

  void initAs7341() {
    memset(asRegs, 0, sizeof(asRegs));
    memset(asSmux, 0, sizeof(asSmux));
    asRegs[0x92] = 0x09 << 2;  // ID[7:2] == 0x09
    asPassCount = 0;
  }

  void initVl53l1x() {
    vlRegs.clear();
    vlRegs[0x010F] = 0xEA;
    vlRegs[0x0110] = 0xCC;
    vlRegs[0x00E5] = 0x01;  // Booted
    vlRegs[0x0030] = 0x00;  // Polarity active high -> expected 1
    vlRegs[0x0031] = 0x01;  // Data ready
  }

  void initCcs811() {
    ccsStatus = 0x10;  // APP_VALID=1, FW_MODE=0
    ccsMeasMode = 0;
    ccsStalled = ccsShortFrame = false;
    ccsFrames = ccsModeWrites = 0;
    ccsEco2 = 612;
    ccsTvoc = 42;
    ccsError = 0;
    ccsBaseline[0] = 0x9A;
    ccsBaseline[1] = 0x34;
    ccsRestoredBaseline[0] = 0;
    ccsRestoredBaseline[1] = 0;
    ccsBaselineWritten = false;
  }
} bus;
}  // namespace

bool TwoWire::begin(int, int, uint32_t freq) { bus.clockHz = freq; return true; }
void TwoWire::end() {}
void TwoWire::setClock(uint32_t freq) { bus.clockHz = freq; }
void TwoWire::setTimeOut(uint16_t) {}
void TwoWire::beginTransmission(uint8_t address) { bus.addr = address; bus.tx.clear(); }
size_t TwoWire::write(uint8_t data) { bus.tx.push_back(data); return 1; }
uint8_t TwoWire::endTransmission(bool stop) {
  (void)stop;
  if (bus.addr == 0x76) {
    if (bus.tx.size() >= 2) {
      const uint8_t reg = bus.tx[0], val = bus.tx[1];
      if (reg == 0xE0 && val == 0xB6) {
        bus.bmpRegs[0xF4] = 0;
      } else if (reg == 0xF4 && val == 0x25) {
        bus.bmpRegs[0xF4] = 0x24;  // Forced conversion finishes back to sleep
      } else {
        bus.bmpRegs[reg] = val;
      }
    }
    return 0;
  }
  if (bus.addr == 0x39) {
    if (bus.tx.size() >= 2) {
      const uint8_t reg = bus.tx[0];
      if (reg == 0x00 && bus.tx.size() == 21) {
        memcpy(bus.asSmux, &bus.tx[1], 20);
      } else {
        for (size_t i = 1; i < bus.tx.size(); ++i) {
          uint8_t v = bus.tx[i];
          if (reg + i - 1 == 0x80 && (v & 0x10)) v &= ~0x10;  // SMUXEN auto-clears
          bus.asRegs[reg + i - 1] = v;
        }
        if (reg == 0x80 && (bus.tx[1] & 0x02)) {
          // Spectral measurement triggered -> populate CH0..CH5 based on SMUX pass
          ++bus.asPassCount;
          const bool isLow = bus.asSmux[0] == 0x30;
          const bool sat = bus.asSatOnce && bus.asPassCount <= 2;
          const bool clip = bus.asClipDigitalOnce && bus.asPassCount <= 2;
          bus.asRegs[0xA3] = sat ? 0x58 : 0x40;  // AVALID (+ ASAT if sat)
          static constexpr uint16_t kSatVals[6] = {18000, 18000, 18000, 18000, 18000, 18000};
          static constexpr uint16_t kLowVals[6] = {1000, 1200, 1500, 1800, 9000, 2200};
          static constexpr uint16_t kHighVals[6] = {2100, 2400, 2700, 3000, 9002, 2202};
          const uint16_t* vals = (sat || clip) ? kSatVals : (isLow ? kLowVals : kHighVals);
          for (uint8_t i = 0; i < 6; ++i) {
            bus.asRegs[0x95 + i * 2] = vals[i] & 0xFF;
            bus.asRegs[0x95 + i * 2 + 1] = vals[i] >> 8;
          }
        }
      }
    }
    return 0;
  }
  if (bus.addr == 0x29) {
    if (bus.tx.size() >= 3) {
      const uint16_t reg = (uint16_t(bus.tx[0]) << 8) | bus.tx[1];
      for (size_t i = 2; i < bus.tx.size(); ++i) {
        const uint16_t r = static_cast<uint16_t>(reg + i - 2);
        bus.vlRegs[r] = bus.tx[i];
        if (r == 0x0087 && bus.tx[i] == 0x40) bus.vlRegs[0x0031] = 0x01;  // Data ready
      }
    }
    return 0;
  }
  if (bus.addr == 0x5A && bus.ccsPresent) {
    if (bus.tx.size() == 1 && bus.tx[0] == 0xF4) {
      // APP_START
      bus.ccsStatus |= 0x80;  // FW_MODE=1
    } else if (bus.tx.size() >= 2 && bus.tx[0] == 0x01) {
      bus.ccsMeasMode = bus.tx[1];
      ++bus.ccsModeWrites;
      bus.ccsCycleStarted = millis();
      bus.ccsStatus &= ~0x08;
    } else if (bus.tx == std::vector<uint8_t>({0xFF,0x11,0xE5,0x72,0x8A})) {
      bus.initCcs811();
    } else if (bus.tx.size() == 3 && bus.tx[0] == 0x11) {
      bus.ccsRestoredBaseline[0] = bus.tx[1];
      bus.ccsRestoredBaseline[1] = bus.tx[2];
      bus.ccsBaselineWritten = true;
    }
    return 0;
  }
  return 2;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  bus.rx.clear();
  bus.rxPos = 0;
  if (address == 0x76 && !bus.tx.empty()) {
    const uint8_t reg = bus.tx[0];
    for (uint8_t i = 0; i < quantity; ++i) bus.rx.push_back(bus.bmpRegs[reg + i]);
  } else if (address == 0x39 && !bus.tx.empty()) {
    const uint8_t reg = bus.tx[0];
    for (uint8_t i = 0; i < quantity; ++i) bus.rx.push_back(bus.asRegs[reg + i]);
  } else if (address == 0x29 && bus.tx.size() >= 2) {
    const uint16_t reg = (uint16_t(bus.tx[0]) << 8) | bus.tx[1];
    if (reg == 0x0089 && quantity == 17) {
      // Exact ST VL53L1X ULD result block layout starting at 0x0089:
      // [0]=status, [3..4]=SPADs (non-zero sentinel to catch offset regressions),
      // [5..6]=peak signal, [7..8]=ambient, [13..14]=distance, [15..16]=crosstalk-corrected signal.
      uint8_t blk[17]{};
      blk[0] = bus.vlRawStatus;
      blk[3] = 0xAA; blk[4] = 0xBB;
      blk[5] = static_cast<uint8_t>(bus.vlRawSignal >> 8);
      blk[6] = static_cast<uint8_t>(bus.vlRawSignal & 0xFF);
      blk[7] = static_cast<uint8_t>(bus.vlRawAmbient >> 8);
      blk[8] = static_cast<uint8_t>(bus.vlRawAmbient & 0xFF);
      blk[13] = static_cast<uint8_t>(bus.vlDistanceMm >> 8);
      blk[14] = static_cast<uint8_t>(bus.vlDistanceMm & 0xFF);
      blk[15] = static_cast<uint8_t>(bus.vlRawSignal >> 8);
      blk[16] = static_cast<uint8_t>(bus.vlRawSignal & 0xFF);
      bus.rx.assign(blk, blk + 17);
    } else {
      for (uint8_t i = 0; i < quantity; ++i) {
        const auto it = bus.vlRegs.find(static_cast<uint16_t>(reg + i));
        bus.rx.push_back(it != bus.vlRegs.end() ? it->second : 0);
      }
    }
  } else if (address == 0x5A && bus.ccsPresent && !bus.tx.empty()) {
    const uint8_t reg = bus.tx[0];
    if (reg == 0x20) bus.rx.push_back(0x81);
    else if (reg == 0x00) {
      const uint32_t period = bus.ccsMeasMode == 0x30 ? 60000 : bus.ccsMeasMode == 0x20 ? 10000 : 1000;
      const uint32_t elapsed = millis() - bus.ccsCycleStarted;
      if (bus.ccsMeasMode && !bus.ccsStalled && elapsed >= period) {
        bus.ccsStatus |= 0x08;
        bus.ccsCycleStarted += (elapsed / period) * period;
      }
      bus.rx.push_back(bus.ccsStatus);
    }
    else if (reg == 0x01) bus.rx.push_back(bus.ccsMeasMode);
    else if (reg == 0xE0) {
      bus.rx.push_back(bus.ccsError);
      bus.ccsStatus &= ~0x01;
      bus.ccsError = 0;
    }
    else if (reg == 0x11 && quantity == 2) {
      bus.rx = {bus.ccsBaseline[0], bus.ccsBaseline[1]};
    }
    else if (reg == 0x02 && quantity == 8) {
      bus.rx = {
          static_cast<uint8_t>(bus.ccsEco2 >> 8), static_cast<uint8_t>(bus.ccsEco2 & 0xFF),
          static_cast<uint8_t>(bus.ccsTvoc >> 8), static_cast<uint8_t>(bus.ccsTvoc & 0xFF),
          bus.ccsStatus, bus.ccsError,
          static_cast<uint8_t>((18U << 2) | 0x01U), 0x80U,  // 18 uA, ADC=384
      };
      ++bus.ccsFrames;
      bus.ccsStatus &= ~0x08;
      if (bus.ccsShortFrame) bus.rx.pop_back();
    }
  }
  return static_cast<uint8_t>(bus.rx.size());
}
size_t TwoWire::requestFrom(uint8_t address, size_t quantity) {
  return requestFrom(address, static_cast<uint8_t>(quantity));
}
int TwoWire::available() { return static_cast<int>(bus.rx.size() - bus.rxPos); }
int TwoWire::read() { return bus.rxPos < bus.rx.size() ? bus.rx[bus.rxPos++] : -1; }

namespace sensor {
uint64_t logicalTimeUs() { return uint64_t(testMillis) * 1000; }
void beginMeasurementBudget(uint32_t) { budgetExpired = false; }
void endMeasurementBudget() {}
bool measurementBudgetExpired() { return budgetExpired; }
void measurementWaitUs(uint32_t us) { testMillis += (us + 999) / 1000; }
}  // namespace sensor

int main() {
  using Type = lil::protocol::EnvironmentalSensorType;

  // 1. BMP280 (0x58) - temperature + pressure, no humidity
  bus.initBmp280();
  sensor::Bme280Driver bmp;
  assert(bmp.begin(0x76));
  auto bmpReading = bmp.read();
  assert(bmpReading.valid);
  assert(bmpReading.sensorType == Type::kBmp280);
  assert(bmpReading.capabilities == (lil::protocol::kTemperature | lil::protocol::kPressure));
  assert(fabs(bmpReading.temperatureC - 25.08F) < 0.05F);
  assert(fabs(bmpReading.pressureHpa - 1006.53F) < 0.5F);

  // 2. AS7341 10-channel spectral sensor + auto-gain on saturation
  bus.initAs7341();
  uint8_t addr = 0;
  assert(sensor::ExtendedSensors::probe(Type::kAs7341, addr) && addr == 0x39);
  sensor::ExtendedSensors ext;
  assert(ext.begin(Type::kAs7341, addr));
  bus.asSatOnce = true;
  auto spReading = ext.read();
  assert(spReading.valid);
  assert(spReading.sensorType == Type::kAs7341);
  assert(spReading.capabilities == lil::protocol::kSpectral);
  assert(spReading.extended.spectral.channels[0] == 1000);
  assert(spReading.extended.spectral.channels[4] == 2100);
  assert(spReading.extended.spectral.channels[7] == 3000);
  assert(spReading.extended.spectral.channels[8] == 9001);
  assert(spReading.extended.spectral.channels[9] == 2201);
  assert(spReading.extended.spectral.gainCode == 2);  // Stepped down from 4 -> 2
  assert((spReading.extended.spectral.status & 2U) != 0);
  assert(bus.asRegs[0x80] == 0x00);  // Powered down after capture

  // AS7341 digital full-scale ceiling (18000 counts) triggers auto-gain even without ASAT bit.
  bus.initAs7341();
  bus.asSatOnce = false;
  bus.asClipDigitalOnce = true;
  assert(ext.begin(Type::kAs7341, 0x39));
  auto clipReading = ext.read();
  assert(clipReading.valid && clipReading.extended.spectral.gainCode == 2);
  assert((clipReading.extended.spectral.status & 3U) == 2U);  // Gain adjusted, no longer clipped
  bus.asClipDigitalOnce = false;

  // 3. TOF400C-VL53L1X 4m ToF sensor
  bus.initVl53l1x();
  assert(sensor::ExtendedSensors::probe(Type::kVl53l1x, addr) && addr == 0x29);
  assert(ext.begin(Type::kVl53l1x, addr));
  auto tofReading = ext.read();
  assert(tofReading.valid);
  assert(tofReading.sensorType == Type::kVl53l1x);
  assert(tofReading.capabilities == lil::protocol::kDistance);
  assert(tofReading.extended.distance.distanceMm == 1234);
  assert(tofReading.extended.distance.signalRateKcps == 400);
  assert(tofReading.extended.distance.ambientRateKcps == 40);
  assert(tofReading.extended.distance.rangeStatus == 0);
  assert(bus.vlRegs[0x0087] == 0x00);  // Ranging stopped

  // Close reflective target count rate (>8191 raw) saturates cleanly at UINT16_MAX instead of wrapping.
  bus.vlRawSignal = 10000;
  bus.vlRawAmbient = 9000;
  auto highRateTof = ext.read();
  assert(highRateTof.valid);
  assert(highRateTof.extended.distance.signalRateKcps == UINT16_MAX);
  assert(highRateTof.extended.distance.ambientRateKcps == UINT16_MAX);
  bus.vlRawSignal = 50;
  bus.vlRawAmbient = 5;

  // Reject invalid range status
  bus.vlRawStatus = 4;  // Signal fail -> LUT[4] == 2
  auto badTof = ext.read();
  assert(!badTof.valid && badTof.extended.distance.rangeStatus == 2);

  // 4. CCS811 eCO2 / TVOC gas sensor (with 50 kHz clock-stretching bus)
  bus.initCcs811();
  bus.clockHz = 100000;
  bus.ccsPresent = false;
  assert(!sensor::ExtendedSensors::probe(Type::kCcs811, addr));
  assert(bus.clockHz == 100000);  // Restores 100 kHz when absent
  bus.ccsPresent = true;
  assert(sensor::ExtendedSensors::probe(Type::kCcs811, addr) && addr == 0x5A);
  assert(bus.clockHz == 50000);   // Drops to 50 kHz already during probe
  assert(ext.begin(Type::kCcs811, addr));
  assert(bus.clockHz == 50000);
  assert((bus.ccsStatus & 0x80) != 0);  // Transitioned to app mode
  assert(ext.read().valid && bus.ccsModeWrites == 0);  // Discovery leaves the heater idle.
  assert(ext.setCcs811Interval(60) && bus.ccsMeasMode == 0x30);
  const uint32_t start = millis();
  auto waiting = ext.readContinuous();
  assert(waiting.valid && waiting.capabilities == 0 && waiting.extended.airQuality.status == 0x90);
  assert(millis() == start && bus.ccsFrames == 0 && waiting.iaqCalibrationRemainingMinutes == 20);
  testMillis += 60000;
  assert(ext.setCcs811Interval(600) && bus.ccsModeWrites == 1);
  auto warming = ext.readContinuous();
  assert(warming.valid && warming.capabilities == 0 && warming.iaqCalibrationRemainingMinutes == 19);
  testMillis = start + 20 * 60000;
  auto ccsReading = ext.readContinuous();
  assert(ccsReading.valid && ccsReading.sensorType == Type::kCcs811);
  assert(ccsReading.capabilities == lil::protocol::kAirQualityGas);
  assert(ccsReading.extended.airQuality.eco2Ppm == 612 && ccsReading.extended.airQuality.tvocPpb == 42);
  assert(ccsReading.extended.airQuality.rawCurrentUa == 18 && ccsReading.extended.airQuality.rawAdc == 384);
  assert(ext.readContinuous().capabilities == 0);  // Cannot duplicate a consumed sample.
  bus.ccsStalled = true;
  testMillis += 67000;
  auto stalled = ext.readContinuous();
  assert(!stalled.valid && !stalled.capabilities && stalled.extended.airQuality.status == 0x90);
  bus.ccsStatus |= 0x09;  // Fault with an unread algorithm frame.
  bus.ccsError = 0x20;
  auto heaterFault = ext.readContinuous();
  assert(!heaterFault.valid && heaterFault.extended.airQuality.errorId == 0x20);
  assert(!(bus.ccsStatus & 0x09) && ext.readContinuous().capabilities == 0);
  bus.ccsStalled = false;
  bus.ccsShortFrame = true;
  assert(!ext.readContinuous().valid);
  bus.ccsShortFrame = false;
  bus.ccsEco2 = 9500;
  bus.ccsTvoc = 2400;
  testMillis = start + 86400000;
  auto highVoc = ext.readContinuous();  // The daily baseline checkpoint is now due.
  assert(highVoc.valid && highVoc.extended.airQuality.eco2Ppm == 9500 && highVoc.extended.airQuality.tvocPpb == 2400);
  assert(!bus.ccsBaselineWritten);
  bus.initCcs811();
  assert(ext.begin(Type::kCcs811,addr) && ext.setCcs811Interval(60));
  testMillis += 19 * 60000;
  assert(ext.readContinuous().capabilities == 0 && !bus.ccsBaselineWritten);
  testMillis += 60000;
  auto restored = ext.readContinuous();
  assert(restored.valid && restored.capabilities == 0 && bus.ccsBaselineWritten);
  assert(bus.ccsRestoredBaseline[0] == 0x9A && bus.ccsRestoredBaseline[1] == 0x34);
  testMillis += 60000;
  assert(ext.readContinuous().capabilities == lil::protocol::kAirQualityGas);

  // A faster drive can start immediately. A slower drive requires ten minutes idle.
  assert(ext.setCcs811Interval(9) && bus.ccsMeasMode == 0x10);
  bus.ccsBaselineWritten = false;
  testMillis += 20 * 60000;
  assert(ext.readContinuous().valid && !bus.ccsBaselineWritten);
  assert(ext.setCcs811Interval(10) && bus.ccsMeasMode == 0);
  testMillis += 599999;
  assert(ext.setCcs811Interval(59) && bus.ccsMeasMode == 0);
  ++testMillis;
  assert(ext.setCcs811Interval(59) && bus.ccsMeasMode == 0x20);
  sensor::ExtendedSensors::clearCcs811Baseline();
  bus.initCcs811();
  assert(ext.begin(Type::kCcs811,addr) && ext.setCcs811Interval(60));
  testMillis += 20 * 60000;
  assert(ext.readContinuous().capabilities == lil::protocol::kAirQualityGas && !bus.ccsBaselineWritten);

  // A brownout restarts conditioning. Never publish pre-reset data.
  bus.initCcs811();
  auto brownout = ext.readContinuous();
  assert(!brownout.valid && brownout.capabilities == 0 && (bus.ccsStatus & 0x80));
  assert(ext.setCcs811Interval(60) && bus.ccsMeasMode == 0x30);
  testMillis += 60000;
  assert(ext.readContinuous().iaqCalibrationRemainingMinutes == 19);
  assert(ext.resetCcs811() && bus.ccsMeasMode == 0 && ext.setCcs811Interval(60));
  assert(ext.readContinuous().iaqCalibrationRemainingMinutes == 20);

  puts("Extended sensors (BMP280, AS7341, TOF400C-VL53L1X, CCS811): probe, init, acquisition, faults and power-down passed");
  return 0;
}

#include "extended_sensors.h"

#include <Arduino.h>
#include <Wire.h>
#include <esp_attr.h>
#include <string.h>

#include "logical_clock.h"
#include "low_power_wait.h"
#include "measurement_budget.h"
#include "sensor_log.h"

namespace sensor {
namespace {
using Type = lil::protocol::EnvironmentalSensorType;

// ams-OSRAM AS7341 (DS000504 / AN000633)
constexpr uint8_t kAs7341Address = 0x39;
constexpr uint8_t kAs7341RegEnable = 0x80;
constexpr uint8_t kAs7341RegAtime = 0x81;
constexpr uint8_t kAs7341RegId = 0x92;
constexpr uint8_t kAs7341RegCh0DataL = 0x95;
constexpr uint8_t kAs7341RegStatus2 = 0xA3;
constexpr uint8_t kAs7341RegCfg0 = 0xA9;
constexpr uint8_t kAs7341RegCfg1 = 0xAA;
constexpr uint8_t kAs7341RegCfg6 = 0xAF;
constexpr uint8_t kAs7341RegAstepL = 0xCA;
constexpr uint8_t kAs7341ChipId = 0x09;  // ID[7:2] == 0b001001
// ATIME=29, ASTEP=599 => (29+1)*(599+1)*2.78us = 50.04 ms integration time,
// full-scale ADC ceiling = (29+1)*(599+1) = 18000 counts.
constexpr uint8_t kAs7341Atime = 29;
constexpr uint16_t kAs7341Astep = 599;
constexpr uint16_t kAs7341IntegrationMs = 50;
constexpr uint16_t kAs7341FullScale =
    static_cast<uint16_t>(static_cast<uint32_t>(kAs7341Atime + 1U) *
                          static_cast<uint32_t>(kAs7341Astep + 1U));

// SMUX RAM mapping (0x00..0x13) for F1..F4 + Clear + NIR -> ADC0..ADC5.
constexpr uint8_t kAs7341SmuxLow[20] = {
    0x30, 0x01, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x50, 0x00,
    0x00, 0x00, 0x20, 0x04, 0x00, 0x30, 0x01, 0x50, 0x00, 0x06,
};
// SMUX RAM mapping (0x00..0x13) for F5..F8 + Clear + NIR -> ADC0..ADC5.
constexpr uint8_t kAs7341SmuxHigh[20] = {
    0x00, 0x00, 0x00, 0x40, 0x02, 0x00, 0x10, 0x03, 0x50, 0x10,
    0x03, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x50, 0x00, 0x06,
};

// STMicroelectronics VL53L1X (TOF400C, STSW-IMG009 ULD)
constexpr uint8_t kVl53l1xAddress = 0x29;
constexpr uint16_t kVl53l1xRegSoftReset = 0x0000;
constexpr uint16_t kVl53l1xRegDefaultBase = 0x002D;
constexpr uint16_t kVl53l1xRegGpioHvMuxCtrl = 0x0030;
constexpr uint16_t kVl53l1xRegGpioTioHvStatus = 0x0031;
constexpr uint16_t kVl53l1xRegSystemInterruptClear = 0x0086;
constexpr uint16_t kVl53l1xRegSystemModeStart = 0x0087;
constexpr uint16_t kVl53l1xRegResultBase = 0x0089;
constexpr uint16_t kVl53l1xRegFirmwareStatus = 0x00E5;
constexpr uint16_t kVl53l1xRegModelId = 0x010F;

// Official ST ULD 91-byte default configuration (0x002D..0x0087, 100ms Long Mode).
constexpr uint8_t kVl53l1xDefaultConfig[91] = {
    0x00, 0x01, 0x01, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08,
    0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02,
    0x0A, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,
    0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01,
    0xDB, 0x0F, 0x01, 0xF1, 0x0D, 0x01, 0x68, 0x00, 0x80, 0x08,
    0xB8, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00,
};

// ST ULD VL53L1X_GetRangeStatus lookup table (raw status & 0x1F -> ULD code).
constexpr uint8_t kVl53l1xStatusLut[24] = {
    255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255,
    9, 13, 255, 255, 255, 255, 10, 6, 255, 255, 11, 12,
};

// ams CCS811 (DS000459)
constexpr uint8_t kCcs811Addresses[2] = {0x5A, 0x5B};
constexpr uint8_t kCcs811RegStatus = 0x00;
constexpr uint8_t kCcs811RegMeasMode = 0x01;
constexpr uint8_t kCcs811RegAlgResultData = 0x02;
constexpr uint8_t kCcs811RegBaseline = 0x11;
constexpr uint8_t kCcs811RegHwId = 0x20;
constexpr uint8_t kCcs811RegErrorId = 0xE0;
constexpr uint8_t kCcs811RegAppStart = 0xF4;
constexpr uint8_t kCcs811RegSwReset = 0xFF;
constexpr uint8_t kCcs811HwId = 0x81;

constexpr uint32_t kCcsConditioningMs = 20UL * 60 * 1000;
constexpr uint32_t kCcsCoolingMs = 10UL * 60 * 1000;
constexpr uint32_t kCcsDayMs = 24UL * 60 * 60 * 1000;
constexpr uint8_t kCcsAppRunning = 0x90;  // FW_MODE | APP_VALID
constexpr uint8_t kCcsDataReady = 0x08;
constexpr uint8_t kCcsError = 0x01;

struct Ccs811RtcBaseline {
  uint64_t savedMs;
  uint8_t address;
  uint8_t bytes[2];
};
// Modes 1, 2 and 3 learn different baselines (ams AN000370).
RTC_DATA_ATTR Ccs811RtcBaseline rtcCcs811Baselines[3]{};

uint16_t little16(const uint8_t* b) {
  return static_cast<uint16_t>(b[0]) | (static_cast<uint16_t>(b[1]) << 8);
}

uint16_t big16(const uint8_t* b) {
  return (static_cast<uint16_t>(b[0]) << 8) | static_cast<uint16_t>(b[1]);
}
}  // namespace

bool ExtendedSensors::readReg8(uint8_t address, uint8_t reg, uint8_t* data, uint8_t count) {
  if (measurementBudgetExpired()) return false;
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, count) != count) return false;
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

bool ExtendedSensors::writeReg8(uint8_t address, uint8_t reg, uint8_t value) {
  if (measurementBudgetExpired()) return false;
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool ExtendedSensors::writeReg8Burst(uint8_t address, uint8_t reg,
                                     const uint8_t* data, uint8_t count) {
  if (measurementBudgetExpired()) return false;
  Wire.beginTransmission(address);
  Wire.write(reg);
  for (uint8_t i = 0; i < count; ++i) Wire.write(data[i]);
  return Wire.endTransmission() == 0;
}

bool ExtendedSensors::readReg16(uint8_t address, uint16_t reg, uint8_t* data, uint8_t count) {
  if (measurementBudgetExpired()) return false;
  Wire.beginTransmission(address);
  Wire.write(static_cast<uint8_t>(reg >> 8));
  Wire.write(static_cast<uint8_t>(reg & 0xFFU));
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, count) != count) return false;
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

bool ExtendedSensors::writeReg16(uint8_t address, uint16_t reg, uint8_t value) {
  if (measurementBudgetExpired()) return false;
  Wire.beginTransmission(address);
  Wire.write(static_cast<uint8_t>(reg >> 8));
  Wire.write(static_cast<uint8_t>(reg & 0xFFU));
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool ExtendedSensors::writeReg16Burst(uint8_t address, uint16_t reg,
                                      const uint8_t* data, uint8_t count) {
  if (measurementBudgetExpired()) return false;
  Wire.beginTransmission(address);
  Wire.write(static_cast<uint8_t>(reg >> 8));
  Wire.write(static_cast<uint8_t>(reg & 0xFFU));
  for (uint8_t i = 0; i < count; ++i) Wire.write(data[i]);
  return Wire.endTransmission() == 0;
}

bool ExtendedSensors::probe(Type type, uint8_t& address) {
  uint8_t bytes[2]{};
  if (type == Type::kAs7341) {
    // Ensure REG_BANK = 0 before reading ID (0x92).
    if (!readReg8(kAs7341Address, kAs7341RegId, bytes, 1)) return false;
    if ((bytes[0] >> 2) == kAs7341ChipId) {
      address = kAs7341Address;
      return true;
    }
    return false;
  }
  if (type == Type::kVl53l1x) {
    if (readReg16(kVl53l1xAddress, kVl53l1xRegModelId, bytes, 2) &&
        bytes[0] == 0xEA && bytes[1] == 0xCC) {
      address = kVl53l1xAddress;
      return true;
    }
    return false;
  }
  if (type == Type::kCcs811) {
    // CCS811 stretches SCL even during HW_ID reads (DS000459 §6).
    Wire.setClock(50000);
    Wire.setTimeOut(100);
    for (const uint8_t addr : kCcs811Addresses) {
      if (ccs811ReadReg(addr, kCcs811RegHwId, bytes, 1) && bytes[0] == kCcs811HwId) {
        address = addr;
        return true;
      }
    }
    Wire.setClock(100000);
    Wire.setTimeOut(20);
  }
  return false;
}

bool ExtendedSensors::begin(Type type, uint8_t address) {
  type_ = type;
  address_ = address;
  initialized_ = false;
  if (type == Type::kAs7341) return beginAs7341();
  if (type == Type::kVl53l1x) return beginVl53l1x();
  if (type == Type::kCcs811) return beginCcs811();
  return false;
}

EnvironmentalReading ExtendedSensors::read() {
  if (type_ == Type::kAs7341) return readAs7341();
  if (type_ == Type::kVl53l1x) return readVl53l1x();
  if (type_ == Type::kCcs811) return readCcs811();
  return EnvironmentalReading{};
}

EnvironmentalReading ExtendedSensors::readContinuous() {
  if (type_ == Type::kCcs811) return readCcs811();
  return read();
}

// --- AS7341 10-channel spectral sensor ---

bool ExtendedSensors::beginAs7341() {
  uint8_t id = 0;
  // Select register bank 0 (0x80..0xFD), power on oscillator (PON = 1).
  if (!writeReg8(address_, kAs7341RegCfg0, 0x00) ||
      !readReg8(address_, kAs7341RegId, &id, 1) ||
      (id >> 2) != kAs7341ChipId) {
    return false;
  }
  if (!writeReg8(address_, kAs7341RegEnable, 0x01)) return false;
  measurementWaitUs(2000);
  const uint8_t astepBytes[2] = {
      static_cast<uint8_t>(kAs7341Astep & 0xFFU),
      static_cast<uint8_t>(kAs7341Astep >> 8),
  };
  as7341Gain_ = 4;  // 8x
  if (!writeReg8(address_, kAs7341RegAtime, kAs7341Atime) ||
      !writeReg8Burst(address_, kAs7341RegAstepL, astepBytes, 2) ||
      !writeReg8(address_, kAs7341RegCfg1, as7341Gain_)) {
    return false;
  }
  uint8_t verifyGain = 0;
  if (!readReg8(address_, kAs7341RegCfg1, &verifyGain, 1) ||
      (verifyGain & 0x0FU) != as7341Gain_) {
    return false;
  }
  initialized_ = true;
  return true;
}

bool ExtendedSensors::as7341LoadSmux(const uint8_t table[20]) {
  // Disable SP_EN while loading SMUX RAM (PON=1, SP_EN=0).
  if (!writeReg8(address_, kAs7341RegEnable, 0x01) ||
      !writeReg8(address_, kAs7341RegCfg6, 0x10) ||  // SMUX_CMD = Write
      !writeReg8Burst(address_, 0x00, table, 20) ||
      !writeReg8(address_, kAs7341RegEnable, 0x11)) {  // PON | SMUXEN
    return false;
  }
  const uint32_t started = millis();
  uint8_t enable = 0x11;
  while ((enable & 0x10U) != 0U) {
    if (millis() - started >= 50U || measurementBudgetExpired()) return false;
    measurementWaitUs(1000);
    if (!readReg8(address_, kAs7341RegEnable, &enable, 1)) return false;
  }
  return true;
}

bool ExtendedSensors::as7341MeasurePass(const uint8_t table[20], uint16_t out[6],
                                        bool& saturated) {
  if (!as7341LoadSmux(table)) return false;
  // Start spectral measurement (PON=1 | SP_EN=1 -> 0x03).
  if (!writeReg8(address_, kAs7341RegEnable, 0x03)) return false;
  measurementWaitUs(static_cast<uint32_t>(kAs7341IntegrationMs) * 1000UL);
  const uint32_t started = millis();
  uint8_t status2 = 0;
  for (;;) {
    if (!readReg8(address_, kAs7341RegStatus2, &status2, 1)) return false;
    if ((status2 & 0x40U) != 0U) break;  // AVALID
    if (millis() - started >= 150U || measurementBudgetExpired()) return false;
    measurementWaitUs(2000);
  }
  uint8_t raw[12]{};
  if (!readReg8(address_, kAs7341RegCh0DataL, raw, sizeof(raw))) return false;
  for (uint8_t i = 0; i < 6; ++i) out[i] = little16(raw + i * 2);
  saturated = (status2 & 0x18U) != 0U;  // ASAT_DIGITAL | ASAT_ANALOG
  return writeReg8(address_, kAs7341RegEnable, 0x01);  // Stop SP_EN
}

EnvironmentalReading ExtendedSensors::readAs7341() {
  EnvironmentalReading reading{};
  reading.sensorType = Type::kAs7341;
  if (!initialized_) return reading;

  // Ensure the internal oscillator is powered and settled before SMUX loads
  // (needed on repeated/continuous captures after the prior power-down).
  if (!writeReg8(address_, kAs7341RegEnable, 0x01)) return reading;
  measurementWaitUs(2000);

  uint16_t low[6]{}, high[6]{};
  uint16_t finalPeak = 0;
  bool satLow = false, satHigh = false;
  bool gainAdjusted = false;

  for (uint8_t pass = 0; pass < 2; ++pass) {
    if (!as7341MeasurePass(kAs7341SmuxLow, low, satLow) ||
        !as7341MeasurePass(kAs7341SmuxHigh, high, satHigh)) {
      (void)writeReg8(address_, kAs7341RegEnable, 0x00);
      return reading;
    }
    uint16_t peak = 0;
    for (uint8_t i = 0; i < 6; ++i) {
      if (low[i] > peak) peak = low[i];
      if (high[i] > peak) peak = high[i];
    }
    finalPeak = peak;
    const bool anySat = satLow || satHigh || peak >= kAs7341FullScale - 1U;
    if (pass == 0 && anySat && as7341Gain_ > 0 && !measurementBudgetExpired()) {
      const uint8_t nextGain =
          as7341Gain_ >= 2 ? static_cast<uint8_t>(as7341Gain_ - 2) : 0;
      if (!writeReg8(address_, kAs7341RegCfg1, nextGain)) break;
      as7341Gain_ = nextGain;
      gainAdjusted = true;
      continue;
    }
    if (pass == 0 && peak < 256U && as7341Gain_ < 9 && !measurementBudgetExpired()) {
      uint8_t nextGain = static_cast<uint8_t>(as7341Gain_ + 2);
      if (nextGain > 9) nextGain = 9;
      if (!writeReg8(address_, kAs7341RegCfg1, nextGain)) break;
      as7341Gain_ = nextGain;
      gainAdjusted = true;
      continue;
    }
    break;
  }

  // Power down oscillator after capture.
  (void)writeReg8(address_, kAs7341RegEnable, 0x00);

  auto& sp = reading.extended.spectral;
  // F1..F4 from low pass ADC0..ADC3, F5..F8 from high pass ADC0..ADC3,
  // Clear and NIR averaged across both passes (integer midpoint).
  sp.channels[0] = low[0];
  sp.channels[1] = low[1];
  sp.channels[2] = low[2];
  sp.channels[3] = low[3];
  sp.channels[4] = high[0];
  sp.channels[5] = high[1];
  sp.channels[6] = high[2];
  sp.channels[7] = high[3];
  sp.channels[8] = static_cast<uint16_t>((static_cast<uint32_t>(low[4]) + high[4] + 1U) >> 1);
  sp.channels[9] = static_cast<uint16_t>((static_cast<uint32_t>(low[5]) + high[5] + 1U) >> 1);
  sp.integrationMs = kAs7341IntegrationMs;
  sp.gainCode = as7341Gain_;
  const bool finalSaturated =
      satLow || satHigh || finalPeak >= kAs7341FullScale - 1U;
  sp.status = (finalSaturated ? 1U : 0U) | (gainAdjusted ? 2U : 0U);
  reading.valid = true;
  reading.capabilities = lil::protocol::kSpectral;
  return reading;
}

// --- TOF400C-VL53L1X 4 m Time-of-Flight sensor ---

bool ExtendedSensors::beginVl53l1x() {
  uint8_t ids[2]{};
  if (!readReg16(address_, kVl53l1xRegModelId, ids, 2) ||
      ids[0] != 0xEA || ids[1] != 0xCC) {
    return false;
  }
  // Soft reset and wait for firmware boot state (0x00E5 bit 0 == 1).
  if (!writeReg16(address_, kVl53l1xRegSoftReset, 0x00)) return false;
  measurementWaitUs(1000);
  if (!writeReg16(address_, kVl53l1xRegSoftReset, 0x01)) return false;
  const uint32_t started = millis();
  uint8_t bootState = 0;
  while ((bootState & 0x01U) == 0U) {
    if (millis() - started >= 100U || measurementBudgetExpired()) return false;
    measurementWaitUs(2000);
    if (!readReg16(address_, kVl53l1xRegFirmwareStatus, &bootState, 1)) return false;
  }
  // Write the 91-byte ST ULD default configuration in 16-byte chunks
  // (well within ESP32 128-byte Wire TX buffer).
  for (uint8_t offset = 0; offset < sizeof(kVl53l1xDefaultConfig); offset += 16) {
    const uint8_t chunk = (sizeof(kVl53l1xDefaultConfig) - offset > 16U)
                              ? 16U
                              : static_cast<uint8_t>(sizeof(kVl53l1xDefaultConfig) - offset);
    if (!writeReg16Burst(address_,
                         static_cast<uint16_t>(kVl53l1xRegDefaultBase + offset),
                         kVl53l1xDefaultConfig + offset, chunk)) {
      return false;
    }
  }
  // Perform mandatory initial VHV calibration cycle per ST ULD SensorInit.
  if (!writeReg16(address_, kVl53l1xRegSystemModeStart, 0x40)) return false;
  if (!vl53l1xWaitDataReady(250U)) {
    (void)writeReg16(address_, kVl53l1xRegSystemModeStart, 0x00);
    return false;
  }
  if (!writeReg16(address_, kVl53l1xRegSystemInterruptClear, 0x01) ||
      !writeReg16(address_, kVl53l1xRegSystemModeStart, 0x00)) {
    return false;
  }
  // Enable VHV auto-recalibration (0x0008 = 0x09) and reset VHV init (0x000B = 0x00).
  if (!writeReg16(address_, 0x0008, 0x09) || !writeReg16(address_, 0x000B, 0x00)) {
    return false;
  }
  initialized_ = true;
  return true;
}

bool ExtendedSensors::vl53l1xWaitDataReady(uint32_t timeoutMs) {
  uint8_t muxCtrl = 0;
  if (!readReg16(address_, kVl53l1xRegGpioHvMuxCtrl, &muxCtrl, 1)) return false;
  const uint8_t expectedPolarity = ((muxCtrl & 0x10U) == 0U) ? 1U : 0U;
  const uint32_t started = millis();
  for (;;) {
    uint8_t status = 0;
    if (!readReg16(address_, kVl53l1xRegGpioTioHvStatus, &status, 1)) return false;
    if ((status & 0x01U) == expectedPolarity) return true;
    if (millis() - started >= timeoutMs || measurementBudgetExpired()) return false;
    measurementWaitUs(5000);
  }
}

EnvironmentalReading ExtendedSensors::readVl53l1x() {
  EnvironmentalReading reading{};
  reading.sensorType = Type::kVl53l1x;
  if (!initialized_) return reading;

  // Start ranging (Long Mode 4m, 100 ms timing budget from default config).
  if (!writeReg16(address_, kVl53l1xRegSystemInterruptClear, 0x01) ||
      !writeReg16(address_, kVl53l1xRegSystemModeStart, 0x40)) {
    return reading;
  }
  if (!vl53l1xWaitDataReady(300U)) {
    (void)writeReg16(address_, kVl53l1xRegSystemModeStart, 0x00);
    return reading;
  }
  uint8_t block[17]{};
  const bool readOk = readReg16(address_, kVl53l1xRegResultBase, block, sizeof(block));
  (void)writeReg16(address_, kVl53l1xRegSystemInterruptClear, 0x01);
  (void)writeReg16(address_, kVl53l1xRegSystemModeStart, 0x00);
  if (!readOk) return reading;

  // ST VL53L1X ULD result block starting at 0x0089 (RESULT__RANGE_STATUS):
  //   [0]      0x0089  RESULT__RANGE_STATUS
  //   [3..4]   0x008C  RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0
  //   [5..6]   0x008E  RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0
  //   [7..8]   0x0090  RESULT__AMBIENT_COUNT_RATE_MCPS_SD0
  //   [13..14] 0x0096  RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0
  //   [15..16] 0x0098  RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0
  // Count rates are 9.7 fixed-point MCPS (~7.8125 kcps/LSB, scaled * 8 per ST ULD).
  const uint8_t rawStatus = block[0] & 0x1FU;
  const uint8_t rangeStatus = rawStatus < sizeof(kVl53l1xStatusLut)
                                  ? kVl53l1xStatusLut[rawStatus]
                                  : 255U;
  const uint16_t rawSignal = big16(block + 15) != 0U ? big16(block + 15) : big16(block + 5);
  const uint32_t signalKcps = static_cast<uint32_t>(rawSignal) * 8UL;
  const uint32_t ambientKcps = static_cast<uint32_t>(big16(block + 7)) * 8UL;
  const uint16_t signalRate =
      signalKcps > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(signalKcps);
  const uint16_t ambientRate =
      ambientKcps > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(ambientKcps);
  const uint16_t distanceMm = big16(block + 13);

  auto& dist = reading.extended.distance;
  dist.distanceMm = distanceMm;
  dist.signalRateKcps = signalRate;
  dist.ambientRateKcps = ambientRate;
  dist.rangeStatus = rangeStatus;
  dist.mode = 2;  // Long mode (up to 4000 mm)

  // Accept only valid target range measurements (ULD status 0) within the
  // sensor's physical ceiling (<= 4500 mm); non-zero status codes remain in
  // dist.rangeStatus for diagnostics while marking the reading invalid.
  reading.valid = (rangeStatus == 0 && distanceMm <= 4500U);
  if (reading.valid) {
    reading.capabilities = lil::protocol::kDistance;
  }
  return reading;
}

// --- CCS811 eCO2 / TVOC gas sensor ---

void ExtendedSensors::clearCcs811Baseline() {
  for (auto& entry : rtcCcs811Baselines) entry = {};
}

bool ExtendedSensors::ccs811ReadReg(uint8_t address, uint8_t reg, uint8_t* data, uint8_t count) {
  // Per ams DS000459 Fig. 12 and AN000369, CCS811 supports STOP between the
  // mailbox register write and the read transfer. Avoiding Repeated-START
  // prevents ESP32-C3 I2C clock-stretching timeouts in application mode.
  for (uint8_t attempt = 0; attempt < 2; ++attempt) {
    if (measurementBudgetExpired()) return false;
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(true) == 0 && Wire.requestFrom(address, count) == count) {
      for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
      return true;
    }
    if (attempt == 0) measurementWaitUs(2000);
  }
  return false;
}

namespace {
bool ccs811WriteBurst(uint8_t address, uint8_t reg, const uint8_t* data, uint8_t count) {
  for (uint8_t attempt = 0; attempt < 2; ++attempt) {
    if (measurementBudgetExpired()) return false;
    Wire.beginTransmission(address);
    Wire.write(reg);
    for (uint8_t i = 0; i < count; ++i) Wire.write(data[i]);
    if (Wire.endTransmission(true) == 0) return true;
    if (attempt == 0) measurementWaitUs(2000);
  }
  return false;
}
}  // namespace

bool ExtendedSensors::ccs811SoftReset(uint8_t address) {
  static constexpr uint8_t kResetSeq[4] = {0x11, 0xE5, 0x72, 0x8A};
  if (!writeReg8Burst(address, kCcs811RegSwReset, kResetSeq, sizeof(kResetSeq))) {
    return false;
  }
  measurementWaitUs(25000);  // DS000459 t_RESET + bootloader ready margin.
  return true;
}

struct Ccs811I2cGuard {
  Ccs811I2cGuard() {
#if defined(ESP_PLATFORM)
    pushLowPowerWaitInhibit();
#endif
  }
  ~Ccs811I2cGuard() {
#if defined(ESP_PLATFORM)
    popLowPowerWaitInhibit();
#endif
  }
};

bool ExtendedSensors::beginCcs811() {
  // Keep the ESP32-C3 I2C peripheral active across CCS811 clock-stretching
  // waits; entering light sleep mid-session disrupts 50 kHz Wire state.
  Ccs811I2cGuard i2cGuard;
  // CCS811 relies heavily on I2C clock stretching (DS000459 §6).
  // Use 50 kHz and a 100 ms bus timeout for reliable ESP32-C3 communication.
  Wire.setClock(50000);
  Wire.setTimeOut(100);

  uint8_t hwId = 0;
  if (!ccs811ReadReg(address_, kCcs811RegHwId, &hwId, 1) || hwId != kCcs811HwId) {
    return false;
  }
  uint8_t status = 0;
  if (!ccs811ReadReg(address_, kCcs811RegStatus, &status, 1)) return false;
  if ((status & 0x10U) == 0U) {  // APP_VALID must be set
    if (!ccs811SoftReset(address_) ||
        !ccs811ReadReg(address_, kCcs811RegStatus, &status, 1) ||
        (status & 0x10U) == 0U) {
      SENSOR_LOG_PRINTLN("[CCS811] No valid application firmware");
      return false;
    }
  }
  if ((status & 0x80U) == 0U) {  // Still in bootloader -> issue APP_START (0xF4)
    bool startedApp = false;
    for (uint8_t attempt = 0; attempt < 2 && !startedApp; ++attempt) {
      if (attempt == 1 && !ccs811SoftReset(address_)) return false;
      Wire.beginTransmission(address_);
      Wire.write(kCcs811RegAppStart);
      if (Wire.endTransmission(true) != 0) continue;
      for (uint8_t wait = 0; wait < 5; ++wait) {
        measurementWaitUs(5000);
        if (ccs811ReadReg(address_, kCcs811RegStatus, &status, 1) &&
            (status & kCcsAppRunning) == kCcsAppRunning) {
          startedApp = true;
          break;
        }
      }
    }
    if (!startedApp) return false;
  }
  uint8_t mode = 0;
  if (!ccs811ReadReg(address_, kCcs811RegMeasMode, &mode, 1)) return false;
  ccsDriveMode_ = mode & 0x70U;
  ccsCooling_ = false;
  ccsConditioned_ = false;
  ccsModeStartedMs_ = ccsLastReadyMs_ = ccsLastSaveMs_ = millis();
  initialized_ = true;
  return true;
}

bool ExtendedSensors::resetCcs811() {
  initialized_ = false;
  return ccs811SoftReset(address_) && beginCcs811();
}

bool ExtendedSensors::setCcs811Interval(uint32_t reportSeconds) {
  if (!initialized_ || type_ != Type::kCcs811) return false;
  // Select the lowest-power drive that can supply a fresh value per report.
  const uint8_t requested = reportSeconds >= 60 ? 0x30 : reportSeconds >= 10 ? 0x20 : 0x10;
  if (requested == ccsDriveMode_) return true;
  if (ccsCooling_ && millis() - ccsModeStartedMs_ < kCcsCoolingMs) return true;

  // DS000459 requires ten minutes in idle before slowing the sample rate.
  const bool cooling = ccsDriveMode_ != 0 &&
      (ccsDriveMode_ == 0x40 || requested > ccsDriveMode_);
  const uint8_t next = cooling ? 0 : requested;
  uint8_t verified = 0;
  if (!ccs811WriteBurst(address_, kCcs811RegMeasMode, &next, 1) ||
      !ccs811ReadReg(address_, kCcs811RegMeasMode, &verified, 1) ||
      (verified & 0x70U) != next) return false;
  ccsDriveMode_ = next;
  ccsCooling_ = cooling;
  ccsConditioned_ = false;
  ccsModeStartedMs_ = ccsLastReadyMs_ = ccsLastSaveMs_ = millis();
  return true;
}

EnvironmentalReading ExtendedSensors::readCcs811() {
  EnvironmentalReading reading{};
  reading.sensorType = Type::kCcs811;
  if (!initialized_) return reading;
  auto& aq = reading.extended.airQuality;
  uint8_t status = 0;
  if (!ccs811ReadReg(address_, kCcs811RegStatus, &status, 1)) return reading;
  aq.status = status;
  if ((status & kCcsError) != 0) {
    // Consume any affected frame before clearing its error. Otherwise a
    // second read could see DATA_READY with the same frame and a cleared ERROR.
    if ((status & (kCcsAppRunning | kCcsDataReady)) == (kCcsAppRunning | kCcsDataReady)) {
      uint8_t discarded[8];
      if (!ccs811ReadReg(address_, kCcs811RegAlgResultData, discarded, sizeof(discarded))) return reading;
      aq.errorId = discarded[5];
    }
    // Only an explicit ERROR_ID read clears the latched error.
    uint8_t error = 0;
    if (ccs811ReadReg(address_, kCcs811RegErrorId, &error, 1)) aq.errorId |= error;
    return reading;
  }
  if ((status & kCcsAppRunning) != kCcsAppRunning) {
    // A brownout may return the powered module to its bootloader. Restore
    // the application once; the next service call selects the report mode.
    (void)beginCcs811();
    return reading;
  }
  if (ccsDriveMode_ == 0) {
    // Discovery only identifies the module. Paired nodes can also be cooling.
    reading.valid = true;
    return reading;
  }
  if (ccsDriveMode_ > 0x30) return reading;
  const uint32_t now = millis();
  const uint32_t periodMs = ccsDriveMode_ == 0x30 ? 60000 : ccsDriveMode_ == 0x20 ? 10000 : 1000;
  const uint32_t modeAgeMs = now - ccsModeStartedMs_;
  if (!ccsConditioned_ && modeAgeMs < kCcsConditioningMs) {
    reading.iaqCalibrationPhase = lil::protocol::IaqCalibrationPhase::kStabilizing;
    reading.iaqCalibrationRemainingMinutes = (kCcsConditioningMs - modeAgeMs + 59999) / 60000;
  }
  if ((status & kCcsDataReady) == 0) {
    // Allow for the sensor oscillator tolerance. A persistently stuck 0x90
    // still reaches the existing fault and recovery path.
    reading.valid = now - ccsLastReadyMs_ <= periodMs + periodMs / 10;
    return reading;
  }
  uint8_t raw[8]{};
  if (!ccs811ReadReg(address_, kCcs811RegAlgResultData, raw, sizeof(raw))) return reading;
  aq.status = raw[4];
  if ((raw[4] & kCcsError) != 0) {
    aq.errorId = raw[5];
    uint8_t error = 0;
    if (ccs811ReadReg(address_, kCcs811RegErrorId, &error, 1)) aq.errorId |= error;
    return reading;
  }
  if ((raw[4] & (kCcsAppRunning | kCcsDataReady)) != (kCcsAppRunning | kCcsDataReady)) return reading;
  ccsLastReadyMs_ = now;
  auto& baseline = rtcCcs811Baselines[(ccsDriveMode_ >> 4) - 1];
  if (!ccsConditioned_) {
    if (modeAgeMs < kCcsConditioningMs) {
      reading.valid = true;
      return reading;
    }
    const bool restore = baseline.address == address_ &&
        logicalTimeMs() - baseline.savedMs <= 7ULL * kCcsDayMs;
    if (restore && !ccs811WriteBurst(address_, kCcs811RegBaseline, baseline.bytes, 2)) return reading;
    ccsConditioned_ = true;
    if (restore) {
      // The frame just consumed predates the baseline write.
      reading.valid = true;
      return reading;
    }
  }
  const uint16_t eco2 = big16(raw);
  const uint16_t tvoc = big16(raw + 2);
  if (eco2 < 400 || eco2 > 29206 || tvoc > 32768) return reading;
  aq.eco2Ppm = eco2;
  aq.tvocPpb = tvoc;
  aq.rawCurrentUa = raw[6] >> 2;
  aq.rawAdc = (static_cast<uint16_t>(raw[6] & 3U) << 8) | raw[7];
  reading.capabilities = lil::protocol::kAirQualityGas;
  reading.valid = true;

  if (now - ccsLastSaveMs_ >= kCcsDayMs) {
    uint8_t bytes[2];
    if (ccs811ReadReg(address_, kCcs811RegBaseline, bytes, 2)) {
      baseline = {logicalTimeMs(), address_, {bytes[0], bytes[1]}};
      ccsLastSaveMs_ = now;
    }
  }
  return reading;
}

}  // namespace sensor

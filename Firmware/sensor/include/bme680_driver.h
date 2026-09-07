#pragma once

#include <Wire.h>
#include <bme68x.h>
#include <bsec2.h>

#include "bsec_state_store.h"
#include "environmental_reading.h"

namespace sensor {

class Bme680Driver {
 public:
  bool begin(uint8_t address, float temperatureOffsetC);
  EnvironmentalReading read();
  void prepareForDeepSleep(uint32_t seconds);
  uint32_t recommendedSleepSeconds(uint32_t fallbackSeconds) const;
  void clearPersistentState();

 private:
  struct RawI2cContext {
    TwoWire* wire = nullptr;
    uint8_t address = 0;
  };

  static int8_t rawI2cRead(uint8_t registerAddress, uint8_t* data,
                           uint32_t length, void* context);
  static int8_t rawI2cWrite(uint8_t registerAddress, const uint8_t* data,
                            uint32_t length, void* context);
  static void rawDelayUs(uint32_t periodUs, void* context);
  bool beginRaw(uint8_t address);
  EnvironmentalReading readBsec();
  EnvironmentalReading readRaw();
  bool tryAddress(uint8_t address);
  void seedClockOverflowCounter();
  void restoreState();
  void saveState(uint8_t iaqAccuracy);
  void saveCalibrationMetadata();
  bool ensureStateStore();
  void updateCalibrationStatus(EnvironmentalReading& reading);
  uint32_t currentCalibrationElapsedSeconds() const;

  Bsec2 bsec_;
  uint8_t bsecMemory_[BSEC_INSTANCE_SIZE]{};
  RawI2cContext rawContext_{};
  bme68x_dev rawDevice_{};
  bme68x_conf rawConfiguration_{};
  bme68x_heatr_conf rawHeater_{};
  BsecStateStore stateStore_;
  float temperatureOffsetC_ = 0.0F;
  bool rawReady_ = false;
  bool bsecReady_ = false;
  bool stateStoreReady_ = false;
  bool scheduleUpdated_ = false;
  uint8_t address_ = 0;
  uint64_t calibrationAwakeStartedMs_ = 0;
  uint32_t calibrationElapsedAtBeginSeconds_ = 0;
};

}  // namespace sensor

#pragma once

#include <Preferences.h>
#include <stddef.h>
#include <stdint.h>

namespace sensor {

class BsecStateStore {
 public:
  bool begin();
  bool load(uint8_t* state, size_t length);
  bool save(const uint8_t* state, size_t length);
  bool loadCalibration(uint32_t& elapsedSeconds, bool& calibrationReady);
  bool saveCalibration(uint32_t elapsedSeconds, bool calibrationReady);
  bool clear();

 private:
  Preferences preferences_;
};

}  // namespace sensor

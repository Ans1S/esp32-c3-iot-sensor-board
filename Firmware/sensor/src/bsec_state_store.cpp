#include "bsec_state_store.h"

#include <string.h>

#include "lil_protocol.h"

namespace sensor {

namespace {
constexpr char kNamespace[] = "wcharger_iaq";
constexpr char kStateKey[] = "bsec";
constexpr char kCalibrationKey[] = "cal";
// Revision E intentionally invalidates every state learned by the former LP
// commissioning path. ULP-only operation must start with a matching baseline.
constexpr uint32_t kStateMagic = 0x42534545UL;  // "BSEE"
constexpr size_t kMaxStateSize = 256;

struct StoredState {
  uint32_t magic = kStateMagic;
  uint16_t length = 0;
  uint16_t reserved = 0;
  uint32_t crc = 0;
  uint8_t data[kMaxStateSize]{};
};

struct StoredCalibration {
  uint32_t magic = 0x49415132UL;  // "IAQ2"
  uint32_t elapsedSeconds = 0;
  uint8_t calibrationReady = 0;
  uint8_t reserved[3]{};
  uint32_t crc = 0;
};

uint32_t calibrationCrc(const StoredCalibration& stored) {
  return lil::protocol::crc32(
      reinterpret_cast<const uint8_t*>(&stored),
      offsetof(StoredCalibration, crc));
}

}  // namespace

bool BsecStateStore::begin() {
  return preferences_.begin(kNamespace, false);
}

bool BsecStateStore::load(uint8_t* state, size_t length) {
  if (state == nullptr || length == 0 || length > kMaxStateSize ||
      preferences_.getBytesLength(kStateKey) != sizeof(StoredState)) {
    return false;
  }
  StoredState stored{};
  if (preferences_.getBytes(kStateKey, &stored, sizeof(stored)) !=
          sizeof(stored) ||
      stored.magic != kStateMagic || stored.length != length ||
      stored.crc != lil::protocol::crc32(stored.data, stored.length)) {
    return false;
  }
  memcpy(state, stored.data, length);
  return true;
}

bool BsecStateStore::save(const uint8_t* state, size_t length) {
  if (state == nullptr || length == 0 || length > kMaxStateSize) {
    return false;
  }
  StoredState stored{};
  stored.length = static_cast<uint16_t>(length);
  memcpy(stored.data, state, length);
  stored.crc = lil::protocol::crc32(stored.data, length);
  return preferences_.putBytes(kStateKey, &stored, sizeof(stored)) ==
         sizeof(stored);
}

bool BsecStateStore::loadCalibration(uint32_t& elapsedSeconds,
                                     bool& calibrationReady) {
  if (preferences_.getBytesLength(kCalibrationKey) !=
      sizeof(StoredCalibration)) {
    return false;
  }
  StoredCalibration stored{};
  if (preferences_.getBytes(kCalibrationKey, &stored, sizeof(stored)) !=
      sizeof(stored) ||
      stored.magic != StoredCalibration{}.magic ||
      stored.calibrationReady > 1 ||
      stored.crc != calibrationCrc(stored)) {
    return false;
  }
  elapsedSeconds = stored.elapsedSeconds;
  calibrationReady = stored.calibrationReady != 0;
  return true;
}

bool BsecStateStore::saveCalibration(uint32_t elapsedSeconds,
                                     bool calibrationReady) {
  StoredCalibration stored{};
  stored.elapsedSeconds = elapsedSeconds;
  stored.calibrationReady = calibrationReady ? 1 : 0;
  stored.crc = calibrationCrc(stored);
  return preferences_.putBytes(kCalibrationKey, &stored, sizeof(stored)) ==
         sizeof(stored);
}

bool BsecStateStore::clear() {
  return preferences_.clear();
}

}  // namespace sensor

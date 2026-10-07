#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>

namespace station {
constexpr size_t kMaxSensorIdentities = 64;
struct MotionReference {
  bool enabled = false;
  float accelerationG[3]{};
  float angularRateDps[3]{};
};
struct SensorIdentity {
  uint8_t mac[6]{};
  char name[25]{};
  MotionReference motion{};
};
inline bool validMotionReference(const MotionReference& reference) {
  for (size_t axis = 0; axis < 3; ++axis) {
    if (!isfinite(reference.accelerationG[axis]) || !isfinite(reference.angularRateDps[axis]) ||
        fabsf(reference.accelerationG[axis]) > 4.0F || fabsf(reference.angularRateDps[axis]) > 5.0F) return false;
  }
  const float* a = reference.accelerationG;
  const float magnitude = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
  return magnitude >= 0.8F && magnitude <= 1.2F;
}
}  // namespace station

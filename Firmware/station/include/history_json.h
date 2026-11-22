#pragma once

#include <ArduinoJson.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "paged_history.h"

namespace station {

// Fast single-precision float-to-ASCII and integer formatters tuned for
// Xtensa LX7 (ESP32-S3): 2-digit LUT avoids 64-bit __udivdi3 and runtime
// divisions; FPU rounding avoids libm lroundf/vfprintf overhead.
inline constexpr char kDigits200[201] =
    "0001020304050607080910111213141516171819"
    "2021222324252627282930313233343536373839"
    "4041424344454647484950515253545556575859"
    "6061626364656667686970717273747576777879"
    "8081828384858687888990919293949596979899";

inline bool appendRawBytes(char* dst, size_t cap, size_t& len,
                           const char* src, size_t n) {
  if (len >= cap || n >= cap - len) return false;
  memcpy(dst + len, src, n);
  len += n;
  return true;
}

template <size_t N>
inline bool appendLit(char* dst, size_t cap, size_t& len, const char (&lit)[N]) {
  return appendRawBytes(dst, cap, len, lit, N - 1);
}

inline bool appendRawStr(char* dst, size_t cap, size_t& len, const char* s) {
  return appendRawBytes(dst, cap, len, s, strlen(s));
}

inline bool appendRawChar(char* dst, size_t cap, size_t& len, char c) {
  if (len + 1 >= cap) return false;
  dst[len++] = c;
  return true;
}

#if defined(__GNUC__) && !defined(__clang__)
[[gnu::optimize("O2")]]
#endif
inline bool appendU32(char* dst, size_t cap, size_t& len, uint32_t v) {
  char tmp[12];
  size_t pos = sizeof(tmp);
  while (v >= 100U) {
    const uint32_t q = v / 100U;
    const uint32_t r = (v - q * 100U) * 2U;
    v = q;
    tmp[--pos] = kDigits200[r + 1U];
    tmp[--pos] = kDigits200[r];
  }
  if (v >= 10U) {
    const uint32_t r = v * 2U;
    tmp[--pos] = kDigits200[r + 1U];
    tmp[--pos] = kDigits200[r];
  } else {
    tmp[--pos] = static_cast<char>('0' + v);
  }
  return appendRawBytes(dst, cap, len, tmp + pos, sizeof(tmp) - pos);
}

inline bool appendUnsigned(char* dst, size_t cap, size_t& len, uint64_t v) {
  if (v <= UINT32_MAX) return appendU32(dst, cap, len, static_cast<uint32_t>(v));
  char tmp[24];
  size_t n = 0;
  do {
    tmp[n++] = static_cast<char>('0' + (v % 10ULL));
    v /= 10ULL;
  } while (v != 0ULL);
  if (len >= cap || n >= cap - len) return false;
  while (n > 0) dst[len++] = tmp[--n];
  return true;
}

inline bool appendSigned(char* dst, size_t cap, size_t& len, int64_t v) {
  if (v < 0) {
    if (!appendRawChar(dst, cap, len, '-')) return false;
    const uint64_t mag = static_cast<uint64_t>(-(v + 1)) + 1ULL;
    return appendUnsigned(dst, cap, len, mag);
  }
  return appendUnsigned(dst, cap, len, static_cast<uint64_t>(v));
}

// Exact integer formatter for milli-scaled unsigned quantities (e.g. millivolts
// -> volts) without floating-point division or __divsf3 on Xtensa LX7.
#if defined(__GNUC__) && !defined(__clang__)
[[gnu::optimize("O2")]]
#endif
inline bool appendMilliU32(char* dst, size_t cap, size_t& len, uint32_t milli) {
  const uint32_t whole = milli / 1000U;
  const uint32_t frac = milli - whole * 1000U;
  if (!appendU32(dst, cap, len, whole)) return false;
  if (frac == 0U) return true;
  if (!appendRawChar(dst, cap, len, '.')) return false;
  const uint32_t hi = frac / 100U;
  const uint32_t lo = frac - hi * 100U;
  char fracBuf[3];
  fracBuf[0] = static_cast<char>('0' + hi);
  fracBuf[1] = kDigits200[lo * 2U];
  fracBuf[2] = kDigits200[lo * 2U + 1U];
  size_t fracLen = 3;
  while (fracLen > 0 && fracBuf[fracLen - 1] == '0') --fracLen;
  return appendRawBytes(dst, cap, len, fracBuf, fracLen);
}

#if defined(__GNUC__) && !defined(__clang__)
[[gnu::noinline]]
#endif
inline bool appendScaledFloatLarge(char* dst, size_t cap, size_t& len,
                                   float v, uint8_t d) {
  static constexpr double kPow10D[] = {1.0, 10.0, 100.0, 1000.0, 10000.0, 100000.0, 1000000.0};
  static constexpr uint32_t kDiv10[] = {1U, 10U, 100U, 1000U, 10000U, 100000U, 1000000U};
  const double clamped = v < -4.294967e9F
                             ? -4.294967e9
                             : (v > 4.294967e9F ? 4.294967e9 : static_cast<double>(v));
  const double x = clamped * kPow10D[d];
  const int64_t scaled = static_cast<int64_t>(x >= 0.0 ? (x + 0.5) : (x - 0.5));
  if (scaled < 0 && !appendRawChar(dst, cap, len, '-')) return false;
  const uint64_t mag = static_cast<uint64_t>(scaled < 0 ? -(scaled + 1) + 1 : scaled);
  const uint32_t div = kDiv10[d];
  const uint64_t whole = mag / div;
  const uint32_t frac = static_cast<uint32_t>(mag - whole * div);
  if (!appendUnsigned(dst, cap, len, whole)) return false;
  if (d == 0 || frac == 0) return true;
  if (!appendRawChar(dst, cap, len, '.')) return false;
  char fracBuf[6];
  uint32_t rem = frac;
  for (int i = static_cast<int>(d) - 1; i >= 0; --i) {
    fracBuf[i] = static_cast<char>('0' + (rem % 10U));
    rem /= 10U;
  }
  size_t fracLen = d;
  while (fracLen > 0 && fracBuf[fracLen - 1] == '0') --fracLen;
  return appendRawBytes(dst, cap, len, fracBuf, fracLen);
}

#if defined(__GNUC__) && !defined(__clang__)
[[gnu::optimize("O2")]]
#endif
inline bool appendScaledFloat(char* dst, size_t cap, size_t& len,
                              float v, uint8_t maxDecimals = 4) {
  if (!isfinite(v)) return appendLit(dst, cap, len, "null");
  const uint8_t d = maxDecimals > 6 ? 6 : maxDecimals;
  // Pure single-precision FPU + 32-bit constant-divisor fast path covers all
  // normal sensor measurements (TMP117 <= 2000 C, LSM6DSOX <= 32 g at d=6,
  // and all environmental/gyro values <= 200000 at d<=4) with zero double
  // soft-float or 64-bit __udivdi3 calls on Xtensa LX7.
  const float fastLimit = d == 6 ? 2000.0F : (d == 5 ? 20000.0F : 200000.0F);
  if (v < -fastLimit || v > fastLimit) {
    return appendScaledFloatLarge(dst, cap, len, v, d);
  }
  static constexpr float kPow10F[] = {1.0F, 10.0F, 100.0F, 1000.0F, 10000.0F, 100000.0F, 1000000.0F};
  const float x = v * kPow10F[d];
  const int32_t scaled = static_cast<int32_t>(x >= 0.0F ? (x + 0.5F) : (x - 0.5F));
  if (scaled < 0 && !appendRawChar(dst, cap, len, '-')) return false;
  const uint32_t mag = static_cast<uint32_t>(scaled < 0 ? -scaled : scaled);
  uint32_t whole = mag;
  uint32_t frac = 0;
  // Compile-time constant divisors emit Xtensa muluh + shift instead of quou.
  if (d == 6) {
    whole = mag / 1000000U;
    frac = mag - whole * 1000000U;
  } else if (d == 5) {
    whole = mag / 100000U;
    frac = mag - whole * 100000U;
  } else if (d == 4) {
    whole = mag / 10000U;
    frac = mag - whole * 10000U;
  } else if (d == 3) {
    whole = mag / 1000U;
    frac = mag - whole * 1000U;
  } else if (d == 2) {
    whole = mag / 100U;
    frac = mag - whole * 100U;
  } else if (d == 1) {
    whole = mag / 10U;
    frac = mag - whole * 10U;
  }
  if (!appendU32(dst, cap, len, whole)) return false;
  if (d == 0 || frac == 0) return true;
  if (!appendRawChar(dst, cap, len, '.')) return false;
  char fracBuf[6];
  if (d == 6) {
    const uint32_t hi = frac / 10000U;
    const uint32_t rem = frac - hi * 10000U;
    const uint32_t mid = rem / 100U;
    const uint32_t lo = rem - mid * 100U;
    fracBuf[0] = kDigits200[hi * 2U];
    fracBuf[1] = kDigits200[hi * 2U + 1U];
    fracBuf[2] = kDigits200[mid * 2U];
    fracBuf[3] = kDigits200[mid * 2U + 1U];
    fracBuf[4] = kDigits200[lo * 2U];
    fracBuf[5] = kDigits200[lo * 2U + 1U];
  } else if (d == 5) {
    const uint32_t hi = frac / 10000U;
    const uint32_t rem = frac - hi * 10000U;
    const uint32_t mid = rem / 100U;
    const uint32_t lo = rem - mid * 100U;
    fracBuf[0] = static_cast<char>('0' + hi);
    fracBuf[1] = kDigits200[mid * 2U];
    fracBuf[2] = kDigits200[mid * 2U + 1U];
    fracBuf[3] = kDigits200[lo * 2U];
    fracBuf[4] = kDigits200[lo * 2U + 1U];
  } else if (d == 4) {
    const uint32_t hi = frac / 100U;
    const uint32_t lo = frac - hi * 100U;
    fracBuf[0] = kDigits200[hi * 2U];
    fracBuf[1] = kDigits200[hi * 2U + 1U];
    fracBuf[2] = kDigits200[lo * 2U];
    fracBuf[3] = kDigits200[lo * 2U + 1U];
  } else if (d == 3) {
    const uint32_t hi = frac / 100U;
    const uint32_t lo = frac - hi * 100U;
    fracBuf[0] = static_cast<char>('0' + hi);
    fracBuf[1] = kDigits200[lo * 2U];
    fracBuf[2] = kDigits200[lo * 2U + 1U];
  } else if (d == 2) {
    fracBuf[0] = kDigits200[frac * 2U];
    fracBuf[1] = kDigits200[frac * 2U + 1U];
  } else {
    fracBuf[0] = static_cast<char>('0' + frac);
  }
  size_t fracLen = d;
  while (fracLen > 0 && fracBuf[fracLen - 1] == '0') --fracLen;
  return appendRawBytes(dst, cap, len, fracBuf, fracLen);
}

// Stable, versioned column names. Null in an array means that the legacy
// object omitted that field; zero and false remain real measurements.
inline constexpr const char* kHistoryColumns[] = {
  "t", "sensorType", "pcbVersion", "iaqAccuracy", "gap", "temperature",
  "humidity", "pressure", "iaq", "gas", "battery", "ax", "ay", "az",
  "gx", "gy", "gz", "peakAcceleration", "peakAngularRate",
  "eco2", "tvoc", "distanceMm", "signalRate", "ambientRate", "rangeStatus",
  "spectral", "spectralGain", "spectralIntMs"
};
inline constexpr size_t kHistoryColumnCount = sizeof(kHistoryColumns) / sizeof(kHistoryColumns[0]);

inline uint32_t historyColumnMask(const NormalHistoryPoint* points, size_t count) {
  uint32_t mask = (1U << 0) | (1U << 1) | (1U << 3) | (1U << 4);
  for (size_t i = 0; i < count; ++i) {
    const auto& t = points[i].telemetry;
    if (t.pcbVersion) mask |= 1U << 2;
    if (t.capabilities & lil::protocol::kTemperature) mask |= 1U << 5;
    if (t.capabilities & lil::protocol::kHumidity) mask |= 1U << 6;
    if (t.capabilities & lil::protocol::kPressure) mask |= 1U << 7;
    if ((t.capabilities & lil::protocol::kIaq) && t.iaqAccuracy) mask |= 1U << 8;
    if (t.capabilities & lil::protocol::kGasResistance) mask |= 1U << 9;
    if (t.capabilities & lil::protocol::kBattery) mask |= 1U << 10;
    if (t.capabilities & lil::protocol::kMotion) mask |= 0xffU << 11;
    if (t.capabilities & lil::protocol::kAirQualityGas) mask |= (1U << 19) | (1U << 20);
    if (t.capabilities & lil::protocol::kDistance) mask |= (1U << 21) | (1U << 22) | (1U << 23) | (1U << 24);
    if (t.capabilities & lil::protocol::kSpectral) mask |= (1U << 25) | (1U << 26) | (1U << 27);
  }
  return mask;
}

inline void historyPointJson(JsonDocument& row, const NormalHistoryPoint& point) {
  const auto& t = point.telemetry;
  row["t"] = uint64_t(point.timestamp) * 1000;
  row["sensorType"] = static_cast<uint8_t>(t.sensorType);
  if (t.pcbVersion) row["pcbVersion"] = t.pcbVersion;
  row["iaqAccuracy"] = t.iaqAccuracy;
  row["gap"] = bool(t.live.flags & lil::protocol::kLiveGap) || t.motion.fifoOverrun;
  if (t.capabilities & lil::protocol::kTemperature) row["temperature"] = float(t.temperatureC);
  if (t.capabilities & lil::protocol::kHumidity) row["humidity"] = float(t.humidityPercent);
  if (t.capabilities & lil::protocol::kPressure) row["pressure"] = float(t.pressureHpa);
  if ((t.capabilities & lil::protocol::kIaq) && t.iaqAccuracy) row["iaq"] = float(t.iaq);
  if (t.capabilities & lil::protocol::kGasResistance) row["gas"] = t.gasResistanceOhms / 1000.0F;
  if (t.capabilities & lil::protocol::kBattery) row["battery"] = t.batteryMillivolts / 1000.0F;
  if (t.capabilities & lil::protocol::kMotion) {
    const char* keys[] = {"ax", "ay", "az", "gx", "gy", "gz"};
    for (size_t axis = 0; axis < 6; ++axis)
      row[keys[axis]] = float(axis < 3 ? t.motion.accelerationG[axis] : t.motion.angularRateDps[axis - 3]);
    row["peakAcceleration"] = float(t.motion.peakAccelerationG);
    row["peakAngularRate"] = float(t.motion.peakAngularRateDps);
  }
  if (t.capabilities & lil::protocol::kAirQualityGas) {
    row["eco2"] = uint16_t(t.extended.airQuality.eco2Ppm);
    row["tvoc"] = uint16_t(t.extended.airQuality.tvocPpb);
  }
  if (t.capabilities & lil::protocol::kDistance) {
    row["distanceMm"] = uint16_t(t.extended.distance.distanceMm);
    row["signalRate"] = uint16_t(t.extended.distance.signalRateKcps);
    row["ambientRate"] = uint16_t(t.extended.distance.ambientRateKcps);
    row["rangeStatus"] = uint8_t(t.extended.distance.rangeStatus);
  }
  if (t.capabilities & lil::protocol::kSpectral) {
    JsonArray sp = row["spectral"].to<JsonArray>();
    for (size_t ch = 0; ch < 10; ++ch) sp.add(uint16_t(t.extended.spectral.channels[ch]));
    row["spectralGain"] = uint8_t(t.extended.spectral.gainCode);
    row["spectralIntMs"] = uint16_t(t.extended.spectral.integrationMs);
  }
}

inline void compactHistoryPointJson(JsonDocument& result, const JsonDocument& row, uint32_t mask) {
  JsonArray values = result.to<JsonArray>();
  for (size_t i = 0; i < kHistoryColumnCount; ++i)
    if (mask & (1UL << i)) values.add(row[kHistoryColumns[i]]);
}

// Direct zero-DOM compact array-v1 serializer (avoids building two
// ArduinoJson documents per history row on ESP32-S3).
#if defined(__GNUC__) && !defined(__clang__)
[[gnu::optimize("O2")]]
#endif
inline bool appendCompactHistoryPoint(char* dst, size_t cap, size_t& len,
                                      const NormalHistoryPoint& point,
                                      uint32_t mask) {
  const auto& t = point.telemetry;
  if (!appendRawChar(dst, cap, len, '[')) return false;
  bool first = true;
  uint32_t rem = mask & ((1UL << kHistoryColumnCount) - 1UL);
  while (rem != 0U) {
    const uint32_t col = static_cast<uint32_t>(__builtin_ctz(rem));
    rem &= rem - 1U;
    if (!first) {
      if (!appendRawChar(dst, cap, len, ',')) return false;
    } else {
      first = false;
    }
    bool ok = true;
    switch (col) {
      case 0:
        ok = point.timestamp == 0
                 ? appendRawChar(dst, cap, len, '0')
                 : (appendU32(dst, cap, len, point.timestamp) &&
                    appendLit(dst, cap, len, "000"));
        break;
      case 1: ok = appendU32(dst, cap, len, static_cast<uint8_t>(t.sensorType)); break;
      case 2: ok = t.pcbVersion ? appendU32(dst, cap, len, t.pcbVersion) : appendLit(dst, cap, len, "null"); break;
      case 3: ok = appendU32(dst, cap, len, t.iaqAccuracy); break;
      case 4: ok = (bool(t.live.flags & lil::protocol::kLiveGap) || t.motion.fifoOverrun) ? appendLit(dst, cap, len, "true") : appendLit(dst, cap, len, "false"); break;
      case 5: {
        const uint8_t tempDecimals =
            t.sensorType == lil::protocol::EnvironmentalSensorType::kTmp117 ? 6 : 4;
        ok = (t.capabilities & lil::protocol::kTemperature)
                 ? appendScaledFloat(dst, cap, len, float(t.temperatureC), tempDecimals)
                 : appendLit(dst, cap, len, "null");
        break;
      }
      case 6: ok = (t.capabilities & lil::protocol::kHumidity) ? appendScaledFloat(dst, cap, len, float(t.humidityPercent), 3) : appendLit(dst, cap, len, "null"); break;
      case 7: ok = (t.capabilities & lil::protocol::kPressure) ? appendScaledFloat(dst, cap, len, float(t.pressureHpa), 3) : appendLit(dst, cap, len, "null"); break;
      case 8: ok = ((t.capabilities & lil::protocol::kIaq) && t.iaqAccuracy) ? appendScaledFloat(dst, cap, len, float(t.iaq), 2) : appendLit(dst, cap, len, "null"); break;
      case 9: ok = (t.capabilities & lil::protocol::kGasResistance) ? appendScaledFloat(dst, cap, len, t.gasResistanceOhms * 0.001F, 3) : appendLit(dst, cap, len, "null"); break;
      case 10: ok = (t.capabilities & lil::protocol::kBattery) ? appendMilliU32(dst, cap, len, uint32_t(t.batteryMillivolts)) : appendLit(dst, cap, len, "null"); break;
      case 11: case 12: case 13:
        ok = (t.capabilities & lil::protocol::kMotion) ? appendScaledFloat(dst, cap, len, float(t.motion.accelerationG[col - 11]), 6) : appendLit(dst, cap, len, "null"); break;
      case 14: case 15: case 16:
        ok = (t.capabilities & lil::protocol::kMotion) ? appendScaledFloat(dst, cap, len, float(t.motion.angularRateDps[col - 14]), 4) : appendLit(dst, cap, len, "null"); break;
      case 17:
        ok = (t.capabilities & lil::protocol::kMotion) ? appendScaledFloat(dst, cap, len, float(t.motion.peakAccelerationG), 6) : appendLit(dst, cap, len, "null"); break;
      case 18:
        ok = (t.capabilities & lil::protocol::kMotion) ? appendScaledFloat(dst, cap, len, float(t.motion.peakAngularRateDps), 4) : appendLit(dst, cap, len, "null"); break;
      case 19:
        ok = (t.capabilities & lil::protocol::kAirQualityGas) ? appendU32(dst, cap, len, uint16_t(t.extended.airQuality.eco2Ppm)) : appendLit(dst, cap, len, "null"); break;
      case 20:
        ok = (t.capabilities & lil::protocol::kAirQualityGas) ? appendU32(dst, cap, len, uint16_t(t.extended.airQuality.tvocPpb)) : appendLit(dst, cap, len, "null"); break;
      case 21:
        ok = (t.capabilities & lil::protocol::kDistance) ? appendU32(dst, cap, len, uint16_t(t.extended.distance.distanceMm)) : appendLit(dst, cap, len, "null"); break;
      case 22:
        ok = (t.capabilities & lil::protocol::kDistance) ? appendU32(dst, cap, len, uint16_t(t.extended.distance.signalRateKcps)) : appendLit(dst, cap, len, "null"); break;
      case 23:
        ok = (t.capabilities & lil::protocol::kDistance) ? appendU32(dst, cap, len, uint16_t(t.extended.distance.ambientRateKcps)) : appendLit(dst, cap, len, "null"); break;
      case 24:
        ok = (t.capabilities & lil::protocol::kDistance) ? appendU32(dst, cap, len, uint8_t(t.extended.distance.rangeStatus)) : appendLit(dst, cap, len, "null"); break;
      case 25:
        if (t.capabilities & lil::protocol::kSpectral) {
          ok = appendRawChar(dst, cap, len, '[');
          for (size_t ch = 0; ok && ch < 10; ++ch) {
            if (ch > 0) ok = appendRawChar(dst, cap, len, ',');
            if (ok) ok = appendU32(dst, cap, len, uint16_t(t.extended.spectral.channels[ch]));
          }
          if (ok) ok = appendRawChar(dst, cap, len, ']');
        } else {
          ok = appendLit(dst, cap, len, "null");
        }
        break;
      case 26:
        ok = (t.capabilities & lil::protocol::kSpectral) ? appendU32(dst, cap, len, uint8_t(t.extended.spectral.gainCode)) : appendLit(dst, cap, len, "null"); break;
      case 27:
        ok = (t.capabilities & lil::protocol::kSpectral) ? appendU32(dst, cap, len, uint16_t(t.extended.spectral.integrationMs)) : appendLit(dst, cap, len, "null"); break;
      default: break;
    }
    if (!ok) return false;
  }
  return appendRawChar(dst, cap, len, ']');
}

// Compact 4-byte-aligned binary wire format (format=bin-v1) for high-speed
// zero-JSON live streaming from ESP32-S3 directly to browser DataView/TypedArray.
struct BinaryLiveHeader {
  uint32_t magic;          // 0x31424357 ("WCB1")
  uint16_t version;        // 1
  uint16_t recordSize;     // sizeof(BinaryLiveRecord) == 68
  uint32_t count;
  uint32_t bucketMs;
};
struct BinaryLiveRecord {
  uint32_t id;
  uint32_t ageMs;
  uint32_t measurementAgeMs;
  uint32_t estimateAgeMs;
  uint16_t windowMs;
  uint16_t capabilities;
  uint16_t flags;
  uint8_t sensorType;
  uint8_t pulseStatus;
  uint8_t quality;
  uint8_t motionOverrun;
  uint16_t batteryMv;
  float temperatureC;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float peakAccel;
  float peakGyro;
  float heartRate;
};
static_assert(sizeof(BinaryLiveHeader) == 16);
static_assert(sizeof(BinaryLiveRecord) == 68);

#if defined(__GNUC__) && !defined(__clang__)
[[gnu::optimize("O2")]]
#endif
inline void packBinaryLiveRecord(BinaryLiveRecord& out,
                                 const lil::protocol::TelemetryPayload& t,
                                 uint32_t receivedMs, uint32_t nowMs) {
  out = {};
  const uint32_t ageMs = static_cast<uint32_t>(nowMs - receivedMs);
  const bool timed = (t.live.flags & lil::protocol::kLiveTimingKnown) != 0;
  out.id = receivedMs;
  out.ageMs = ageMs;
  out.measurementAgeMs = ageMs + (timed && t.live.acquisitionAgeMs != UINT16_MAX ? t.live.acquisitionAgeMs : 0U);
  out.estimateAgeMs = ageMs + (timed && t.live.estimateAgeMs != UINT16_MAX ? t.live.estimateAgeMs : 0U);
  out.windowMs = t.live.windowMs;
  out.capabilities = t.capabilities;
  out.flags = t.flags;
  out.sensorType = static_cast<uint8_t>(t.sensorType);
  out.pulseStatus = t.pulse.status;
  out.quality = t.pulse.quality;
  out.motionOverrun = t.motion.fifoOverrun;
  out.batteryMv = t.batteryMillivolts;
  out.temperatureC = t.temperatureC;
  out.ax = t.motion.accelerationG[0];
  out.ay = t.motion.accelerationG[1];
  out.az = t.motion.accelerationG[2];
  out.gx = t.motion.angularRateDps[0];
  out.gy = t.motion.angularRateDps[1];
  out.gz = t.motion.angularRateDps[2];
  out.peakAccel = t.motion.peakAccelerationG;
  out.peakGyro = t.motion.peakAngularRateDps;
  out.heartRate = t.pulse.beatsPerMinute;
}

}  // namespace station

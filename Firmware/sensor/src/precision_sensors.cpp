#include "precision_sensors.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

namespace sensor {
using Type = lil::protocol::EnvironmentalSensorType;
namespace {
uint8_t opticalCurrent(uint8_t current, uint32_t level) {
  // Separate optical paths can have very different DC levels. Use a wide
  // settling band and bounded proportional steps, not opposing shared gains.
  // Below 2000 counts there is no useful evidence of contact: do not boost
  // dark-current noise all the way to the maximum LED setting.
  uint32_t desired = current;
  if (level > 220000) desired = uint32_t(current) * 150000 / level;
  else if (level >= 2000 && level < 50000)
    desired = (uint32_t(current) * 100000 + level - 1) / level;
  desired = constrain(desired, uint32_t(4), uint32_t(0x60));
  return static_cast<uint8_t>(constrain(desired,
      current > 12 ? uint32_t(current - 8) : uint32_t(4), uint32_t(current) + 8));
}
}
bool PrecisionSensors::readBytes(uint8_t reg, uint8_t* bytes, uint8_t count) {
  Wire.beginTransmission(address_); Wire.write(reg);
  if (Wire.endTransmission(false) || Wire.requestFrom(address_, count) != count) return false;
  for (uint8_t i = 0; i < count; ++i) bytes[i] = Wire.read();
  return true;
}
bool PrecisionSensors::write8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address_); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}
bool PrecisionSensors::write16(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(address_); Wire.write(reg); Wire.write(value >> 8); Wire.write(value & 255);
  return Wire.endTransmission() == 0;
}
bool PrecisionSensors::probe(Type type, uint8_t& address) {
  PrecisionSensors probe;
  uint8_t bytes[2];
  if (type == Type::kTmp117) {
    for (uint8_t a = 0x48; a <= 0x4B; ++a) {
      probe.address_ = a;
      if (probe.readBytes(0x0F, bytes, 2) && (((bytes[0] << 8) | bytes[1]) & 0xFFF) == 0x117) {
        address = a; return true;
      }
    }
  } else if (type == Type::kMax30102) {
    probe.address_ = 0x57;
    if (probe.readBytes(0xFF, bytes, 1) && bytes[0] == 0x15) { address = 0x57; return true; }
  }
  return false;
}
void PrecisionSensors::resetSignal() { head_ = count_ = 0; hasEstimate_ = hasCalculated_ = false; }
bool PrecisionSensors::invalidateOpticalCapture() {
  // A partial FIFO read can already advance the hardware pointer. Reinitialize
  // instead of accepting an uncertain sample boundary after the bus recovers.
  initialized_ = false; failed_ = waveGap_ = true; waveCount_ = 0;
  resetSignal();
  return false;
}
bool PrecisionSensors::clearOpticalFifo() {
  // Stop conversions while changing pointers so a newly completed sample
  // cannot race the three register writes. SHDN preserves configuration.
  return write8(9, 0x83) && write8(4, 0) && write8(5, 0) &&
      write8(6, 0) && write8(9, 3);
}
bool PrecisionSensors::begin(Type type, uint8_t address) {
  *this = PrecisionSensors{}; type_ = type; address_ = address;
  uint8_t bytes[2];
  if (type == Type::kTmp117) {
    if (!write16(1, 2)) return false;
    delay(5);
    // TI accuracy test condition: eight averages, 1 Hz continuous conversion.
    // About 124 ms active + 876 ms standby limits self-heating.
    if (!write16(1, 0x0220) || !readBytes(1, bytes, 2) ||
        ((((bytes[0] << 8) | bytes[1]) & 0xFFE) != 0x220)) return false;
    initialized_ = true; lastPollMs_ = millis() - 10;
    return true;
  }
  if (type != Type::kMax30102 || !write8(9, 0x40)) return false;
  const uint32_t start = millis();
  do {
    delay(1);
    if (!readBytes(9, bytes, 1) || millis() - start > 100) return false;
  } while (bytes[0] & 0x40);
  // Red + IR, 100 samples/s, 411 us / 18 bits, 4096 nA range.
  // Average four conversions per FIFO sample: 25 Hz signal, 1.28 s FIFO.
  const uint8_t config[][2] = {{2,0xA0},{3,0},{4,0},{5,0},{6,0},
      {8,0x40},{0x0A,0x27},{0x0C,redCurrent_},{0x0D,infraredCurrent_},{9,3}};
  for (const auto& entry : config) {
    if (!write8(entry[0], entry[1]) || !readBytes(entry[0], bytes, 1) || bytes[0] != entry[1]) return false;
  }
  // Clear the initial PWR_RDY indication; a subsequent one is a brownout.
  if (!readBytes(0, bytes, 1)) return false;
  initialized_ = true; lastAdjustMs_ = lastPollMs_ = millis();
  return true;
}
bool PrecisionSensors::poll() {
  if (!initialized_) return false;
  if (type_ == Type::kTmp117) {
    if (millis() - lastPollMs_ < 10) return !failed_;
    lastPollMs_ = millis();
    uint8_t b[2];
    if (!readBytes(1,b,2)) { failed_ = true; freshTemperature_ = false; return false; }
    if (b[0] & 0x20) {
      if (!readBytes(0,b,2)) { failed_ = true; freshTemperature_ = false; return false; }
      temperature_ = static_cast<int16_t>((b[0] << 8) | b[1]) / 128.0F;
      hasTemperature_ = temperature_ >= -55 && temperature_ <= 150;
      lastSampleMs_ = millis(); freshTemperature_ = hasTemperature_;
    }
    return true;
  }
  uint8_t status, ptr[3];
  if (!readBytes(0, &status, 1)) return invalidateOpticalCapture();
  if (status & 0x01) {
    // Brownout resets mode, averaging and LED currents. The acquisition task
    // must reinitialize the device before any more samples are accepted.
    return invalidateOpticalCapture();
  }
  if (!readBytes(4,ptr,3)) return invalidateOpticalCapture();
  const bool pollGap = millis() - lastPollMs_ >= 1200;
  lastPollMs_ = millis();
  if (ptr[1] || (status & 0xA0) || pollGap) {
    // Equal read/write pointers can also mean all 32 slots are occupied.
    // A_FULL resolves that ambiguity. ALC_OVF means ambient light corrupted
    // the ADC signal even when the numerical sample range looks plausible.
    overflow_ = waveGap_ = true; waveCount_ = 0; hasOptical_ = false; resetSignal();
    if (!clearOpticalFifo()) return invalidateOpticalCapture();
    return !failed_;
  }
  const uint8_t count = (ptr[0] - ptr[2]) & 31;
  const uint32_t newestMs = millis();
  if (hasOptical_ && count == 0 && newestMs - lastSampleMs_ >= 300) {
    waveGap_ = true; waveCount_ = 0; hasOptical_ = false; resetSignal();
  }
  for (uint8_t i = 0; i < count; ++i) {
    uint8_t b[6];
    if (!readBytes(7,b,6)) return invalidateOpticalCapture();
    red_ = ((uint32_t(b[0]) << 16) | (uint32_t(b[1]) << 8) | b[2]) & 0x3FFFF;
    infrared_ = ((uint32_t(b[3]) << 16) | (uint32_t(b[4]) << 8) | b[5]) & 0x3FFFF;
    // No hardware timestamp: backdate FIFO entries at the nominal 40 ms period.
    lastSampleMs_ = newestMs - uint32_t(count - 1 - i) * lil::timing::kOpticalSampleMs;
    hasOptical_ = true;
    if (waveCount_ == lil::protocol::kOpticalBatchSize) {
      for (size_t j = 1; j < lil::protocol::kOpticalBatchSize; ++j) { wave_[j-1] = wave_[j]; waveTimes_[j-1] = waveTimes_[j]; }
      --waveCount_; waveGap_ = true;
    }
    wave_[waveCount_] = {red_, infrared_, 0}; waveTimes_[waveCount_++] = lastSampleMs_;
    // Only IR enters the heart-rate estimator. A dim or clipped red channel
    // is not evidence that the separately sampled IR pulse is unusable.
    if (infrared_ < 10000 || infrared_ > 250000) { resetSignal(); continue; }
    signal_[head_] = infrared_; head_ = (head_ + 1) % 200;
    if (count_ < 200) ++count_;
  }
  // Bounded optical gain settling; every change invalidates the analysis window.
  if (count && millis() - lastAdjustMs_ >= 1000) {
    lastAdjustMs_ = millis();
    const uint8_t nextRed = opticalCurrent(redCurrent_, red_);
    const uint8_t nextInfrared = opticalCurrent(infraredCurrent_, infrared_);
    if (nextRed != redCurrent_ || nextInfrared != infraredCurrent_) {
      // Samples acquired before the gain change must not enter the new window.
      // Discard the FIFO and mark the discontinuity in the waveform explicitly.
      const bool changed = write8(9,0x83) && write8(0x0C,nextRed) && write8(0x0D,nextInfrared) &&
          clearOpticalFifo();
      if (!changed) return invalidateOpticalCapture();
      redCurrent_ = nextRed; infraredCurrent_ = nextInfrared;
      // Do not present pre-adjustment ADC values as a fresh settled capture.
      waveGap_ = true; waveCount_ = 0; hasOptical_ = false; resetSignal();
    }
  }
  return !failed_;
}
EnvironmentalReading PrecisionSensors::read() {
  poll();
  EnvironmentalReading result{}; result.sensorType = type_;
  const uint32_t processingStart = millis();
  result.live.flags = lil::protocol::kLiveTimingKnown;
  if (type_ == Type::kTmp117) {
    result.valid = initialized_ && !failed_ && hasTemperature_ && millis() - lastSampleMs_ < 1500;
    result.temperatureC = temperature_;
    result.live.windowMs = lil::timing::kTemperatureActiveMs;
    if (freshTemperature_) result.live.flags |= lil::protocol::kLiveSampleFresh;
    freshTemperature_ = false;
    if (result.valid) result.capabilities = lil::protocol::kTemperature;
  } else {
    result.valid = initialized_ && !failed_ && hasOptical_ && millis() - lastSampleMs_ < 300;
    result.live.samplePeriodUs = lil::timing::kOpticalSampleMs * 1000;
    result.live.windowMs = lil::timing::kPulseWindowMs;
    result.live.warmupMs = (lil::timing::kPulseSamples - count_) * lil::timing::kOpticalSampleMs;
    result.live.count = waveCount_;
    for (uint8_t i = 0; i < waveCount_; ++i) {
      result.live.optical[i] = wave_[i];
      result.live.optical[i].ageMs = static_cast<uint16_t>(min(uint32_t(65534), millis() - waveTimes_[i]));
    }
    if (waveCount_) result.live.flags |= lil::protocol::kLiveSampleFresh;
    if (waveGap_) result.live.flags |= lil::protocol::kLiveGap;
    waveCount_ = 0; waveGap_ = false;
    result.pulse.red = red_; result.pulse.infrared = infrared_;
    result.pulse.status = overflow_ ? 3 : (infrared_ < 10000 ? 0 : 1);
    if (result.valid) result.capabilities = lil::protocol::kOptical;
    // Eight seconds of uninterrupted samples. Detrend with a centered 1 s
    // mean, then accept only a strong periodic peak (30..200 bpm).
    if (result.valid && count_ == lil::timing::kPulseSamples && !overflow_ &&
        (!hasCalculated_ || millis() - lastEstimateMs_ >= lil::timing::kPulseUpdateMs)) {
      lastEstimateMs_ = millis(); hasCalculated_ = true;
      hasEstimate_ = false;
      constexpr int halfMean = 12, analyzed = 200 - 2 * halfMean;
      float ac[analyzed]; double energy = 0;
      for (int i = halfMean; i < 200 - halfMean; ++i) {
        double mean = 0;
        for (int j = -halfMean; j <= halfMean; ++j) mean += signal_[(head_ + i + j) % 200];
        ac[i-halfMean] = signal_[(head_ + i) % 200] - mean / (2 * halfMean + 1);
        energy += ac[i-halfMean] * ac[i-halfMean];
      }
      float correlations[52]{};
      for (int lag = 6; lag <= 51; ++lag) {
        double xy = 0, xx = 0, yy = 0;
        for (int i = 0; i < analyzed-lag; ++i) { xy += ac[i]*ac[i+lag]; xx += ac[i]*ac[i]; yy += ac[i+lag]*ac[i+lag]; }
        correlations[lag] = xx > 0 && yy > 0 ? xy / sqrt(xx*yy) : 0;
      }
      float best = 0; int lagBest = 0;
      for (int lag = 7; lag <= 50; ++lag) {
        if (correlations[lag] > 0.75F && correlations[lag] > correlations[lag-1] &&
            correlations[lag] >= correlations[lag+1] && correlations[lag] > best + 0.05F) {
          best = correlations[lag]; lagBest = lag;
        }
      }
      const float rms = sqrt(energy / analyzed);
      if (lagBest && rms > 20 && rms < infrared_ * 0.1F) {
        // Sub-sample peak interpolation avoids 25 Hz integer-period quantization.
        const float left = correlations[lagBest-1], right = correlations[lagBest+1];
        const float denominator = left - 2*best + right;
        const float offset = denominator < -0.00001F ? 0.5F*(left-right)/denominator : 0;
        const float bpm = 1500.0F / (lagBest + offset);
        if (bpm >= 30 && bpm <= 200) {
          estimate_.beatsPerMinute = bpm;
          estimate_.quality = static_cast<uint8_t>(best * 100);
          estimateSampleMs_ = lastSampleMs_; hasEstimate_ = true;
          result.live.flags |= lil::protocol::kLiveEstimateFresh;
        }
      }
    }
  }
  if (type_ == Type::kMax30102 && result.valid && hasEstimate_ && millis() - estimateSampleMs_ < 1500 && !overflow_) {
    result.pulse.beatsPerMinute = estimate_.beatsPerMinute; result.pulse.quality = estimate_.quality;
    result.pulse.status = 2; result.capabilities |= lil::protocol::kHeartRate;
    result.live.estimateAgeMs = static_cast<uint16_t>(millis() - estimateSampleMs_);
  }
  result.live.processingMs = static_cast<uint16_t>(min(uint32_t(65534), millis() - processingStart));
  if (hasTemperature_ || hasOptical_) result.live.acquisitionAgeMs = static_cast<uint16_t>(min(uint32_t(65534), millis() - lastSampleMs_));
  // Include estimator work in the relative ages carried over the radio.
  for (uint8_t i = 0; i < result.live.count; ++i) result.live.optical[i].ageMs =
      static_cast<uint16_t>(min(uint32_t(65534), uint32_t(result.live.optical[i].ageMs) + result.live.processingMs));
  failed_ = overflow_ = false;
  return result;
}
}

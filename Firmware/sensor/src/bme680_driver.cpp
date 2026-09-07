#include "bme680_driver.h"

#include <esp_attr.h>
#include <esp_timer.h>
#include <string.h>

#include "low_power_wait.h"
#include "sensor_log.h"
#include "measurement_budget.h"
#include "logical_clock.h"
#include "power_policy.h"

namespace sensor {

namespace {
// Revision 8 invalidates RTC data learned by the former LP/ULP switching
// routine. It complements the NVS format bump in bsec_state_store.cpp.
constexpr uint32_t kRtcSignature = 0x42525439UL;  // "BRT9"
// Bosch's examples persist the adaptive BSEC state periodically. Every sample
// remains in RTC memory; NVS is only updated on an accuracy improvement or
// after six hours.
constexpr uint64_t kNvsSaveIntervalMs = 6ULL * 60ULL * 60ULL * 1000ULL;
constexpr uint32_t kUlpStabilizationSeconds = 20UL * 60UL;
constexpr uint16_t kRawHeaterTemperatureC = 320;
constexpr uint16_t kRawHeaterDurationMs = 150;
constexpr uint32_t kRawMeasurementMarginUs = 10000;
// A power-cycled BME680 has no previous field which BSEC can consume.  Keep
// the rail on for one bounded acquisition window and require a genuinely new
// BSEC output before falling back to the direct Bosch API.  The five-second
// ceiling covers the BME680's maximum 4032-ms gas wait plus T/P/H conversion,
// I2C and scheduling margin without ever keeping the node awake for 300 s.
constexpr uint32_t kBsecOutputTimeoutMs = 5000;
constexpr uint32_t kBsecOutputPollMs = 10;

// The BME680 stays in ULP mode for its complete service life. Bosch advises
// against changing between LP and ULP because their adaptive gas baselines are
// not interchangeable. The board powers the sensor from 3.3 V, so use only the
// matching energy-efficient 300-second configuration.
const uint8_t kBsecConfigUlp33v[] = {
#include "config/bme680/bme680_iaq_33v_300s_4d/bsec_iaq.txt"
};
static_assert(sizeof(kBsecConfigUlp33v) == BSEC_MAX_PROPERTY_BLOB_SIZE,
              "Unexpected Bosch ULP configuration size");

struct BsecRtcState {
  uint32_t signature = 0;
  uint8_t state[BSEC_MAX_STATE_BLOB_SIZE]{};
  uint8_t lastSavedAccuracy = 0;
  uint64_t lastNvsSaveLogicalMs = 0;
  bool stateValid = false;
  uint32_t calibrationElapsedSeconds = 0;
  bool calibrationReady = false;
  bool stabilizationStatusSeen = false;
  bool stabilizationFinished = false;
  bool runInStatusSeen = false;
  bool runInFinished = false;
  bool bsecOutputTimestampSeen = false;
  int64_t lastBsecOutputTimestampNs = 0;
  float lastValidIaq = 0.0F;
  uint8_t lastValidIaqAccuracy = 0;
  bool lastValidIaqSeen = false;
  uint64_t nextBsecCallLogicalMs = 0;
  // Diagnostic only. BSEC must still be attempted on every ULP wake: skipping
  // calls stops IAQ learning and can keep accuracy at zero indefinitely.
};

RTC_DATA_ATTR BsecRtcState rtcBsecState{};
bool clockOverride = false;
uint32_t clockOverrideValue = 0;

uint64_t bootElapsedMs() {
  return static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
}

unsigned long bsecLogicalMillis() {
  if (clockOverride) {
    return clockOverrideValue;
  }
  return static_cast<uint32_t>(sensor::logicalTimeMs());
}

bool validReading(float value) {
  return isfinite(value);
}

bool inRange(float value, float minimum, float maximum) {
  return validReading(value) && value >= minimum && value <= maximum;
}

uint64_t logicalNowMs() {
  return sensor::logicalTimeMs();
}
}  // namespace

bool Bme680Driver::ensureStateStore() {
  if (!stateStoreReady_) {
    stateStoreReady_ = stateStore_.begin();
  }
  return stateStoreReady_;
}

bool Bme680Driver::begin(uint8_t address, float temperatureOffsetC) {
  rawReady_ = false;
  bsecReady_ = false;
  scheduleUpdated_ = false;
  address_ = address;
  temperatureOffsetC_ = constrain(temperatureOffsetC, -10.0F, 10.0F);
  if (rtcBsecState.signature != kRtcSignature) {
    rtcBsecState = BsecRtcState{};
    rtcBsecState.signature = kRtcSignature;
    if (ensureStateStore()) {
      rtcBsecState.stateValid =
          stateStore_.load(rtcBsecState.state, sizeof(rtcBsecState.state));
      stateStore_.loadCalibration(rtcBsecState.calibrationElapsedSeconds,
                                  rtcBsecState.calibrationReady);
    }
  }
  calibrationAwakeStartedMs_ = bootElapsedMs();
  calibrationElapsedAtBeginSeconds_ =
      rtcBsecState.calibrationElapsedSeconds;

  SENSOR_LOG_PRINTF("[BME680] Chip ID 0x61 detected at address 0x%02X\n",
                    address);

  // Run BSEC on every ULP wake to preserve IAQ learning. Initialize the raw
  // Bosch fallback only when this path fails, avoiding a second sensor reset.
  bsec_.allocateMemory(bsecMemory_);
  bsecReady_ = tryAddress(address);
  if (bsecReady_) {
    seedClockOverflowCounter();
    const bool configApplied = bsec_.setConfig(kBsecConfigUlp33v);
    if (!configApplied || bsec_.status < BSEC_OK) {
      SENSOR_LOG_PRINTF("[BME680] BSEC configuration failed: %d\n",
                        static_cast<int>(bsec_.status));
      bsecReady_ = false;
    }
  }
  if (bsecReady_) {
    bsec_.setTemperatureOffset(temperatureOffsetC_);
    restoreState();

    bsecSensor outputs[] = {
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_STABILIZATION_STATUS,
        BSEC_OUTPUT_RUN_IN_STATUS,
    };
    bsecReady_ = bsec_.updateSubscription(
        outputs, ARRAY_LEN(outputs), BSEC_SAMPLE_RATE_ULP);
    if (!bsecReady_) {
      SENSOR_LOG_PRINTF("[BME680] BSEC subscription failed: %d\n",
                        static_cast<int>(bsec_.status));
    }
  }
  if (bsecReady_) {
    SENSOR_LOG_PRINTF(
        "[BME680] BSEC %u.%u.%u.%u ready (%s, 3.3 V profile)\n",
        bsec_.version.major, bsec_.version.minor,
        bsec_.version.major_bugfix, bsec_.version.minor_bugfix, "ULP/300 s");
  } else {
    SENSOR_LOG_PRINTLN("[BME680] Direct forced-mode fallback active");
  }
  // Initialize the fallback only if BSEC initialization failed. A successful
  // BSEC cycle already owns a fully initialized physical sensor.
  return bsecReady_ || (!measurementBudgetExpired() && beginRaw(address_));
}

int8_t Bme680Driver::rawI2cRead(uint8_t registerAddress, uint8_t* data,
                                uint32_t length, void* context) {
  auto* bus = static_cast<RawI2cContext*>(context);
  if (measurementBudgetExpired()) return BME68X_E_COM_FAIL;
  if (bus == nullptr || bus->wire == nullptr || data == nullptr) {
    return BME68X_E_NULL_PTR;
  }
  bus->wire->beginTransmission(bus->address);
  bus->wire->write(registerAddress);
  if (bus->wire->endTransmission() != 0 || measurementBudgetExpired() ||
      bus->wire->requestFrom(bus->address, static_cast<size_t>(length)) !=
          length) {
    return BME68X_E_COM_FAIL;
  }
  for (uint32_t index = 0; index < length; ++index) {
    if (!bus->wire->available()) {
      return BME68X_E_COM_FAIL;
    }
    data[index] = bus->wire->read();
  }
  return BME68X_OK;
}

int8_t Bme680Driver::rawI2cWrite(uint8_t registerAddress,
                                 const uint8_t* data, uint32_t length,
                                 void* context) {
  auto* bus = static_cast<RawI2cContext*>(context);
  if (measurementBudgetExpired()) return BME68X_E_COM_FAIL;
  if (bus == nullptr || bus->wire == nullptr || data == nullptr) {
    return BME68X_E_NULL_PTR;
  }
  bus->wire->beginTransmission(bus->address);
  bus->wire->write(registerAddress);
  for (uint32_t index = 0; index < length; ++index) {
    bus->wire->write(data[index]);
  }
  return bus->wire->endTransmission() == 0 ? BME68X_OK
                                            : BME68X_E_COM_FAIL;
}

void Bme680Driver::rawDelayUs(uint32_t periodUs, void* context) {
  (void)context;
  measurementWaitUs(periodUs);
}

bool Bme680Driver::beginRaw(uint8_t address) {
  rawContext_ = RawI2cContext{&Wire, address};
  rawDevice_ = bme68x_dev{};
  rawDevice_.read = rawI2cRead;
  rawDevice_.write = rawI2cWrite;
  rawDevice_.delay_us = rawDelayUs;
  rawDevice_.intf = BME68X_I2C_INTF;
  rawDevice_.intf_ptr = &rawContext_;
  rawDevice_.amb_temp = 25;
  if (bme68x_init(&rawDevice_) != BME68X_OK ||
      bme68x_get_conf(&rawConfiguration_, &rawDevice_) != BME68X_OK) {
    return false;
  }
  // This path is only a one-shot availability fallback. With no continuous
  // sample stream there is no useful IIR history, so filtering is disabled
  // and modest oversampling avoids heating the MCU/sensor rail unnecessarily.
  rawConfiguration_.filter = BME68X_FILTER_OFF;
  rawConfiguration_.odr = BME68X_ODR_NONE;
  rawConfiguration_.os_temp = BME68X_OS_2X;
  rawConfiguration_.os_hum = BME68X_OS_1X;
  rawConfiguration_.os_pres = BME68X_OS_4X;
  rawHeater_ = bme68x_heatr_conf{};
  rawHeater_.enable = BME68X_ENABLE;
  rawHeater_.heatr_temp = kRawHeaterTemperatureC;
  rawHeater_.heatr_dur = kRawHeaterDurationMs;
  rawReady_ = bme68x_set_conf(&rawConfiguration_, &rawDevice_) == BME68X_OK &&
              bme68x_set_heatr_conf(BME68X_FORCED_MODE, &rawHeater_,
                                    &rawDevice_) == BME68X_OK;
  return rawReady_;
}

bool Bme680Driver::tryAddress(uint8_t address) {
  rawContext_ = RawI2cContext{&Wire, address};
  return bsec_.begin(BME68X_I2C_INTF, rawI2cRead, rawI2cWrite,
                     rawDelayUs, &rawContext_, bsecLogicalMillis) &&
         bsec_.status >= BSEC_OK && bsec_.sensor.status >= BME68X_OK;
}

void Bme680Driver::seedClockOverflowCounter() {
  const uint64_t absoluteMs = sensor::logicalTimeMs();
  const uint32_t wraps = static_cast<uint32_t>(absoluteMs >> 32U);
  clockOverride = true;
  for (uint32_t wrap = 0; wrap < wraps; ++wrap) {
    clockOverrideValue = UINT32_MAX;
    bsec_.getTimeMs();
    clockOverrideValue = 0;
    bsec_.getTimeMs();
  }
  clockOverride = false;
}

void Bme680Driver::restoreState() {
  if (!rtcBsecState.stateValid) {
    return;
  }
  if (!bsec_.setState(rtcBsecState.state)) {
    SENSOR_LOG_PRINTF("[BME680] Discarded BSEC state, status %d\n",
                      static_cast<int>(bsec_.status));
    rtcBsecState.stateValid = false;
  } else {
    SENSOR_LOG_PRINTLN("[BME680] Restored saved BSEC learning state");
  }
}

EnvironmentalReading Bme680Driver::read() {
  if (bsecReady_) {
    EnvironmentalReading bsecReading = readBsec();
    if (bsecReading.valid) {
      return bsecReading;
    }
  }
  if (!rawReady_ && !measurementBudgetExpired()) beginRaw(address_);
  return readRaw();
}

EnvironmentalReading Bme680Driver::readBsec() {
  EnvironmentalReading reading{};
  reading.sensorType = lil::protocol::EnvironmentalSensorType::kBme680;
  if (!bsecReady_) {
    return reading;
  }

  const bsecOutputs* outputs = nullptr;
  int64_t newestTimestampNs = INT64_MIN;
  const uint32_t startedMs = millis();
  for (;;) {
    if (measurementBudgetExpired()) return reading;
    const bool ran = bsec_.run();
    if (!ran || bsec_.status < BSEC_OK ||
        bsec_.sensor.status < BME68X_OK) {
      SENSOR_LOG_PRINTF(
          "[BME680] BSEC cycle produced no output (BSEC %d, sensor %d)\n",
          static_cast<int>(bsec_.status),
          static_cast<int>(bsec_.sensor.status));
      return reading;
    }
    const int64_t nextCallNs = bsec_.getNextCallNs();
    if (nextCallNs > 0) {
      scheduleUpdated_ = true;
      rtcBsecState.nextBsecCallLogicalMs =
          static_cast<uint64_t>((nextCallNs + 999999LL) / 1000000LL);
    }
    if (bsec_.status > BSEC_OK || bsec_.sensor.status > BME68X_OK) {
      SENSOR_LOG_PRINTF("[BME680] BSEC warning %d, sensor warning %d\n",
                        static_cast<int>(bsec_.status),
                        static_cast<int>(bsec_.sensor.status));
    }

    outputs = bsec_.getOutputs();
    newestTimestampNs = INT64_MIN;
    if (outputs != nullptr) {
      for (uint8_t index = 0; index < outputs->nOutputs; ++index) {
        newestTimestampNs =
            max(newestTimestampNs, outputs->output[index].time_stamp);
      }
    }
    const bool freshOutput =
        outputs != nullptr && outputs->nOutputs > 0 &&
        (!rtcBsecState.bsecOutputTimestampSeen ||
         newestTimestampNs > rtcBsecState.lastBsecOutputTimestampNs);
    if (freshOutput) {
      rtcBsecState.bsecOutputTimestampSeen = true;
      rtcBsecState.lastBsecOutputTimestampNs = newestTimestampNs;
      break;
    }
    outputs = nullptr;
    const uint32_t elapsedMs = millis() - startedMs;
    if (elapsedMs >= kBsecOutputTimeoutMs) {
      break;
    }
    // Bsec2::run() returning true only means that no library error occurred;
    // it does not promise that next_call was due or that do_steps produced an
    // output.  Poll for a bounded time while the rail is still powered.
    uint32_t waitMs = kBsecOutputPollMs;
    const int64_t nowMs = bsec_.getTimeMs();
    if (nextCallNs > nowMs * 1000000LL) {
      const uint64_t untilNextMs = static_cast<uint64_t>(
          (nextCallNs - nowMs * 1000000LL + 999999LL) / 1000000LL);
      if (untilNextMs > kBsecOutputTimeoutMs - elapsedMs) {
        // Do not burn energy waiting for a future ULP slot. The absolute
        // next_call is retained so the caller can power the rail down and
        // deep-sleep for only the remaining time.
        break;
      }
      waitMs = static_cast<uint32_t>(max<uint64_t>(1, untilNextMs));
    }
    measurementWaitUs(
        min(waitMs, kBsecOutputTimeoutMs - elapsedMs) * 1000UL);
  }
  if (outputs == nullptr) {
    SENSOR_LOG_PRINTF(
        "[BME680] BSEC produced no fresh output within %lu ms\n",
        static_cast<unsigned long>(kBsecOutputTimeoutMs));
    return reading;
  }

  bool hasTemperature = false;
  bool hasHumidity = false;
  bool hasPressure = false;
  bool hasIaq = false;
  bool hasGas = false;
  for (uint8_t index = 0; index < outputs->nOutputs; ++index) {
    const bsecData& output = outputs->output[index];
    switch (output.sensor_id) {
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
        reading.temperatureC = output.signal;
        hasTemperature = true;
        break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
        reading.humidityPercent = output.signal;
        hasHumidity = true;
        break;
      case BSEC_OUTPUT_RAW_PRESSURE:
        reading.pressureHpa =
            output.signal > 2000.0F ? output.signal / 100.0F : output.signal;
        hasPressure = true;
        break;
      case BSEC_OUTPUT_RAW_GAS:
        reading.gasResistanceOhms = output.signal;
        hasGas = true;
        break;
      case BSEC_OUTPUT_STATIC_IAQ:
        reading.iaq = output.signal;
        reading.iaqAccuracy = output.accuracy;
        hasIaq = true;
        break;
      case BSEC_OUTPUT_STABILIZATION_STATUS:
        rtcBsecState.stabilizationStatusSeen = true;
        rtcBsecState.stabilizationFinished = output.signal >= 0.5F;
        break;
      case BSEC_OUTPUT_RUN_IN_STATUS:
        rtcBsecState.runInStatusSeen = true;
        rtcBsecState.runInFinished = output.signal >= 0.5F;
        break;
      default:
        break;
    }
  }

  if (hasTemperature && inRange(reading.temperatureC, -40.0F, 85.0F)) {
    reading.capabilities |= lil::protocol::kTemperature;
  }
  if (hasHumidity && inRange(reading.humidityPercent, 0.0F, 100.0F)) {
    reading.capabilities |= lil::protocol::kHumidity;
  }
  if (hasPressure && inRange(reading.pressureHpa, 300.0F, 1100.0F)) {
    reading.capabilities |= lil::protocol::kPressure;
  }
  if (hasIaq && inRange(reading.iaq, 0.0F, 500.0F)) {
    reading.capabilities |= lil::protocol::kIaq;
    rtcBsecState.lastValidIaq = reading.iaq;
    rtcBsecState.lastValidIaqAccuracy =
        min(reading.iaqAccuracy, static_cast<uint8_t>(3));
    rtcBsecState.lastValidIaqSeen = true;
  }
  if (hasGas && validReading(reading.gasResistanceOhms) &&
      reading.gasResistanceOhms > 0.0F) {
    reading.capabilities |= lil::protocol::kGasResistance;
  }
  reading.iaqAccuracy = min(reading.iaqAccuracy, static_cast<uint8_t>(3));
  reading.valid = hasTemperature && hasHumidity && hasPressure &&
                  (reading.capabilities & lil::protocol::kTemperature) != 0 &&
                  (reading.capabilities & lil::protocol::kHumidity) != 0 &&
                  (reading.capabilities & lil::protocol::kPressure) != 0;
  updateCalibrationStatus(reading);
  saveState(reading.iaqAccuracy);
  SENSOR_LOG_PRINTF(
      "[BME680] BSEC ULP: T=%.2f C, H=%.2f %%, P=%.1f hPa, "
      "gas=%.0f ohm, IAQ=%.1f, accuracy=%u, stabilization=%d, "
      "Run-in=%d\n",
      reading.temperatureC, reading.humidityPercent, reading.pressureHpa,
      reading.gasResistanceOhms, reading.iaq, reading.iaqAccuracy,
      rtcBsecState.stabilizationStatusSeen
          ? static_cast<int>(rtcBsecState.stabilizationFinished)
          : -1,
      rtcBsecState.runInStatusSeen
          ? static_cast<int>(rtcBsecState.runInFinished)
          : -1);
  return reading;
}

EnvironmentalReading Bme680Driver::readRaw() {
  EnvironmentalReading reading{};
  reading.sensorType = lil::protocol::EnvironmentalSensorType::kBme680;
  reading.bme680RawFallback = true;
  if (measurementBudgetExpired() || !rawReady_ ||
      bme68x_set_conf(&rawConfiguration_, &rawDevice_) != BME68X_OK ||
      bme68x_set_heatr_conf(BME68X_FORCED_MODE, &rawHeater_, &rawDevice_) !=
          BME68X_OK ||
      bme68x_set_op_mode(BME68X_FORCED_MODE, &rawDevice_) != BME68X_OK) {
    SENSOR_LOG_PRINTLN("[BME680] Forced-mode configuration failed");
    updateCalibrationStatus(reading);
    return reading;
  }

  const uint32_t measurementUs =
      bme68x_get_meas_dur(BME68X_FORCED_MODE, &rawConfiguration_,
                          &rawDevice_) +
      static_cast<uint32_t>(rawHeater_.heatr_dur) * 1000UL +
      kRawMeasurementMarginUs;
  rawDelayUs(measurementUs, nullptr);

  bme68x_data data{};
  uint8_t fields = 0;
  int8_t status = BME68X_W_NO_NEW_DATA;
  for (uint8_t attempt = 0; attempt < 3 && fields == 0; ++attempt) {
    status = bme68x_get_data(BME68X_FORCED_MODE, &data, &fields, &rawDevice_);
    if (status < BME68X_OK) {
      SENSOR_LOG_PRINTF("[BME680] Read failed, Bosch status %d\n",
                        static_cast<int>(status));
      updateCalibrationStatus(reading);
      return reading;
    }
    if (fields == 0) {
      measurementWaitUs(10000);
    }
  }
  if (fields == 0) {
    SENSOR_LOG_PRINTLN("[BME680] No new data after forced-mode measurement");
    updateCalibrationStatus(reading);
    return reading;
  }

  reading.temperatureC = data.temperature - temperatureOffsetC_;
  reading.humidityPercent = data.humidity;
  reading.pressureHpa = data.pressure / 100.0F;
  reading.gasResistanceOhms = data.gas_resistance;
  if (inRange(reading.temperatureC, -40.0F, 85.0F)) {
    reading.capabilities |= lil::protocol::kTemperature;
  }
  if (inRange(reading.humidityPercent, 0.0F, 100.0F)) {
    reading.capabilities |= lil::protocol::kHumidity;
  }
  if (inRange(reading.pressureHpa, 300.0F, 1100.0F)) {
    reading.capabilities |= lil::protocol::kPressure;
  }

  const bool gasValid =
      (data.status & BME68X_GASM_VALID_MSK) != 0 &&
      validReading(reading.gasResistanceOhms) &&
      reading.gasResistanceOhms > 0.0F;
  const bool heaterStable = (data.status & BME68X_HEAT_STAB_MSK) != 0;
  (void)heaterStable;
  if (gasValid) {
    reading.capabilities |= lil::protocol::kGasResistance;
  }

  reading.valid =
      (reading.capabilities & lil::protocol::kTemperature) != 0 &&
      (reading.capabilities & lil::protocol::kHumidity) != 0 &&
      (reading.capabilities & lil::protocol::kPressure) != 0;
  updateCalibrationStatus(reading);
  if (rtcBsecState.lastValidIaqSeen) {
    // Preserve the last real BSEC value for the local display.  The raw
    // fallback flag still marks it as stale, and the station deliberately
    // excludes it from cloud uploads until a fresh BSEC cycle succeeds.
    reading.iaq = rtcBsecState.lastValidIaq;
    reading.iaqAccuracy = rtcBsecState.lastValidIaqAccuracy;
    reading.capabilities |= lil::protocol::kIaq;
  }
  // Raw gas resistance is useful diagnostic data, but it is not Bosch Static
  // IAQ. Deliberately leave the IAQ capability unset instead of presenting a
  // locally invented score as a calibrated BSEC result.
  SENSOR_LOG_PRINTF(
      "[BME680] Direct reading: %.2f C, %.2f %%, %.1f hPa, %.0f ohm "
      "(gas_valid=%u, heat_stable=%u; no substitute IAQ value)\n",
      reading.temperatureC, reading.humidityPercent, reading.pressureHpa,
      reading.gasResistanceOhms, gasValid, heaterStable);
  return reading;
}

void Bme680Driver::updateCalibrationStatus(EnvironmentalReading& reading) {
  const uint32_t elapsed = currentCalibrationElapsedSeconds();
  reading.iaqCalibrationElapsedMinutes = static_cast<uint16_t>(
      min(elapsed / 60UL, static_cast<uint32_t>(UINT16_MAX - 1U)));
  // Accuracy 1 is the first BSEC level that provides a usable learned IAQ
  // baseline. From this point onward the sensor remains in normal operation
  // while BSEC continues improving the accuracy in the background.
  if (reading.iaqAccuracy >= 1 && !rtcBsecState.calibrationReady) {
    rtcBsecState.calibrationElapsedSeconds = elapsed;
    rtcBsecState.calibrationReady = true;
    saveCalibrationMetadata();
  }
  if (rtcBsecState.calibrationReady) {
    reading.iaqCalibrationPhase =
        reading.iaqAccuracy >= 3
            ? lil::protocol::IaqCalibrationPhase::kHighAccuracy
            : lil::protocol::IaqCalibrationPhase::kReady;
    reading.iaqCalibrationRemainingMinutes = 0;
    return;
  }
  const uint32_t stabilizationSeconds = kUlpStabilizationSeconds;
  const bool bsecStillStabilizing =
      (rtcBsecState.stabilizationStatusSeen &&
       !rtcBsecState.stabilizationFinished) ||
      (rtcBsecState.runInStatusSeen && !rtcBsecState.runInFinished);
  if (elapsed < stabilizationSeconds || bsecStillStabilizing) {
    reading.iaqCalibrationPhase =
        lil::protocol::IaqCalibrationPhase::kStabilizing;
    reading.iaqCalibrationRemainingMinutes =
        elapsed < stabilizationSeconds
            ? static_cast<uint16_t>(
                  (stabilizationSeconds - elapsed + 59UL) / 60UL)
            : lil::protocol::kCalibrationTimeUnknown;
    return;
  }
  reading.iaqCalibrationPhase =
      lil::protocol::IaqCalibrationPhase::kLearning;
  // BSEC accuracy depends on receiving both clean-air and polluted-air
  // stimuli. There is no honest fixed countdown after stabilization.
  reading.iaqCalibrationRemainingMinutes =
      lil::protocol::kCalibrationTimeUnknown;
}

void Bme680Driver::saveState(uint8_t iaqAccuracy) {
  if (!bsecReady_ || !bsec_.getState(rtcBsecState.state)) {
    return;
  }
  rtcBsecState.stateValid = true;
  const bool accuracyImproved = iaqAccuracy > rtcBsecState.lastSavedAccuracy;
  const uint64_t nowMs = logicalNowMs();
  const bool periodicSaveDue =
      rtcBsecState.lastNvsSaveLogicalMs != 0 &&
      nowMs >= rtcBsecState.lastNvsSaveLogicalMs &&
      nowMs - rtcBsecState.lastNvsSaveLogicalMs >= kNvsSaveIntervalMs;
  if ((accuracyImproved || periodicSaveDue) && ensureStateStore()) {
    if (stateStore_.save(rtcBsecState.state, sizeof(rtcBsecState.state))) {
      rtcBsecState.lastNvsSaveLogicalMs = nowMs;
      rtcBsecState.lastSavedAccuracy = iaqAccuracy;
      if (!rtcBsecState.calibrationReady) {
        rtcBsecState.calibrationElapsedSeconds =
            currentCalibrationElapsedSeconds();
      }
      saveCalibrationMetadata();
      SENSOR_LOG_PRINTF("[BME680] Saved BSEC state (accuracy %u)\n",
                        iaqAccuracy);
    }
  }
}

void Bme680Driver::saveCalibrationMetadata() {
  if (ensureStateStore()) {
    stateStore_.saveCalibration(rtcBsecState.calibrationElapsedSeconds,
                                rtcBsecState.calibrationReady);
  }
}

uint32_t Bme680Driver::currentCalibrationElapsedSeconds() const {
  const uint64_t awakeSeconds =
      (bootElapsedMs() - calibrationAwakeStartedMs_) / 1000ULL;
  return UINT32_MAX - calibrationElapsedAtBeginSeconds_ < awakeSeconds
             ? UINT32_MAX
             : calibrationElapsedAtBeginSeconds_ +
                   static_cast<uint32_t>(awakeSeconds);
}

void Bme680Driver::prepareForDeepSleep(uint32_t seconds) {
  if (rtcBsecState.signature == kRtcSignature) {
    if (!rtcBsecState.calibrationReady) {
      const uint32_t elapsed = currentCalibrationElapsedSeconds();
      rtcBsecState.calibrationElapsedSeconds =
          UINT32_MAX - elapsed < seconds ? UINT32_MAX : elapsed + seconds;
    }
  }
}

uint32_t Bme680Driver::recommendedSleepSeconds(
    uint32_t fallbackSeconds) const {
  if (!scheduleUpdated_ || rtcBsecState.signature != kRtcSignature ||
      rtcBsecState.nextBsecCallLogicalMs == 0) {
    return fallbackSeconds;
  }
  return lil::power::scheduledSleepSeconds(
      scheduleUpdated_, logicalNowMs(), rtcBsecState.nextBsecCallLogicalMs,
      fallbackSeconds);
}

void Bme680Driver::clearPersistentState() {
  rtcBsecState = BsecRtcState{};
  if (ensureStateStore()) {
    stateStore_.clear();
  }
}

}  // namespace sensor

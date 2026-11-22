#include "low_power_wait.h"
#include "measurement_budget.h"

#include <Arduino.h>
#include <atomic>
#include <driver/gpio.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <soc/gpio_reg.h>
#include <soc/soc.h>

#include "hardware_profile.h"

namespace sensor {

namespace {
constexpr uint32_t kMinimumLightSleepUs = 8000;
std::atomic<int32_t> lightSleepInhibitCount{0};

void cooperativeDelayUs(uint32_t periodUs) {
  const int64_t deadlineUs = esp_timer_get_time() + periodUs;
  if (periodUs >= 1000) delay(periodUs / 1000UL);
  // RTOS tick alignment can shorten delay() by nearly one tick.
  const int64_t remainingUs = deadlineUs - esp_timer_get_time();
  if (remainingUs > 0) delayMicroseconds(static_cast<uint32_t>(remainingUs));
}
}  // namespace

void pushLowPowerWaitInhibit() {
  lightSleepInhibitCount.fetch_add(1, std::memory_order_acq_rel);
}

void popLowPowerWaitInhibit() {
  int32_t current = lightSleepInhibitCount.load(std::memory_order_acquire);
  while (current > 0 &&
         !lightSleepInhibitCount.compare_exchange_weak(
             current, current - 1, std::memory_order_acq_rel)) {
  }
}

bool lowPowerWaitAllowed() {
  return lightSleepInhibitCount.load(std::memory_order_acquire) <= 0;
}

void lowPowerSensorWaitUs(uint32_t periodUs) {
  if (periodUs < kMinimumLightSleepUs || !lowPowerWaitAllowed()) {
    cooperativeDelayUs(periodUs);
    return;
  }

  const int64_t startedUs = esp_timer_get_time();
  // On the C3 both gates are in the low GPIO bank. Restore the commanded
  // output latch, even if the loaded V3 supply pin reads low at the pad.
  const uint32_t outputs = REG_READ(GPIO_OUT_REG);
  const int sensorPowerLevel = (outputs >> kHardware.sensorPowerPin) & 1U;
  const bool sensorPowerHeld =
      gpio_hold_en(kHardware.sensorPowerPin) == ESP_OK;
  const bool hasAdcEnable = kHardware.adcEnablePin != GPIO_NUM_NC;
  int adcEnableLevel = 0;
  if constexpr (kHardware.adcEnablePin != GPIO_NUM_NC) {
    adcEnableLevel = (outputs >> kHardware.adcEnablePin) & 1U;
  }
  const bool adcEnableHeld =
      !hasAdcEnable || gpio_hold_en(kHardware.adcEnablePin) == ESP_OK;
  bool timerEnabled = false;
  if (sensorPowerHeld && adcEnableHeld) {
    timerEnabled = esp_sleep_enable_timer_wakeup(periodUs) == ESP_OK;
  }
  if (timerEnabled) {
    (void)esp_light_sleep_start();
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  }

  if (sensorPowerHeld) {
    // Restore the output latch before releasing the pad so neither the direct
    // V3 rail nor the active-low V4 PMOS sees a power glitch.
    gpio_set_level(kHardware.sensorPowerPin, sensorPowerLevel);
    gpio_hold_dis(kHardware.sensorPowerPin);
  }
  if (hasAdcEnable && adcEnableHeld) {
    // GPIO6 can be high while its 100-ms divider settling time overlaps a
    // BME680 conversion. Preserve that state across the same Light-sleep.
    gpio_set_level(kHardware.adcEnablePin, adcEnableLevel);
    gpio_hold_dis(kHardware.adcEnablePin);
  }

  const int64_t elapsedUs = esp_timer_get_time() - startedUs;
  if (elapsedUs < static_cast<int64_t>(periodUs)) {
    cooperativeDelayUs(
        static_cast<uint32_t>(static_cast<int64_t>(periodUs) - elapsedUs));
  }
}

}  // namespace sensor

extern "C" void wchargerBsecLowPowerWait(uint32_t periodUs) {
  sensor::measurementWaitUs(periodUs);
}

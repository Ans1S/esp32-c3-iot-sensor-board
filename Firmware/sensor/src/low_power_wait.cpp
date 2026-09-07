#include "low_power_wait.h"
#include "measurement_budget.h"

#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_sleep.h>
#include <esp_timer.h>

#include "hardware_profile.h"

namespace sensor {

namespace {
constexpr uint32_t kMinimumLightSleepUs = 20000;

void cooperativeDelayUs(uint32_t periodUs) {
  delay(periodUs / 1000UL);
  delayMicroseconds(periodUs % 1000UL);
}
}  // namespace

void lowPowerSensorWaitUs(uint32_t periodUs) {
  if (periodUs < kMinimumLightSleepUs) {
    cooperativeDelayUs(periodUs);
    return;
  }

  const int64_t startedUs = esp_timer_get_time();
  const int sensorPowerLevel = gpio_get_level(kHardware.sensorPowerPin);
  const bool sensorPowerHeld =
      gpio_hold_en(kHardware.sensorPowerPin) == ESP_OK;
  const bool hasAdcEnable = kHardware.adcEnablePin != GPIO_NUM_NC;
  const int adcEnableLevel =
      hasAdcEnable ? gpio_get_level(kHardware.adcEnablePin) : 0;
  const bool adcEnableHeld =
      !hasAdcEnable || gpio_hold_en(kHardware.adcEnablePin) == ESP_OK;
  bool timerEnabled = false;
  if (sensorPowerHeld && adcEnableHeld) {
    timerEnabled = esp_sleep_enable_timer_wakeup(periodUs) == ESP_OK;
  }
  if (timerEnabled) {
    esp_light_sleep_start();
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

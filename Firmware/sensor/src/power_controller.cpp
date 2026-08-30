#include "power_controller.h"

#include <driver/gpio.h>

namespace sensor {

void PowerController::writeActive(gpio_num_t pin, bool activeHigh,
                                  bool enabled) {
  if (pin == GPIO_NUM_NC) {
    return;
  }
  digitalWrite(static_cast<uint8_t>(pin), enabled == activeHigh ? HIGH : LOW);
}

void PowerController::begin() {
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis(kHardware.sensorPowerPin);
  pinMode(static_cast<uint8_t>(kHardware.sensorPowerPin), OUTPUT);
  sensorPower(false);

  if (kHardware.adcEnablePin != GPIO_NUM_NC) {
    gpio_hold_dis(kHardware.adcEnablePin);
    pinMode(static_cast<uint8_t>(kHardware.adcEnablePin), OUTPUT);
    adcPower(false);
  }
}

void PowerController::sensorPower(bool enabled) {
  writeActive(kHardware.sensorPowerPin, kHardware.sensorPowerActiveHigh,
              enabled);
}

void PowerController::adcPower(bool enabled) {
  writeActive(kHardware.adcEnablePin, kHardware.adcEnableActiveHigh, enabled);
}

void PowerController::prepareForDeepSleep() {
  sensorPower(false);
  adcPower(false);
  gpio_hold_en(kHardware.sensorPowerPin);
  if (kHardware.adcEnablePin != GPIO_NUM_NC) {
    gpio_hold_en(kHardware.adcEnablePin);
  }
  gpio_deep_sleep_hold_en();
}

}  // namespace sensor

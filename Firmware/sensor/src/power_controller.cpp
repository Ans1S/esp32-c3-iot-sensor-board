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
  // Load the inactive output latch before enabling the driver. V4's PMOS is
  // active low; the reset latch would briefly turn it on with pinMode first.
  gpio_set_level(kHardware.sensorPowerPin, kHardware.sensorPowerActiveHigh ? 0 : 1);
  pinMode(static_cast<uint8_t>(kHardware.sensorPowerPin), OUTPUT);
  sensorPower(false);
  pinMode(static_cast<uint8_t>(kHardware.sdaPin), INPUT);
  pinMode(static_cast<uint8_t>(kHardware.sclPin), INPUT);

  if (kHardware.adcEnablePin != GPIO_NUM_NC) {
    gpio_hold_dis(kHardware.adcEnablePin);
    gpio_set_level(kHardware.adcEnablePin, kHardware.adcEnableActiveHigh ? 0 : 1);
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
  // Do not back-power the switched sensor rail through I2C pull-ups or pads.
  pinMode(static_cast<uint8_t>(kHardware.sdaPin), INPUT);
  pinMode(static_cast<uint8_t>(kHardware.sclPin), INPUT);

  // GPIO6..21 are digital GPIOs on the ESP32-C3. Holding one of these pads
  // through deep sleep can keep its digital power domain supplied and increase
  // sleep current dramatically. Both board revisions have a safe unpowered
  // state when the control pin becomes high impedance: V3 powers the sensor
  // directly from GPIO10, while V4 biases its PMOS and ADC gates off with
  // external resistors. Let the ESP32-C3 place these pins into their normal
  // high-impedance deep-sleep state instead of retaining the output drivers.
  gpio_hold_dis(kHardware.sensorPowerPin);
  if (kHardware.adcEnablePin != GPIO_NUM_NC) {
    gpio_hold_dis(kHardware.adcEnablePin);
  }
  gpio_deep_sleep_hold_dis();
}

}  // namespace sensor

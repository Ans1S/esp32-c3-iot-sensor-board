#pragma once

#include <Arduino.h>

#ifndef PCB_VERSION
#error "PCB_VERSION must be supplied by the PlatformIO environment"
#endif

namespace sensor {

struct HardwareProfile {
  uint8_t pcbVersion;
  gpio_num_t adcPin;
  gpio_num_t adcEnablePin;
  gpio_num_t sensorPowerPin;
  gpio_num_t sdaPin;
  gpio_num_t sclPin;
  bool sensorPowerActiveHigh;
  bool adcEnableActiveHigh;
  float batteryDividerScale;
  uint16_t adcSettleMs;
};

#if PCB_VERSION == 3
constexpr HardwareProfile kHardware{
    3, GPIO_NUM_3, GPIO_NUM_NC, GPIO_NUM_10, GPIO_NUM_5, GPIO_NUM_4,
    // Keep the field-proven V3 scaling. It intentionally takes precedence over
    // the nominal schematic resistor ratio for this board revision.
    true, true, 167.0F / 100.0F, 4};
#elif PCB_VERSION == 4
constexpr HardwareProfile kHardware{
    4, GPIO_NUM_3, GPIO_NUM_6, GPIO_NUM_10, GPIO_NUM_5, GPIO_NUM_4,
    // V4 places 100 kOhm in series with the ADC and 100 nF from the ADC input
    // to ground. Together with the 100 kOhm / 150 kOhm divider, the charging
    // time constant is about 16 ms. Wait for more than six time constants so
    // the first battery reading is within about 0.2% of its final value.
    false, true, 5.0F / 3.0F, 100};
#else
#error "Only PCB_VERSION 3 and 4 are supported"
#endif

}  // namespace sensor

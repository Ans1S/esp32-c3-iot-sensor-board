#pragma once
#include <Arduino.h>
inline int gpio_set_level(gpio_num_t pin, int level) { padLatch[pin] = level; return 0; }
inline int gpio_hold_dis(gpio_num_t) { return 0; }
inline void gpio_deep_sleep_hold_dis() {}

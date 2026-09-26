#pragma once
#include <stdint.h>

// PCB V4 uses the requested 2.8 V battery protection threshold by default.
// V3 retains its existing policy unless a threshold is supplied explicitly.
#ifndef SENSOR_STORAGE_DISCOVERY_MAX_SECONDS
#define SENSOR_STORAGE_DISCOVERY_MAX_SECONDS 300
#endif
#ifndef SENSOR_LOW_BATTERY_PAUSE_MV
#if PCB_VERSION == 4
#define SENSOR_LOW_BATTERY_PAUSE_MV 2800
#else
#define SENSOR_LOW_BATTERY_PAUSE_MV 0
#endif
#endif
#ifndef SENSOR_LOW_BATTERY_RESUME_MARGIN_MV
#define SENSOR_LOW_BATTERY_RESUME_MARGIN_MV 150
#endif
constexpr uint32_t kBatteryProtectionSleepSeconds = 24UL * 60UL * 60UL;
static_assert(SENSOR_STORAGE_DISCOVERY_MAX_SECONDS >= 300 &&
              SENSOR_STORAGE_DISCOVERY_MAX_SECONDS <= 86400);
static_assert(SENSOR_LOW_BATTERY_PAUSE_MV == 0 ||
              (SENSOR_LOW_BATTERY_PAUSE_MV >= 2000 &&
               SENSOR_LOW_BATTERY_PAUSE_MV + SENSOR_LOW_BATTERY_RESUME_MARGIN_MV <= 4200));

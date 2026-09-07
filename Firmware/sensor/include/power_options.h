#pragma once

// Opt-in board experiments. Defaults preserve discovery responsiveness and do
// not guess a safe discharge voltage for the user's cells/protection circuit.
#ifndef SENSOR_STORAGE_DISCOVERY_MAX_SECONDS
#define SENSOR_STORAGE_DISCOVERY_MAX_SECONDS 300
#endif
#ifndef SENSOR_LOW_BATTERY_PAUSE_MV
#define SENSOR_LOW_BATTERY_PAUSE_MV 0
#endif
#ifndef SENSOR_LOW_BATTERY_RESUME_MARGIN_MV
#define SENSOR_LOW_BATTERY_RESUME_MARGIN_MV 150
#endif
static_assert(SENSOR_STORAGE_DISCOVERY_MAX_SECONDS >= 300 &&
              SENSOR_STORAGE_DISCOVERY_MAX_SECONDS <= 86400);
static_assert(SENSOR_LOW_BATTERY_PAUSE_MV == 0 ||
              (SENSOR_LOW_BATTERY_PAUSE_MV >= 2000 &&
               SENSOR_LOW_BATTERY_PAUSE_MV + SENSOR_LOW_BATTERY_RESUME_MARGIN_MV <= 4200));

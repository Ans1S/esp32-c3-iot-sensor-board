# Power management in firmware 4.3.1

## Default behavior

| Mode | Measurement and radio behavior | Between measurements |
| --- | --- | --- |
| BME280 | Forced measurement with 1x oversampling; battery and radio at the configured report interval | Sensor rail off, Wi-Fi off, ESP32-C3 deep sleep |
| BME680 | BSEC ULP maintenance approximately every 300 seconds; V4 checks the battery first on every wake; radio only when a report is due | Sensor rail off and deep sleep until the next maintenance/report deadline |
| Battery only | ADC and radio at the configured report interval | Sensor rail off, Wi-Fi off, deep sleep |
| LSM6DSOX | Continuous 104 Hz FIFO acquisition, interval summaries sent at the selected interval | CPU and sensor remain active to retain motion events |
| Discovery | Bounded channel scan; ten-second wakes initially, five-minute wakes after ten minutes | Deep sleep |
| OTA | Download while awake only when an update is offered; bounded retries and session duration | Return to scheduled operation after completion/recovery |
| V4 battery protection | Below 2.8 V or invalid ADC: no external measurements, one bounded battery reporting window every 24 hours; resume at 2.95 V | Sensor/ADC rails off, Wi-Fi off, deep sleep |

Normal builds retain the 80 MHz CPU setting. `SENSOR_DIAGNOSTIC_LOGGING` and
`CORE_DEBUG_LEVEL` default to zero; temporary OTA stage logging is removed.
Experimental CPU/compiler settings remain in `sensor/platformio-experiments.ini`.
Normal V4 builds enable battery protection; V3 retains its opt-in policy.
See [review, limits and hardware checks](SENSOR_REVIEW.md).

## Energy audit against the preceding repository release

Above the protection threshold, environmental nodes retain their deep-sleep
cadence. `main.cpp` enters live mode for a successfully started LSM6DSOX,
TMP117 or MAX30102 on a provisioned node; SW2 controls manual recording.
`EnvironmentalSensor::end()` releases I2C and turns sensor power off;
`SleepController::deepSleep()` applies GPIO shutdown/holds, disables Wi-Fi and
starts timer deep sleep. V3/V4 retain their opposite GPIO10 polarities. V4's
ADC divider is gated; V3's divider is physically permanent.

The update functionality does add bounded overhead: a version/offer exchange
at an existing successful reporting contact, and a read of OTA state. It does
not add periodic wakeups solely to poll for updates. The extra five-second
telemetry reply window and 60-second boot guard apply only to an unconfirmed
OTA trial boot. The ten-minute session allowance applies to an actual transfer,
not every wake. Identical battery life to a build without OTA is therefore not
claimed.

BME680 maintenance wakes do not silently increase radio frequency, including
after failed reports. Measurement work has a six-second budget, fetch retries
are bounded, I2C has a timeout, and logical time includes awake and sleep time.
Conversion and ADC settling waits use GPIO-preserving light sleep where safe.
V4's 100 ms divider settling now precedes sensor initialization, including
maintenance wakes, so undervoltage cannot power the sensor first. Live V4
operation checks the battery every ten seconds. V3 retains its overlapping
report-time ADC path. Channel recovery is bounded and is disabled in the
daily battery-protection path.

Existing reporting settings are retained. Selecting one-second reports increases
wake/radio frequency. Dashboard refresh alone does not change a node's saved
report interval. Continuous LSM6DSOX acquisition is intentionally more demanding
than environmental deep sleep, even with a long reporting interval. One-second
motion summaries are supported by the protocol; manual sessions currently
report every 50 ms for five minutes and every 100 ms thereafter. See
[LSM6DSOX](LSM6DSOX.md) for FIFO limits and history semantics.

## Verification and limits

[Host checks](tests/README.md) cover report scheduling, failed-report cadence,
logical time, measurement budgets, ADC calculations, radio retry bounds and OTA
trial-boot behavior. All three normal targets must build. These tests establish
software behavior; they do not measure current on a physical board.

For a comparable battery test, measure integrated charge per reporting cycle
and deep-sleep current on both revisions with identical battery, sensor,
interval, RF conditions and calibration. Include a no-update contact, offline
station recovery, BME680 maintenance-only wake and an actual OTA separately.
Use the [hardware test plan](HARDWARE_TESTPLAN.md). No measured battery-life
regression or improvement is claimed without those measurements.

## Station behavior

Fast history uses a bounded 900-entry RAM ring instead of a flash write for
every subminute report. Restart clears that fast history; intervals of at least
60 seconds retain persistent history. Radio callbacks hand off persistence to a
bounded queue. ThingSpeak scheduling retains per-channel limits, fair service,
bounded retries and queue expiry. These changes target flash wear and station
responsiveness; the station is not the battery-powered sensor.

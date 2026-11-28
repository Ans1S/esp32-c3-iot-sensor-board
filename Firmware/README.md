# W-Charger Firmware

This directory contains the only maintained firmware in this repository. The
station and both sensor-board revisions share the version 5 ESP-NOW protocol;
retired experiments are intentionally excluded from Git.

```text
Firmware/
├── shared/   versioned ESP-NOW protocol
├── station/  ESP32-S3 home station
└── sensor/   ESP32-C3 sensor firmware for PCB V3 and V4
```

The station and sensor are independent PlatformIO projects. Flash the station
first, then the sensor target matching PCB V3 or V4. One sensor codebase
supports BMP280, BME280, BME680, BME688, CCS811, VL53L1X, AS7341, LSM6DSOX, TMP117, MAX30102 and battery-only nodes; the attached
sensor is selected in the station UI rather than compiled into a separate
image. Configure Wi-Fi, reporting intervals, sensor calibration and optional
ThingSpeak integration only through the station UI. Never add credentials to
source files.

For the shortest upload path, run `python Firmware/upload.py` from the
repository root and select the connected board. Pass `station`, `sensor-v3` or
`sensor-v4` directly to make the command non-interactive. Add `--build-only` to
verify a target without writing to a connected device.

- [Station](station/README.md)
- [Sensor](sensor/README.md)
- [Architecture](ARCHITECTURE.md)
- [Hardware test plan](HARDWARE_TESTPLAN.md)

The current full release is **4.4.11 / 40411**, protocol V5. The
[release archive](releases/README.md) lists the complete bundle, both signed
sensor OTA packages and the application-only station image.
For an installed station, `python Firmware/upload.py station --application-only`
updates only its application and preserves Wi-Fi, sensors, settings and files.
Initialize the installation signing key as described
in the OTA guide before the first build. Never replace an existing signing key.

- [Signed OTA updates](OTA.md)
- [Release packages](releases/README.md)
- [LSM6DSOX motion acquisition](LSM6DSOX.md)
- [Power management](POWER_MANAGEMENT.md)
- [Regression checks](tests/README.md)
- [Manual recordings and USB migration](RECORDINGS.md)
- [Sensor accuracy review and V4 battery protection](SENSOR_REVIEW.md)
- [Station and sensor V3/V4 reliability review](RELIABILITY_REVIEW.md)
- [Real-time and long-uptime review](REALTIME_REVIEW.md)
- [Storage performance review](PERFORMANCE_REVIEW.md)
- [Mobile usability, long-uptime fixes and storage capacity](RESOURCE_AND_UX_REVIEW.md)

## Live precision sensors

TMP117 and LSM6DSOX take fresh normal measurements at the configured interval
with the external rail off between windows. MAX30102 starts through SW2 only.
SW2 / GPIO9 starts/stops a separate fast recording: 1 s TMP117 values, 200 ms
optical waveform packets, or 50 ms IMU means for five minutes and 100 ms afterward.
Pulse estimates update once per second. Saved recordings load as complete,
stable snapshots; full-session CSV retains the original samples. See [operation, accuracy limits and OTA migration](PRECISION_SENSORS.md).


## Changes since 4.4.5

- **Five new I²C sensors** cover pressure (BMP280), indoor air quality (BME688 and CCS811), distance (VL53L1X) and spectral light (AS7341). Both PCB revisions use the same driver set.
- **Sensor nodes** reject repeated Bosch factory calibration data, retry failed acquisitions within a bounded window and recover the radio channel after a link change. The V4 low-battery cutoff also applies to continuous CCS811 operation.
- **Station** displays the new measurements, preserves compatible older latest-reading records during upgrades and caches history pages. CSV exports retain the underlying samples.
- **Sensor updates** can use 1,024-byte data blocks on compatible links, fall back to 192 bytes during a transfer and retry the handshake after a station restart.

[Change inventory](CHANGELOG.md) · [Publication checks](PUBLICATION_REVIEW.md)

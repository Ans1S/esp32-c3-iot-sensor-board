# W-Charger Firmware

This directory contains the only maintained firmware in this repository. The
station and both sensor-board revisions share the version 5 ESP-NOW protocol;
retired experiments are intentionally excluded from Git.

```text
Firmware/
â”œâ”€â”€ shared/   versioned ESP-NOW protocol
â”œâ”€â”€ station/  ESP32-S3 home station
â””â”€â”€ sensor/   ESP32-C3 sensor firmware for PCB V3 and V4
```

The station and sensor are independent PlatformIO projects. Flash the station
first, then the sensor target matching PCB V3 or V4. One sensor codebase
supports BME280, BME680, LSM6DSOX and battery-only nodes; the attached environmental
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

Current firmware: **4.1.3**. Initialize the installation signing key as described
in the OTA guide before the first build. Never replace an existing signing key.

- [Signed OTA updates](OTA.md)
- [Latest local packages](releases/README.md)
- [LSM6DSOX motion acquisition](LSM6DSOX.md)
- [Power management](POWER_MANAGEMENT.md)
- [Regression checks](tests/README.md)

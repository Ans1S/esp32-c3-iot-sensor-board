# W-Charger Firmware

This directory contains the only maintained firmware in this repository. The
station and both sensor-board revisions share the version 4 ESP-NOW protocol;
retired experiments are intentionally excluded from Git.

```text
Firmware/
├── shared/   versioned ESP-NOW protocol
├── station/  ESP32-S3 home station
└── sensor/   ESP32-C3 sensor firmware for PCB V3 and V4
```

The station and sensor are independent PlatformIO projects. Flash the station
first, then the sensor target matching PCB V3 or V4. One sensor codebase
supports BME280, BME680 and battery-only nodes; the attached environmental
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

# W-Charger Firmware

This is the only maintained firmware in this repository.

```text
Firmware/
├── shared/   versioned ESP-NOW protocol
├── station/  ESP32-S3 home station
└── sensor/   ESP32-C3 sensor firmware for PCB V3 and V4
```

Both projects are independent PlatformIO projects. Flash the station first,
then the matching sensor target. Configure Wi-Fi and ThingSpeak only through
the station UI; never add credentials to source files.

For the shortest upload path, run `python Firmware/upload.py` from the
repository root and select the connected board. Pass `station`, `sensor-v3` or
`sensor-v4` directly to make the command non-interactive.

- [Station](station/README.md)
- [Sensor](sensor/README.md)
- [Architecture](ARCHITECTURE.md)
- [Hardware test plan](HARDWARE_TESTPLAN.md)

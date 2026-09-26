# Firmware 4.3.1

The current sensor release is **4.3.1** (release integer **40304**) for PCB V3
and V4. This directory publishes the latest signed OTA packages:

| File | Target |
| --- | --- |
| `sensor-v3-4.3.1.ota` | PCB V3 |
| `sensor-v4-4.3.1.ota` | PCB V4 |
| `sensor-v3-4.3.1.factory.bin` | PCB V3, fresh USB installation at offset `0x0` |
| `sensor-v4-4.3.1.factory.bin` | PCB V4, fresh USB installation at offset `0x0` |
| `station-4.3.1.factory.bin` | Station, fresh USB installation at offset `0x0` |

Factory images include the bootloader, partition table and application. Back up
the device first, then erase flash before a fresh installation; this clears
settings, pairing and history. Writing a factory image alone does not erase
the new recording region. For an NVS-preserving migration, follow the separate
component workflow in [the migration guide](../RECORDINGS.md).

Build and sign release packages using the [OTA guide](../OTA.md). They require
the installation key already trusted by both the station and sensor. A newly
generated key cannot update an existing installation. `SHA256SUMS.txt`
identifies the published packages; signatures are verified by the packager and
both devices. The 4.1.3 packages remain available for the previous release; they are not the current update.

The internal release increases from the local 4.3.0 build's 40303 to 40304.
This permits updating installations that already run that recording build.

## Changes

- PCB V4 battery protection below 2.8 V: sensor power off, battery-only reports
  every 24 hours using deep sleep, and recovery at 2.95 V. The station displays
  the protected state and retains daily battery history.
- Checked BME280 register transfers, coherent compensation, conservative
  100 kHz Bosch startup and safe power-gate/I2C shutdown.
- Datasheet-reviewed precision drivers with explicit FIFO gaps, ambient-light
  overflow handling, brownout recovery and rejection of partial I2C reads.
  See [sensor review and hardware acceptance](../SENSOR_REVIEW.md).
- Quick feedback above sensor graphs: estimated steps/cadence/active time,
  temperature stability/rate, and pulse quality/statistics/recovery.
  See [feedback definitions](../SENSOR_FEEDBACK.md). Update the station first;
  an existing 40302 recording partition layout remains compatible.

- TMP117 and MAX30102 drivers with accuracy-oriented initialization and signal validation.
- Sensor-timed updates: 50/100 ms LSM6DSOX session means, data-ready TMP117 reports,
  200 ms optical waveform packets and 1 s pulse estimates.
- SW2 start/stop, at least 30-minute sensor recording capacity, durable synchronization and saved station sessions.
- One-minute live/saved graphs and full-session CSV export. Normal BME sleep
  scheduling is retained; V4 low battery overrides it with daily reporting.
- One-time USB partition migration on sensor and station; see [recording migration](../RECORDINGS.md).
- See [precision sensor operation and validation](../PRECISION_SENSORS.md).

- LSM6DSOX I2C support on both PCBs, continuous 104 Hz acquisition, interval
  peaks and a fixed 100 ms live dashboard cadence.
- Signed ESP-NOW updates with 4 KiB buffered writes, 16 KiB durable resume
  checkpoints, a ten-minute transfer budget, and explicit failure status.
- Inclusive OTA minimum voltage: PCB V3 3350 mV; PCB V4 2800 mV.
- Original V5 environmental packet length, a five-second reply retry window
  only during trial boot, and a 60-second boot guard with automatic rollback.
- Temporary trial-boot serial diagnostics removed; optional normal logging
  remains disabled. Bounded measurements, power gating and deep sleep remain
  active for environmental sensors. See [power management](../POWER_MANAGEMENT.md).
- Bounded station persistence/upload queues and resilient update-page polling.

## Compatibility and validation

Install the new partition tables once by USB before using manual recordings.
Use the current station firmware for motion configuration and improved OTA
status reporting. Environmental telemetry retains compatibility with the
original V5 packet prefix. Updating a running old client still uses that old
client's battery threshold and transfer timeout until the new image boots.
Same-version OTA packages are rejected.

Both sensor targets and the station build with PlatformIO. The host and browser
checks are documented in [tests](../tests/README.md). Hardware RF, power-loss,
bootloader rollback and current measurements remain separate acceptance work;
no battery-life or transfer-duration benchmark is implied by these checks.

The application includes Bosch BSEC2 binaries under their separate
[license terms](https://github.com/boschsensortec/Bosch-BSEC2-Library/blob/4f559a6aa450f0ce436e602b42dc384f52128c66/LICENSE.md).
The repository license does not relicense third-party components. Review the
applicable terms before redistributing a compiled package.

# Firmware 4.1.3

The current sensor release is **4.1.3** (release integer **40103**) for PCB V3
and V4. This directory keeps only the latest locally built OTA packages:

| File | Target |
| --- | --- |
| `sensor-v3-4.1.3.ota` | PCB V3 |
| `sensor-v4-4.1.3.ota` | PCB V4 |

Binary packages are excluded from Git. Build and sign them using the
[OTA guide](../OTA.md); they require the installation key already trusted by
both the station and sensor. A newly generated key cannot update an existing
installation. `SHA256SUMS.txt` identifies the current local packages; signatures
are verified by the packager and both devices. Factory images and older
packages are not part of this release directory.

## Changes

- LSM6DSOX I2C support on both PCBs, continuous 104 Hz acquisition, interval
  peaks and local dashboard/history intervals down to one second.
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

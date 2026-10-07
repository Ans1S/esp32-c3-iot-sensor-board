# Firmware 4.3.4

The current release is **4.3.4**, internal release **40307**, for protocol V5.

| File | Target | Use |
| --- | --- | --- |
| [sensor-v3-4.3.4.ota](sensor-v3-4.3.4.ota) | Sensor PCB V3 | Signed OTA through the station |
| [sensor-v4-4.3.4.ota](sensor-v4-4.3.4.ota) | Sensor PCB V4 | Signed OTA through the station |
| [station-4.3.4.bin](station-4.3.4.bin) | ESP32-S3 station | Application-only USB image at `0x10000` |

The release includes bounded sensor reinitialization, BME680 learning-state
persistence, MAX30102 weak-signal/independent LED control, durable settings and
archive fixes, stable complete saved graphs, timed setup notices and correct
BME280 tiles. Read [the full change inventory](../CHANGELOG.md) for the main
improvements and every additional correction since 4.3.1 and PR #30.

## Installation

Update the station for UI/storage changes and sensor nodes for driver/recovery
changes. To build and upload the station normally, run from the repository root:

```sh
python Firmware/upload.py station
```

Normal station uploads preserve settings, identities and recording archives.
The supplied station BIN contains only the application. It must not be flashed
at `0x0`, uploaded as a sensor OTA package or mistaken for a factory image.
On an existing compatible layout, write it at `0x10000` without erasing flash.
Only the explicit factory-reset environment intentionally erases the device.

Upload the matching V3/V4 `.ota` on the station's **Firmware updates** page.
Existing 4.3.1/4.3.2/4.3.3 installations with this trusted signing identity and
recording layout can use it. Same-version OTA is rejected; rebuilding 4.3.4
with sanitized compiler paths does not change its release integer. Older
layouts still need the [one-time USB migration](../RECORDINGS.md#one-time-usb-migration).
The recording partition layout and low-power policies remain unchanged.

## Integrity and publication

[SHA256SUMS.txt](SHA256SUMS.txt) covers all three files. V3/V4 signatures,
embedded PCB/protocol/version identities and payloads match the final builds
and the existing installation's public key. That private key and the local
public header are never uploaded. Firmware intentionally contains the public
verification key; that is not private key material.

New builds remap personal compiler paths before compilation. The publication
checker inspects firmware bytes, source text and ZIP contents with redacted
findings. Earlier distribution binaries were removed from this current tree
because they carried local build paths; retained local copies are ignored.
Their previously published Git objects are still present in history. See
[the publication review](../PUBLICATION_REVIEW.md) for this limitation.

This directory supplies only the current packages, not flash dumps or factory
images. The unpublished 4.3.2/4.3.3 intermediate packages are superseded by 4.3.4.
Historical 4.3.1 behavior and validation are documented in
[PR #30](https://github.com/Ans1S/esp32-c3-iot-sensor-board/pull/30).

## Validation and licenses

All three PlatformIO targets, 22 production C++ suites, six embedded browser
suites, six cryptographic OTA package tests, two upload tests, three pinned
library-patch checks and publication regressions pass. Eight station script
blocks parse. No hardware was flashed or measured. Accuracy, RF/power-loss,
rollback behavior, rail discharge and current acceptance remain separate work;
see [tests](../tests/README.md) and [the reliability review](../RELIABILITY_REVIEW.md).

BME680 firmware includes Bosch BSEC2 binaries under their separate
[license terms](https://github.com/boschsensortec/Bosch-BSEC2-Library/blob/4f559a6aa450f0ce436e602b42dc384f52128c66/LICENSE.md).
The repository license does not relicense third-party components. Review the
applicable terms before redistributing compiled packages.

# Firmware packages

## Current full release 4.4.1

Station and PCB V3/V4 use **4.4.1 / 40401**, protocol V5. This full release
contains every current firmware change, including the complete 4.4.0 dashboard,
normal-history and long-uptime work, earlier sensor recovery fixes, and the
latest audited interaction/reliability corrections. Existing partition layouts,
settings schemas and the installation signing identity are preserved.

| File | Target | Installation |
| --- | --- | --- |
| [Complete update bundle](w-charger-4.4.1.zip) | All three targets | Firmware, instructions, release notes and SHA-256 checksums |
| [sensor-v3-4.4.1.ota](sensor-v3-4.4.1.ota) | Sensor PCB V3 | Signed OTA through the station |
| [sensor-v4-4.4.1.ota](sensor-v4-4.4.1.ota) | Sensor PCB V4 | Signed OTA through the station |
| [station-4.4.1.bin](station-4.4.1.bin) | ESP32-S3 station | Application-only USB at `0x10000`, without full erasure |
| [Installation instructions](INSTALL-4.4.1.txt) | Existing compatible recording layout | Station first, then the matching sensor OTA |
| [Release notes](RELEASE-NOTES-4.4.1.txt) | All three targets | Included changes, preserved behavior and validation limits |

The new release integer permits an OTA update from installed 4.4.0 nodes.
Use [4.4.1 checksums](SHA256SUMS-4.4.1.txt) for its six individual download
assets, including the complete ZIP. The unversioned manifest also covers older
retained packages; each ZIP includes a checksum manifest for its own contents.
The station itself is updated by USB; its web Firmware updates page accepts
sensor OTA packages. The station BIN is an application, not a sensor package
or a factory image. Use `0x10000` without full erasure on the existing compatible
layout; older layouts still require the migration in [recordings](../RECORDINGS.md).

The station corrections prevent expired live points from returning after a
full millis wrap, preserve a reopened layout draft when an older save finishes,
show retryable history timeouts without replacing complete data, and repair
dashboard error handling during disconnection/recovery. See the
[change inventory](../CHANGELOG.md) for the complete release history.

All three final targets build. All 24 production C++ suites and eight embedded
browser suites pass, including interaction/fault scenarios with 19 synthetic
nodes. OTA cryptography, library-patch, upload-preservation, script-syntax and
publication regression checks also pass. The existing key matches the trusted
public header and five previous packages. New signatures, PCB/protocol/release
identities and payloads match their final compiled images. ESP image checksums,
partition layouts, ZIP members and all manifest checksums were verified.
The final publication scan includes binary and ZIP contents. No hardware was
flashed or physically measured during preparation.

| Build | Static RAM bytes | Linked flash bytes / slot bytes |
| --- | ---: | ---: |
| Station S3 | 108,244 | 1,595,746 / 2,359,296 |
| Sensor PCB V3 | 45,908 | 1,076,317 / 1,310,720 |
| Sensor PCB V4 | 45,908 | 1,077,149 / 1,310,720 |

These are linked static sections, not runtime free-heap measurements. Older
versioned packages below retain their original bytes and checksums.

## Previous full release 4.4.0

Station and PCB V3/V4 use **4.4.0 / 40400**, protocol V5. The update preserves
the existing partition tables, Wi-Fi/sensor settings formats and installation
signing identity. No device was flashed during preparation.

| File | Target | Installation |
| --- | --- | --- |
| [Complete update bundle](w-charger-4.4.0.zip) | All three targets | Contains the files below, instructions and checksums |
| [sensor-v3-4.4.0.ota](sensor-v3-4.4.0.ota) | Sensor PCB V3 | Signed OTA through the station |
| [sensor-v4-4.4.0.ota](sensor-v4-4.4.0.ota) | Sensor PCB V4 | Signed OTA through the station |
| [station-4.4.0.bin](station-4.4.0.bin) | ESP32-S3 station | Application-only USB at `0x10000`, without full erasure |
| [Installation instructions](INSTALL-4.4.0.txt) | Installed compatible layout | Preserves Wi-Fi, settings, sensors and files |

The compact overview opens details from the whole card or the selected
measurement from its entire tile. Dashboard order/visibility/columns/spacing
are saved on the station. Automatic orange setup notices are removed. Normal
history adapts to interval, sensor count and available space, while manual
recordings remain independent. Only the opened node is paged; large charts
reduce display geometry while retaining raw inspection and CSV. See
[changes](../CHANGELOG.md) and [storage limits](../RESOURCE_AND_UX_REVIEW.md).

For an existing compatible station, run
`python Firmware/upload.py station --application-only`, or use the supplied BIN
at `0x10000`. This target omits bootloader/partition/OTA-metadata images and
full erasure. It uses the unchanged settings schemas; dashboard layout uses a
separate NVS key. The station is updated by USB, and its web OTA page handles
the two sensor packages.

All 23 production C++ suites, seven browser suites, eight script parse checks,
six cryptographic package tests, four upload tests, three library-patch checks
and five publication regressions pass. All three PlatformIO targets build.
Signatures match the existing public header and three prior trusted packages;
new payloads match the final build files byte for byte. ZIP members and the
checksum manifest were verified. The worktree scan includes firmware/ZIP
contents and reports no findings. Hardware acceptance remains separate.

| Build | Static RAM bytes | Linked flash bytes / slot bytes |
| --- | ---: | ---: |
| Station S3 | 108,084 | 1,595,138 / 2,359,296 |
| Sensor PCB V3 | 45,908 | 1,076,317 / 1,310,720 |
| Sensor PCB V4 | 45,908 | 1,077,149 / 1,310,720 |

These are linked static sections, not runtime free-heap measurements.


## PCB V4 maintenance update 4.3.5

[sensor-v4-4.3.5.ota](sensor-v4-4.3.5.ota) is the previous PCB V4 maintenance package,
internal release **40308**, for protocol V5. It adds bounded recovery after
BME280 conversion failures even when initialization succeeded. See the
[change inventory](../CHANGELOG.md) and [investigation](../SENSOR_REVIEW.md).

Upload this signed package through the station's **Firmware updates** page and
select the PCB V4 node. Station firmware 4.3.4 supports this update; no station
update is required. The package retains the installation signing identity and
recording partition layout. Optional serial diagnostics remain disabled in
this normal build. No board was flashed or measured during preparation.

The checksum manifest below includes this package and the previous full release.
The V4 build, all 22 production host suites and six cryptographic package tests
pass. The new signature verifies against the existing trusted public header,
both prior 4.3.4 packages verify with that identity, and the new payload matches
the final V4 application byte for byte. The worktree publication scan, including
firmware and ZIP contents, reports no findings.

## Previous full release 4.3.4

The previous full release is **4.3.4**, internal release **40307**, for protocol V5.

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

[SHA256SUMS.txt](SHA256SUMS.txt) covers the supplied files. V3/V4 signatures,
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

This directory supplies versioned packages, not flash dumps or factory
images. The unpublished 4.3.2/4.3.3 intermediate packages are superseded by 4.3.4.
Historical 4.3.1 behavior and validation are documented in
[PR #30](https://github.com/Ans1S/esp32-c3-iot-sensor-board/pull/30).

## Validation and licenses

All three PlatformIO targets, 24 production C++ suites, eight embedded browser
suites, six cryptographic OTA package tests, four upload tests, three pinned
library-patch checks and publication regressions pass. Eight station script
blocks parse. No hardware was flashed or measured. Accuracy, RF/power-loss,
rollback behavior, rail discharge and current acceptance remain separate work;
see [tests](../tests/README.md) and [the reliability review](../RELIABILITY_REVIEW.md).

BME680 firmware includes Bosch BSEC2 binaries under their separate
[license terms](https://github.com/boschsensortec/Bosch-BSEC2-Library/blob/4f559a6aa450f0ce436e602b42dc384f52128c66/LICENSE.md).
The repository license does not relicense third-party components. Review the
applicable terms before redistributing compiled packages.

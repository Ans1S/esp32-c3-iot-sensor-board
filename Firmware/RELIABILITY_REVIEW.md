# Station and sensor V3/V4 reliability review

Reviewed on 2026-10-07 against the current working tree, the pinned dependencies,
manufacturer documentation, production host regressions and embedded browser
pages. The existing acquisition, recording, chart and upload changes were
preserved. This report describes the additional reliability corrections from
this review. It does not certify physical accuracy or measured battery life.

## Review coverage

The review covered both `sensor_pcb_v3` and `sensor_pcb_v4`, the ESP32-S3 station,
and their shared protocol. Areas included startup/shutdown, battery protection,
sleep scheduling, ADC calibration, I2C detection and transfer failures, sensor
freshness, FIFO discontinuities, BSEC learning/state persistence, live acquisition
and SW2 transitions, recording storage/replay, radio correlation/retries,
station settings and history, archive durability, network recovery, cloud upload
queues, HTTP input validation, asynchronous browser actions, CSV export, signed
OTA packages and flash-slot limits.

The source review also checked Wi-Fi reconnect/recovery, DNS/mDNS, bounded radio
queues, adaptive transmit power, ThingSpeak HTTPS/timeouts/rate limits, OTA
signature/digest/target checks, checkpoints and trial-boot rollback. No hardware
was flashed and no live cloud account was modified during verification.

## Confirmed sensor and power corrections

| Area | Failure | Correction |
| --- | --- | --- |
| BME680 persistence | At accuracy zero, the initial timestamp prevented the first periodic learning-state save indefinitely. | Start the six-hour save timer at state initialization. Keep RTC retention and accuracy-improvement saves; do not write flash on every measurement. |
| BME680 calibration | Orphaned or rejected learning state could retain a misleading calibrated/Ready status. | Restore metadata only with valid state; reset readiness, elapsed learning and retained IAQ when BSEC rejects it. |
| BME680 output | Invalid IAQ with positive accuracy could complete calibration; raw gas could be advertised with an unstable heater. | Require valid fresh BSEC IAQ for readiness. Raw gas requires both gas-valid and heater-stable status; temperature, pressure and humidity remain independent. |
| BSEC clock | Reinitializing the wrapper after a 32-bit clock wrap could apply its overflow count twice. | Reinitialize the wrapper before allocating its existing memory and seeding the absolute logical clock. |
| MAX30102 freshness | FIFO/ambient-light overflow or a polling gap could revive an old ADC sample after clearing the FIFO. | Remove optical validity and queued waveform data until a real new frame arrives. Existing partial-frame/brownout handling remains fail-closed. |
| LSM6DSOX freshness | A scheduling stall could date old FIFO backlog at drain time. | Discard the incomplete window and FIFO at the existing 200 ms limit. Mark the gap independently of hardware overflow; reject counts above 512 uncompressed entries. |
| Live acquisition | A stop or profile change during yielding initialization/read could continue the cancelled operation. | Recheck the request after those operations and release I2C and sensor power before continuing. |
| Bosch selection | Selecting BME280 at `0x77` failed when BME680 answered first at `0x76`, and vice versa. | Search both Bosch addresses for the requested type. Automatic detection retains its order. |
| Failed initialization | Some failed `begin()` paths left I2C pads and the external rail active. | All failed environmental initialization paths release the bus/pads and turn the rail off immediately. |
| Sensor settings | An NVS write failure could expose and acknowledge the new configuration revision. | Validate and persist the proposed settings before replacing the runtime configuration or its acknowledged revision. |
| IAQ/factory reset | Reset success was not checked, and identical responses could erase the learned baseline repeatedly. | Propagate durable-clear success, retain learning on clear failure, and ignore already-applied explicit IAQ-reset revisions. Complete required state clearing before committing/acknowledging the new revision. |
| Sensor configuration migration | Matching blob size alone could accept a wrong legacy identity/version; a partial read could poison the RTC cache with defaults. | Read once, require matching schema identity and size, and reject partial reads. RTC also records whether the cached configuration was actually persisted, so a failed migration save remains retryable. |
| Live battery status | A cached telemetry packet retained obsolete battery capability/error bits after a new ADC result. | Replace voltage, battery capability and battery-failure flag together while preserving sensor flags. |

The reset sequence deliberately favors honest acknowledgement: if clearing IAQ
succeeds but the subsequent configuration write fails, a later retry may clear
the already-reset state again. The old revision remains unacknowledged until all
required durable operations succeed. No unsuccessful reset is acknowledged.

## Recording, station and network corrections

| Area | Failure | Correction |
| --- | --- | --- |
| Recording duration | The reported duration kept growing after stop, and repeated stops could extend it again. | Freeze duration on the first stop; make subsequent stops idempotent, including after synchronization. |
| Recording UTC | First station contact after stop could anchor the start using the frozen duration and a later wall clock. | Use the actual monotonic request time for late anchoring while keeping the public duration frozen. Preserve unknown UTC after reboot. |
| Recording ACK | Repeated session/sample/checksum fields allowed a delayed response to match a new status request. | Also require the current request sequence, as already done for OTA responses. |
| Station filesystem | A transient/corrupt mount failure could automatically format an existing recording archive. | Mount without automatic formatting. Initialize only after reading the complete filesystem partition and proving every byte is erased. Preserve nonblank/unreadable storage and expose its unavailable status. |
| Station sensor settings | Failed saves could publish unpersisted intervals, provisioning or reset commands to radio traffic. | Stage individual proposals until NVS succeeds. Preserve command acknowledgements received during storage operations. |
| Cloud profiles | Failed profile synchronization/removal left changed local configuration active. | Restore the previous local state and attempt durable rollback. HTTP errors distinguish a successful remote change from failed local storage. Creation errors retain the new remote channel ID for recovery. |
| Cloud queue | Outage-retained jobs could upload after deletion, re-pairing, upload disablement or changed credentials/mapping. | Carry registry generation and recheck current settings after waiting for TLS access. Discard invalid jobs without sending or retrying. |
| Archive queue | Queued uploads/replies could cross a deleted/re-paired sensor lifecycle. | Carry and check registry generation during archive processing and again before replying. |
| Telemetry ingestion | CRC-valid data could contain invalid enums, oversized optical batches or NaN/Infinity in supported measurements. | Apply shared semantic checks at gateway, registry and archive ingestion. Failed readings may carry unavailable placeholders, which consumers suppress. |
| Compact history | Finite extreme values could overflow multiplication or integer rounding before saturation. | Clamp to the compact target's range in floating point before multiplication/rounding, including legacy history conversion. |
| Battery history | Failed ADC results could remain advertised as valid battery history. | Remove the battery capability when its failure flag is present. |
| Stored settings/identity | Unbounded stored text, invalid settings/mappings, a full historical-name cache or failed deletion could corrupt identity/configuration behavior. | Bound and normalize stored data, reject short reads, evict inactive historical identities when needed and preserve the durable sensor entry on failed deletion. |
| OTA job state | Failed upload initialization or cancellation saves still replaced the in-memory job. | Restore the previous job when the initial NVS write fails under the existing OTA mutex. |

## Browser and HTTP corrections

| Area | Failure | Correction |
| --- | --- | --- |
| Numeric HTTP fields | Prefix parsing and narrowing accepted values such as `sensorType=259`, `field=257`, negative channel IDs or malformed cursors. | Require complete decimal syntax and range checks before conversion to protocol fields. Reject nonfinite/out-of-range calibration values. |
| Measurement display | JSON `null` became numeric zero; sensor failures could leave old IAQ advice visible. | Treat missing/nonfinite readings as unavailable and honor failure/protection status. |
| Discovery/setup | Discovering another node rebuilt the form and lost names, selection and focus; polling could overlap. | Preserve drafts/focus and serialize discovery requests. |
| Settings races | Double Save could apply calibration twice. A delayed save/import could use or close a newly opened form. | Serialize saves, capture the submitted form before awaiting, and match completion to its wizard generation. |
| Dashboard startup | An initial configuration request failure prevented normal operation indefinitely. | Retry initialization and retain recovery feedback. |
| Archive CSV | Concurrent exports could duplicate downloads; cancellation could leave archive bandwidth reserved. | Guard concurrent export, show progress and provide cancellation that releases transfer state. CSV retains original samples. |
| Storage failure | Users could continue without knowing local history/archive storage was unavailable. | Display a storage warning in overview/settings without formatting their data. |
| OTA selection | Invalid headers remained selected as compatible, and delayed file parsing could overwrite a later selection. | Clear rejected metadata and correlate parsing to the selected file. Validate target, protocol, sizes, release and version. |
| OTA size contract | The shared manifest and package tool allowed `0x1e0000` bytes while both sensor slots contain `0x140000`. | Use `0x140000` consistently in firmware, package creation/verification, browser checks and realistic OTA test partitions. Reject even correctly signed oversized packages. |

## Energy behavior retained

- BME280 still initializes asleep and uses x1 forced measurements followed by
  sleep/power shutdown. No continuous normal-mode conversion was introduced.
- BME680 retains the 3.3 V, 300-second BSEC ULP configuration, bounded conversion
  waits and scheduled maintenance wakes. Reporting cadence remains distinct from
  BSEC maintenance. State retention uses RTC and infrequent NVS writes.
- Failed sensor initialization now shuts down earlier. Radio recovery remains
  bounded and successful application replies remain separate from MAC ACKs.
- V4 protection remains strictly below 2800 mV, with 2950 mV recovery, invalid-ADC
  protection, sensor shutdown and one bounded battery reporting window followed
  by 86400 seconds of deep sleep. V3 retains its configurable opt-in policy.
- Manual precision recordings keep their separate acquisition/idle behavior.
  Their light-sleep/SW2 handling does not change the BME deep-sleep path.

These are verified software settings and control paths. Actual board current,
radio-on time under RF load and lifetime have not been measured in this review.

## Validation

All 22 production C++ host suites passed. The new suites execute the production
BME680 driver/state store, environmental dispatcher, sensor configuration store,
station configuration store and HTTP numeric parser. Expanded suites exercise
faults, retries, stale data, reset/save ordering, migration, cancelled live work,
recording duration/UTC, ACK correlation and compact saturation. BME680 uses real
Bosch type/configuration headers with a substituted proprietary BSEC algorithm
and hardware API responses; these tests do not execute Bosch learning on a board.

All six browser suites passed against the actual embedded pages with mock APIs:
motion dashboard, recording dashboard, chart readability, sensor settings,
OTA status/file selection and UI reliability. Eight embedded script blocks
parsed successfully. Six real SHA-256/ECDSA package tests, two station-upload
preservation tests and all three pinned library-patch checks passed.

PlatformIO builds passed for `station_s3`, `sensor_pcb_v3` and `sensor_pcb_v4`.
The final station objects were rebuilt after the last HTTP/persistence and
numeric changes. Builds used separate temporary output directories because
other processes were building the ordinary project outputs concurrently.
Python compilation and working-tree whitespace checks also passed.

Reproduction commands and substitute limitations are documented in
[tests/README.md](tests/README.md). No release packages were published, installed
images changed, or signing identities replaced.

## Remaining acceptance work and architectural limits

| Check | Required evidence |
| --- | --- |
| BME280 V4 low-voltage behavior | Measure the switched sensor rail and SDA/SCL during the reported failure, including startup and RF load. Battery voltage alone cannot establish the cause. See [the earlier design review](SENSOR_REVIEW.md). |
| Energy | Measure sleep, conversion, idle precision and transmission current on V3/V4. Verify rail shutdown during absent/mismatched sensors, bus errors and battery protection. |
| Sensor accuracy | Compare TMP117 after thermal equilibrium, IMU orientations/known rotation and MAX30102 contact/light/motion conditions against suitable references. Synthetic driver tests cannot establish physical accuracy. |
| Concurrent load and interruption | Exercise RF loss, Wi-Fi channel changes, slow HTTPS, repeated UI actions and power loss during flash/recording/OTA activity on the real station and sensor. Host RTOS substitutes cover selected interleavings only. |
| Archive/storage recovery | Test interrupted append and reboot with nearly full/failed storage. An unavailable nonblank filesystem is intentionally preserved and requires recovery rather than automatic erasure. |

Profile operations spanning multiple sensor keys use best-effort rollback;
persistent NVS failure or power loss is not a multi-key atomic transaction. A
cloud request already sent cannot be withdrawn after settings change. Deletion
racing after an archive operation's initial generation check can preserve one
additional archive row, but the response generation check suppresses its ACK
to a new lifecycle. These limits are distinguished from the corrected queued
and failed-save behavior above.

## Primary sources

- [Bosch BME280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf): forced/sleep operation, calibration and coherent compensation.
- [Bosch BME680 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme680-ds001.pdf): gas-valid/heater-stability semantics and ULP operation.
- [TI TMP117 datasheet](https://www.ti.com/lit/ds/symlink/tmp117.pdf): addresses, data-ready, averaging, signed resolution and preserved EEPROM/offset registers.
- [ADI MAX30102 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX30102.pdf): FIFO, averaging, ambient-light overflow, reset and red/IR operation.
- [ST LSM6DSOX datasheet](https://www.st.com/resource/en/datasheet/lsm6dsox.pdf) and [AN5272](https://www.st.com/resource/en/application_note/an5272-lsm6dsox-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf): scaling, status and 512-entry FIFO semantics.
- [Espressif Preferences API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/preferences.html): exact transferred-byte counts and persistence errors.
- [Espressif ESP-NOW API](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/api-reference/network/esp_now.html): MAC/application ACK distinction, sequence correlation and serialized sends.
- [Arduino LittleFS implementation](https://github.com/espressif/arduino-esp32/blob/master/libraries/LittleFS/src/LittleFS.h): formatting option; the installed pinned implementation was also inspected.



## Follow-up: dashboard notices and sensor changes (4.3.4)

The local-network/password and ThingSpeak setup/waiting notices now disappear
once after five seconds. Identical polling responses do not reset that timer or
show a dismissed notice again. Changed conditions show the new notice; an
obsolete notice timer is cancelled before displaying a connection/upload error.
Settings and the overview summary retain the current password/cloud state.
BME280 cards no longer display IAQ or gas resistance; both remain on BME680.

Sensor initialization now releases an active previous driver/bus/rail, and a
mismatched first probe no longer prevents power-cycle recovery. Detection state
is reset before the second probe. RTC state tracks three bounded startup
attempts with five-second deep sleep and prompt status reporting, including
configuration changes and failed starts after a previously working module.
Exhausted attempts return to the configured low-power schedule. Success renews
the retry allowance for a later fault. The normal successful BME path remains
12 ms without an added off delay; active-driver replacement uses a 100 ms off
interval. V4 battery protection always wins over startup recovery.

Pending dashboard selections discard previous-driver values, error flags and
recording state while showing initialization feedback. Genuine mismatches
reported after applying the selection remain visible; explicit configuration
never silently substitutes another chip. Bounded startup recovery also runs
on a cold boot. Physical wiring/address faults still require hardware checks.

The production boot/dispatch and browser suites cover these paths, along with
five-second retry reports, exhausted budgets and low voltage during recovery.
The signed 4.3.4 V3/V4 files and application-only station image are listed in
[the release directory](releases/README.md). Software validation is separate
from physical module-swap, rail-discharge and current measurements.

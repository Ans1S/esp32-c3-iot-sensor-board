# Firmware regression checks

The [2026-10-07 reliability review](../RELIABILITY_REVIEW.md) documents the
current fixes, energy behavior and remaining hardware acceptance. The runner
now includes 22 production C++ suites.

Run from the repository root after resolving/building the sensor dependencies:

```sh
python Firmware/tests/run_tests.py --cxx /path/to/clang++
python Firmware/tests/test_ota_package.py
python Firmware/tests/test_library_patches.py
```

`g++`, Clang and `zig c++` are supported; pass the path to `zig.exe` directly.
The runner enables assertions explicitly, builds into a temporary directory and
executes production C++ sources with deterministic hardware/RTOS substitutes.
It does not flash a board or access a cloud account. The patch test downloads
five immutable upstream files and verifies their SHA-256 hashes. `--cache DIR`
uses already downloaded originals for an offline run.

Coverage:

- Actual OTA client transfer, checkpoint resume, corrupted prefix, rejected
  manifest/hardware, low battery, flash failure and trial-boot rollback using
  deterministic flash substitutes. This suite substitutes the hash primitive;
  the package tests independently use real SHA-256 and ECDSA.
- OTA radio response correlation, wrong sender, retry limits and a missing
  send callback; signed-package corruption, wrong keys and truncation.

- Failed first report followed by BSEC maintenance wakes; stale, missing and
  future schedules; rounding; optional discovery backoff.
- Every possible 12-sample ADC sum from 0 to 39600 at seven calibration factors,
  plus every representable float in the accepted calibration range at the
  maximum sum, for both divider ratios. Maximum allowed difference is 1 mV.
- CRC reference vectors, a finalized telemetry packet and corruption of every
  packet byte. The host uses the portable CRC path; inspect the target ELF to
  verify the ESP32 ROM call. Hardware ROM execution is not emulated.
- Late application replies, replies after a failed MAC ACK, replies during
  retry backoff, missing/late send callbacks, and all 13 discovery channels.
- Radio registration injected into every simulated flash write, command
  acknowledgement during persistence, slot deletion/reuse, timestamp buckets,
  interval changes and stale raw-fallback IAQ.
- Per-channel upload limits, fairness, ordered bounded retries, queue overflow,
  expiry and `millis()` wraparound.
- Measurement/fetch budgets, wait clamping and logical time across both normal
  and early-error deep sleep.
- The actual patched Bosch C driver with acknowledged but permanently stuck
  mode status and missing measurement fields.
- BSEC/BME68x build patches and the retained historical BME280 patch against
  pristine pinned upstream sources, repeated application and incompatible input
  rejection. The active BME280 driver now uses checked register transfers and
  no longer depends on the Adafruit library or its build patch.

The RTOS substitutes detect lock-order violations and deterministic event
interleavings. They do not replace on-device concurrent-load, RF, TLS, GPIO,
BSEC algorithm or current measurements. See `../HARDWARE_TESTPLAN.md`.

## Motion support

The C++ runner also tests the LSM6DSOX register driver against a FIFO/I2C
substitute, signed scaling, interval peaks, overflow and fault handling.
Registry tests cover a 1201-slot live ring without per-report flash writes and
migration from the persisted V3 environmental history layout.

Run `node Firmware/tests/test_motion_dashboard.cjs` with Playwright available.
`PLAYWRIGHT_MODULE` can identify an installed module; `PLAYWRIGHT_CHANNEL` can
select an installed browser channel. Set `QA_SCREENSHOT_DIR` to save desktop
and mobile captures. The test uses the actual embedded HTML with mock APIs;
it never connects to a physical station or cloud account.

Run `node Firmware/tests/test_recording_dashboard.cjs` with the same Playwright
configuration to verify automatic archive discovery, selector focus during
telemetry, persistent cursor readouts, slow archive responses, live/archive
switching during acquisition, partially synchronized sessions and cancelled
or failed reads. The fixtures exercise the real embedded dashboard; no sensor
or station is flashed.

Run `node Firmware/tests/test_chart_readability.cjs` to verify single-point
history charts, time-window switching, browsing retained history, exact cursor
values, gap-safe visual smoothing, unchanged CSV samples and mobile layouts.

The C++ acquisition suite executes the production acquisition task for normal
1/10 s measurement windows, recording transitions, startup/read failures,
rail shutdown and button debounce. Registry checks also cover reception-time
preservation across history interval changes and exclude cached heartbeats
from fresh graphs.

## OTA performance and status

The C++ OTA suite asserts sector-sized writes, a final partial sector, resume
with an unflushed RAM tail, completion beyond three minutes and explicit
timeout/resume at ten minutes. It reports flash API call counts without
claiming a hardware timing benchmark. Run
`node Firmware/tests/test_ota_status_page.cjs` with the same Playwright module
to check overlapping refresh suppression, sensor-status caching and recovery
from a network error without overwriting upload results.

The transport suite also covers legacy environmental packet lengths and bounded
trial-boot retries. OTA tests cover both board voltage thresholds at equality
and one millivolt below, as well as boot-confirmation failure handling.
Run `python Firmware/tests/test_web_scripts.py` to syntax-check embedded scripts
with Node.js. Python package tests require `cryptography`.

Precision sensor tests use register/FIFO substitutes and a synthetic 120 bpm waveform.
They check TMP117 averaging and signed resolution, freshness, MAX30102 warmup,
contact loss, overflow, and optical telemetry length. Live history tests retain
601 reports at 100 ms and expire them after one minute without flash writes.


Manual recording tests additionally exercise the production codec/journal,
button debounce, adaptive cadence boundary, sensor lifecycle/clock recovery,
and station filesystem append/deduplication/paging. `test_motion_dashboard.cjs`
loads a 30-minute saved recording and checks minute navigation and full CSV
export. Hardware acceptance is described in [RECORDINGS.md](../RECORDINGS.md).

Quick-feedback tests cover raw-rate motion estimation, unchanged compact record
capacity, stable/drifting/stale temperatures, pulse recovery coverage and the
feedback panels on desktop/mobile. See [definitions](../SENSOR_FEEDBACK.md).

## Sensor accuracy and battery protection regression checks

`battery_boot_test.cpp` executes the production `main.cpp` flow with substituted
hardware: strict 2800 mV entry, 2950 mV RTC recovery, invalid ADC, offline and
unpaired stations, no sensor startup, battery-only packets, 24-hour sleep,
deferred commands/OTA and stopping live mode. Driver checks cover Bosch reference
compensation, signed humidity trims, coherent bursts, failed I2C reads/writes,
configuration/calibration faults and bounded waits. TMP117 tests cover all four
addresses, endpoints, the startup sentinel and preserved EEPROM/offset registers.
MAX30102 tests cover nine synthetic rates from 35 to 195 bpm, ambient-light
overflow, exactly full FIFO, brownout and long polling gaps. IMU tests reject
both current and latched overruns and require reinitialization after bus faults.

The registry and browser tests verify persisted battery-only status for live
sensor types and a protection badge without misleading zero sensor values.
These are software checks; use the [review's hardware acceptance table](../SENSOR_REVIEW.md)
to measure absolute accuracy, supply integrity, threshold calibration and current.

## Recording response, identities and motion references

`recording_json_test.cpp` uses the installed ArduinoJson library and the actual
production point conversion/append writer, including 0/1/16/128-point responses,
multiple output chunks and all three recording types. This catches destination
replacement during serialization instead of relying only on mocked API JSON.
Archive tests also cover torn tails, replay and CRC errors without skipping
unreadable offsets. Registry tests cover MAC-based names/references after
removal, re-pairing and reboot, stationary reference validation and reset.

`battery_boot_test.cpp` executes changed-type application in the production
setup path (one-second reinitialization) and the running live path (fixed type
back to auto detection). `python Firmware/tests/test_station_upload.py` checks
that normal uploads preserve flash and only the explicit reset environment
inserts full erasure. Build the station once to resolve ArduinoJson before
running the host suites.

`node Firmware/tests/test_sensor_settings_dashboard.cjs` checks all six zeroed
axes, reference application in saved plots, original values in CSV, clearing the
reference and selected/applied sensor types with offline name updates. It uses
the same Playwright environment settings as the other dashboard checks.

## Reliability and persistence faults

`sensor_config_test.cpp` exercises V1-V5 schema migration, wrong identities,
partial NVS reads, failed writes/reset, RTC caching and retryable migrations.
`config_store_test.cpp` verifies preserved nonblank filesystem mount failures,
erased-flash initialization, unreadable partitions and bounded stored settings.
`web_input_test.cpp` rejects incomplete, overflowing and out-of-range decimal
HTTP inputs before narrowing. These suites are included in `run_tests.py`.

The BME680 production suite checks ULP cadence, six-hour persistence at accuracy
zero, corrupted/orphaned/rejected learning state, clock wraps, invalid IAQ,
heater stability, durable-reset failures and bounded waits. It uses real Bosch
type/config headers with substituted BSEC algorithm and hardware responses.
The environmental dispatcher suite checks both Bosch addresses, explicit versus
automatic selection, absent/mismatched modules and failed-start rail shutdown.
The live/precision/IMU suites additionally reject cancelled acquisition and
stale FIFO data after scheduling gaps.

Recording tests cover frozen/idempotent stop duration, delayed first UTC anchor
and current-request ACK sequence matching. Registry/archive tests reject invalid
telemetry, saturate finite extreme history values before rounding, preserve
durable settings across failed/interleaved writes and invalidate stale cloud
jobs by generation and current settings.

Run `node Firmware/tests/test_ui_reliability.cjs` with the same Playwright
configuration as the other five browser suites. It checks preserved setup
drafts/focus, serialized discovery, missing values, storage warnings, startup
retry, duplicate saves and delayed responses after reopening forms. The
recording suite additionally checks export progress/cancellation, complete
loading before the first graph, fixed SVG/cursor identity during growing
synchronization, explicit snapshot refresh, completion-triggered loading and
full-session/minute views without extra archive requests; the OTA suite
checks rejected headers and file-selection races. Together these use the actual
embedded pages and mock station APIs, not a live device.

The package tests check the real `0x140000` sensor partition boundary, including
correctly signed oversized-package rejection. The station upload tests execute
the upload script with a substituted environment and never flash hardware.

Sensor-transition regressions also cover a slow selected chip becoming visible
after an initial mismatch, releasing an active driver before changing types,
three bounded startup attempts across deep sleep, early retry reports, return
to the normal interval and battery protection during recovery. Browser checks
cover five-second security/cloud notices that stay dismissed across polling,
new notices when their condition changes, cancellation of obsolete notice
timers before errors, pending/applied sensor mismatch states and BME280 cards
without IAQ or gas-resistance tiles. BME680 retains both tiles.


## Publication guard

Run `python Firmware/tests/test_publication.py` for binary/ZIP token detection,
private material versus TLS parser markers, path/file exclusions, redacted
reports, an index/worktree race and a removed secret in new commit history. Run the checker from the repository root:

```sh
python Firmware/tools/check_publication.py
python Firmware/tools/check_publication.py --staged
python Firmware/tools/check_publication.py --revision HEAD
python Firmware/tools/check_publication.py --history
```

Enable `.githooks` with `git config core.hooksPath .githooks`. The pre-commit
hook scans actual indexed blobs. The pre-push hook scans the pushed commit tree
and newly introduced history, including binary firmware and ZIP members.
Historical scans can legitimately fail on already published old objects;
consult [the review](../PUBLICATION_REVIEW.md) rather than claiming history was
removed. The checker is heuristic and cannot read text encoded only as pixels;
review screenshots visually and use synthetic fixtures. It prints locations
and categories, never matched values.

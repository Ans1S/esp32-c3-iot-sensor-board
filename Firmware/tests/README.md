# Firmware regression checks

The current release is **4.4.4 / 40404**, protocol V5. The runner includes
30 production C++ suites and there are ten embedded-page browser programs.
See the [release evidence](../audit/2026-11-07-release/README.md) for checks
repeated during release preparation and the [reliability review](../RELIABILITY_REVIEW.md)
for earlier fixes and remaining hardware acceptance.

Run from the repository root after resolving/building the sensor dependencies:

```sh
python Firmware/tests/run_tests.py --cxx /path/to/clang++
python Firmware/tests/test_ota_package.py
python Firmware/tests/test_library_patches.py
```

Add `--sanitize` to the C++ runner for undefined-behavior, alignment and
float-to-integer overflow checks. `--suite NAME` selects a suite and can be
repeated; omitting it runs all 30 suites. `--benchmark` additionally runs the
optional host benchmarks in selected `pulse_signal` and `registry` suites.
For example:

```sh
python Firmware/tests/run_tests.py --cxx /path/to/zig --sanitize
python Firmware/tests/run_tests.py --cxx /path/to/clang++ --suite pulse_signal --suite history_flush
python Firmware/tests/run_tests.py --cxx /path/to/clang++ --suite registry --benchmark
```

## Efficiency audit regressions (2026-11-07)

`pulse_signal_test.cpp` compares the production centered-mean helper with the
original loop over 19,200 windows: every ring head, 18-bit endpoints, impulses,
drift, periodic and deterministic random inputs. Float residuals and double
energy sums must match bit for bit. An accessor counts 4,576 versus 551 logical
sample reads. These counts are not physical memory/bus transactions. Existing
precision tests retain pulse decisions across nine synthetic rates and faults.

To repeat the optional host-only benchmark with the same compiler/options:

```sh
c++ -std=c++17 -O2 -UNDEBUG -I Firmware/shared/include -I Firmware/sensor/include Firmware/tests/pulse_signal_test.cpp -o pulse_signal
./pulse_signal --benchmark
```

The benchmark alternates original/optimized order in nine runs of 20,000
windows, verifies checksums and reports median/range in microseconds per
window. It does not measure a complete estimator or an ESP32.

`history_flush_test.cpp` measures full 91/126/63-row environmental/TMP117/IMU
pages using production persistence and real temporary files. Every row must
be read back, match exactly and decode successfully. Injected positioning,
short-read and corrupt-readback faults retain the entire pending batch for a
successful retry; reboot reads retain all original rows. Both write and
readback positioning now require two seeks in total. For a baseline source
comparison only, `EXPECT_HISTORY_FLUSH_SEEKS=0` expects the former `rows+1`.

The registry suite now covers finite extreme, negative, NaN and infinite gas
placeholders before legacy integer conversion. Recording JSON tests use real
ArduinoJson with records at all four byte offsets; sanitizers catch reference
binding to packed unaligned numeric fields. Related status/history/OTA JSON
assignments also materialize scalar values. The complete HTTP handlers still
require target/integration testing beyond the host serializer checks.

See [the efficiency audit](../EFFICIENCY_AUDIT.md) for source/ELF inventory,
compiler settings, before/after costs, limitations and hardware priorities.

## Additional MCU regressions (2026-11-07)

- The BME280 suite compares 1,064,960 complete production readings with the
  original double formula, including all 20-bit temperature ADC codes and
  signed/extreme trim cases. Output float bits, capabilities and validity match.
- `imu_peak` instruments actual driver sqrt calls and compares 130,566 gyro
  vectors in 1,024 windows with the original sqrt/max results. It checks empty,
  signed-limit, multi-poll, discard/reinitialization and balanced/mixed windows.
  A balanced 104+104 window requires 105 instead of 208 roots.
- `protocol_validation` covers every packet length from the 16-byte header to
  250 bytes at four byte alignments, 125,020 single-byte corruptions, immutable
  input and invalid bounds. Host CRC does not execute the MCU's ROM function.
- `optical_fifo` covers every 0..31-frame pointer snapshot, exact values/order/
  ages and short requests at all four batch positions. The FIFO substitute
  follows the documented fixed register address during bursts; physical
  transfer races and partial-read recovery still need hardware acceptance.
- Registry tests cover all three live-ring capacities, rollover, expiry,
  wrapped/truncated snapshots and exact sample bytes. Its optional benchmark
  uses nine runs of 1,000 reads per type and reports host-only median/range.

See [the MCU follow-up](../MCU_OPTIMIZATION_REVIEW.md) and its evidence README
for the isolated original-source comparison and actual target assembly costs.

## Host compiler and coverage

The [main/radio/OTA review](../FLOW_TRANSFER_REVIEW.md) adds operation counters
around the real protocol implementation, exact short/motion/full retry bytes,
lazy queue/failure checks, single-poll snapshots, resume-prefix read/buffer
checks and a station OTA transfer suite. Both client and station exercise the
maximum 1,310,720-byte slot. The station fixture substitutes JSON, signatures,
hash primitives, partition/NVS and RTOS operations; real crypto/serializer tests,
target builds and hardware acceptance remain separate. Allocation/prefetch
failure falls back to direct reads, and incorrect signatures/digests are rejected.

The registry's optional `--benchmark` also reports nine batches of 20,000
environmental registrations for BME280 and BME680/raw fallback. These are host
batch averages with fixture locks, not MCU or radio latency.

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

Set `RECORDING_PAGE_SIZE=512` to exercise larger recording pages. Run
`node Firmware/tests/test_dashboard_performance.cjs` for a controlled browser
comparison with 19 simulated sensors and 4,400 history points. The simulated
API delays and page size do not measure an actual station, radio or filesystem.

Run `node Firmware/tests/test_history_priority.cjs` to check that a complete
selected window appears before older history finishes, and that failed older
pages preserve the window while navigation and full CSV export remain gated
until a successful retry restores the complete snapshot.

Run `node Firmware/tests/test_chart_readability.cjs` to verify single-point
history charts, time-window switching, browsing retained history, exact cursor
values, gap-safe visual smoothing, unchanged CSV samples and mobile layouts.
Touch regressions use a touch-enabled browser context and verify pinning after
pointer leave, adjacent-sample controls, incremental updates and retained raw
values when a selected point leaves the rolling window. Axis label sizing
follows the rendered SVG width without enlarging the stored data.

Long-uptime regressions exercise normal precision acquisition after a simulated
2^31 ms uptime, inactive/expired retries, millis wrap, 64-bit login cooldowns
and stale motion-reference rejection after a complete millis cycle. Additional
checks cover restart scheduling at the exact millis wrap, archive-reserve expiry
after a full wrap, exhausted recording timestamps, malformed MAC addresses and
unterminated OTA identities. The radio send guard is checked with missing,
delayed and completed callbacks
across a 32-bit timer wrap. Battery tests exercise low and invalid OTA voltage
results in both ordinary and live V4 operation; BSEC checks reject invalid
output counts and extreme scheduling values. A stuck acquisition during battery
protection is tested with a simulated restart. These are accelerated deterministic
checks, not a multi-week physical soak test. The status API exposes internal-heap
minimum/free/largest-block and PSRAM
free/largest-block bytes for the remaining device load checks. See
[the resource and usability review](../RESOURCE_AND_UX_REVIEW.md).

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

The journal recovery suite counts flash reads for two full pages of each record
width and checks failed reads and corrupt committed data across batch boundaries.
The archive and normal-history suites count file positioning calls for a
sequential page and still check CRC failures, offsets and snapshot boundaries.
These host call counts are not measurements of device latency or power. See
[the storage performance review](../PERFORMANCE_REVIEW.md).

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
configuration as the other browser programs. It checks preserved setup
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
to the normal interval and battery protection during recovery. BME280 conversion
failures after successful initialization also consume the bounded retry budget;
tests cover exhaustion, valid-sample recovery, automatic detection, low battery
during retries and unchanged BME680 maintenance without a new sample. Browser checks
cover absent startup security/cloud notices across polling,
new notices when their condition changes, cancellation of obsolete notice
timers before errors, pending/applied sensor mismatch states and BME280 cards
without IAQ or gas-resistance tiles. BME680 retains both tiles.


## Publication guard

Run `python Firmware/tests/test_publication.py` for binary/ZIP token detection,
private material versus TLS parser markers, path/file exclusions, redacted
reports, an index/worktree race and a removed secret in new commit history.
The seven checks also reject device exports and authentication files while
confirming that Git ignore rules preserve public source/build inputs and the
reviewed release files. Run the checker from the repository root:

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

## Compact dashboard and adaptive normal history

`paged_history_test.cpp` runs the production paged store against actual temporary
files: all 19 nodes, fair quotas, full storage, explicit reserve, immutable
pagination ceilings, stale readers, CRC/torn-tail recovery and exact precision
values. Registry integration also imports the legacy ring, preserves settings,
and rejects cached precision contacts as new measurements. The configuration
suite checks the independent dashboard NVS key and CRC fallback.

`node Firmware/tests/test_compact_dashboard.cjs` exercises the actual page with
a 4,400-point, multi-week fixture. It checks entire-card and entire-voltage-tile
touch navigation, selected-node-only paging, retained raw points, bounded SVG
geometry, long periods, station-backed order/visibility/columns, mobile width,
reachable settings/firmware and absent startup notices. There are ten
embedded-page browser programs in total, including the later readability and
selected-window history-priority checks.

The upload tests verify inherited erasure removal and the application-only
target. These software tests never flash a board. Power-cut recovery under
actual LittleFS/flash timing, 19-node RF load, PSRAM failure and flash endurance
still require the hardware plan.

## Interaction and fault stress checks

Run `node Firmware/tests/test_interaction_stress.cjs` with the same Playwright
configuration. It uses the actual embedded dashboard on a mobile viewport and
19 synthetic nodes. It checks keyboard/touch navigation, cancellation during
slow history reads, a reopened layout while an older save finishes, failed
later pages without publishing a partial snapshot, HTTP failure, a stuck
pagination cursor, the real 15-second request timeout, explicit retry,
station restart with unchanged revisions, and disconnection/recovery without
JavaScript errors. The overview must not fetch all nineteen histories.

The registry suite tests a full 32-bit millis cycle without reviving expired
live points, both with no new contact and with cached/fresh packets after the
outage. It retains the inclusive 60-second boundary and normal wrap handling.
The power-pin suite executes the production controller separately for PCB V3
and V4, including inactive output latches before pin enable and the absence of
an ADC gate on V3.

Paged-history checks additionally inject low internal RAM, absent PSRAM and
allocation failure, then verify recovery without recording rejected values.
They exercise full PSRAM pages through a TMP117-to-IMU row-width change and
report a CRC error encountered by an active reader. The existing full-storage,
nineteen-node quotas, torn-tail, durable-ACK and power-protection checks remain.
These are accelerated software scenarios; physical flash power cuts, RF load,
real PSRAM/heap behavior and current/accuracy measurements require hardware.

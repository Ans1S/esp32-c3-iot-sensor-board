# Firmware efficiency and correctness audit - 2026-11-07

Publication follow-up: these measurements preserve the pre-release 4.4.3 local
reference. The changes are now packaged as [4.4.4 / 40404](releases/README.md);
release-build hashes and checks are separate from the historical audit artifacts.

This report records the first local audit result. The later
[MCU optimization follow-up](MCU_OPTIMIZATION_REVIEW.md) adds five changes and
records the second-pass results against this preserved intermediate reference.
The latest changes are documented in the [main/radio/OTA follow-up](FLOW_TRANSFER_REVIEW.md).

## Scope and reference

This is a software audit of the maintained `Firmware/` tree, covering PCB V3,
PCB V4 and the ESP32-S3 station. The reference is clean commit
`ff7afa30032221ec961f2bc939c1dbba4c90a0c2` (4.4.3 / 40403). The worktree was
clean before any maintained file was changed. A Git archive, build logs,
ELF/BIN/map/partition files and complete disassemblies were retained locally
under ignored `.tools/firmware-efficiency-20261107/`, separately for before
and after. [Artifact hashes](audit/2026-11-07/artifact-manifest.json) identify
those files. No existing work, installation key or release package was replaced.

The maintained architecture, component READMEs, changelog, precision/recording,
power, sensor, hardware-test, performance, reliability and real-time guides were
read before implementation. The local `wcharger-release` workflow supplied the
relevant validation, preservation and publication rules. No dedicated performance
skill was available. This review is not a Codex Security vulnerability scan.

All own production definitions received a source-level review pass, including
inline/template functions and embedded JavaScript callbacks. This is a review
coverage statement, not proof of all execution paths or absence of defects.
Test code and substitutes were inventoried and exercised at suite level;
selected test implementations were reviewed in detail, without claiming a
separate semantic audit of every test helper. External libraries and physical
behavior have the limits below.

## Inventory and coverage

The immutable baseline contains 98 production source/configuration modules and
1,112 production definitions: 579 C++, 515 JavaScript and 18 Python. The result
contains 101 such modules and 1,125 definitions: 581 C++, 515 JavaScript and
29 Python. Including tests/substitutes, the inventory grows from 205 modules /
1,832 definitions to 210 / 1,858. Definitions include lambdas, template bodies
and defaulted methods; these numbers are not counts of distinct machine-code
entry points. Documentation, release assets and external dependency bodies are
not included in these definition counts.

- [Before definitions](audit/2026-11-07/before-functions.csv) and
  [after definitions](audit/2026-11-07/after-functions.csv) identify every
  inventoried body by relative source path and start/end line.
- [Module review](audit/2026-11-07/module-review.csv) records the review scope
  and evidence for each component, including test-only and inactive inputs.
  The accompanying before/after module CSVs record source byte hashes.
- Six `*-symbols.csv` files under [audit evidence](audit/2026-11-07/README.md)
  enumerate the emitted project symbols, sections, sizes, instruction counts,
  arithmetic/division/fence sites and annotated direct calls for each target
  and phase. There are 161 V3, 165 V4 and 289 station entries in each phase.
- [Stack frames](audit/2026-11-07/stack-frames.csv) record auxiliary GCC
  `-fstack-usage` compilations for V4 and the station. V3's changed function
  was also checked in its own ELF; it has the same 1,104-byte prologue frame.

The AST inventory completed without parse errors. ESP storage attributes and
initializer includes are normalized only for parsing, preserving source line
numbers; actual builds use the original inputs. Python is parsed with `ast`.
Embedded scripts are parsed independently of the C++ raw string containing them.

| Component group | Source review and verification | Important boundary |
| --- | --- | --- |
| Sensor boot, config, logical clock, ADC, power, waits, sleep | All own bodies; boot/power/config/clock/ADC tests; V3/V4 builds | No rail, ADC-reference, sleep-current or reset-time measurement |
| BME280, BME680, BSEC state and environmental dispatch | All own bodies; register/calibration/fault/budget/ULP tests; active patch tests | BSEC learning is proprietary; synthetic outputs do not establish physical accuracy |
| LSM6DSOX, TMP117, MAX30102 and live acquisition | All own bodies; FIFO/freshness/cancellation/SW2/normal-window tests; numerical comparison | Real bus timing, contact/motion artifacts and sampling during flash stalls remain open |
| Shared V5 protocol, journal, recording codec, scheduler and policies | All own bodies; packet corruption, crash recovery, ACK, timing and queue tests | ROM CRC execution, actual flash atomicity and RF are not emulated |
| Sensor OTA and station OTA staging/service | All own bodies; client checkpoint/low-voltage/rollback tests; real package crypto; target builds | Station service is not fully executed by a host HTTP/flash integration test |
| Registry, config, paged history and archive | All own bodies; real temporary-file, interleaving, low-memory, reserve, CRC and durability tests | Host files and RTOS substitutes do not reproduce LittleFS power cuts or arbitrary concurrency |
| Gateway, Wi-Fi and ThingSpeak | All own bodies; transport/scheduler/registry tests; target builds; callback/task assembly | Vendor Wi-Fi/TLS runtime and slow/chunked HTTP account responses need integration/device work |
| HTTP/JSON and embedded UI | All own bodies/callbacks; actual ArduinoJson serializer; eight script syntax checks; ten browser programs | Browser APIs are fixtures; complete production HTTP handlers are not executed on the host |
| Build scripts, active patches, partitions, package/upload/publication tools | Source/config review; three target builds; six crypto, four upload, three patch and seven publication tests | No release, installation-key initialization, upload or device enumeration was performed |

### Tasks, callbacks and state machines

The sensor's Arduino task owns boot, battery preflight, environmental exchange,
live-mode orchestration, recording replay and OTA. The live-acquisition task
uses a 6,144-byte requested stack and priority 2; it exclusively owns I2C during
normal/manual precision operation. Its profiles are Stopped/Normal/Recording,
with independent cadence, pause, retry, acquisition ownership and SW2 debounce.
ESP-NOW send/receive callbacks execute through the Wi-Fi service, not as own
hardware interrupt handlers. The OTA boot-guard timer callback restarts an
unconfirmed image. No own GPIO/FIFO hardware ISR is registered.

The station Arduino task services HTTP, Wi-Fi recovery, DNS and mDNS. Gateway
processing requests a 6,144-byte stack at priority 3; telemetry and archive
persistence each request 6,144 bytes at priority 1. The lazy ThingSpeak worker
requests 10,240 bytes at priority 1 only after an enabled valid upload. Callback
handoffs, bounded queue policies, send completion and generation checks were
reviewed with their consumers. SDK-created Wi-Fi/event/timer tasks remain an
external scheduling boundary.

Other reviewed state includes RTC assignment/recovery and battery hysteresis;
BSEC schedule/checkpoint state; recording preparation/acquisition/stop/sync;
journal page commit/ACK bits; OTA nonce, checkpoints and trial boot; registry
generations/config commands; pending history buffers, quotas and archive
reserves; cloud ordering/rate limits/retries; Wi-Fi reconnect/scan/AP state;
authentication/session/restart state; and browser request generations,
selected history/recording snapshots, cancellation, exports and cursors.

## Build and machine-code method

[Toolchain settings](audit/2026-11-07/toolchain.json) capture the relevant flags.
Both phases use PlatformIO 6.1.19, pioarduino 55.03.311, Arduino-ESP32 3.3.11,
GCC 14.2.0 (`esp-14.2.0_20260121`) and matching GNU binutils 2.43.1. C3 uses
`riscv32-esp-elf-*`, `rv32imc_zicsr_zifencei` and 80 MHz. S3 uses
`xtensa-esp32s3-elf-*`, its long-call/windowed ABI settings and 240 MHz.
Normal builds use `-Os`, function/data sections, stack protection and no LTO.
The shared public-path mapping is retained.

An unexpected configuration detail was verified rather than assumed: the
actual compile command contains `-std=gnu++17`, `-std=gnu++2b`, then
`-std=gnu++2a`; the last flag selects C++20. This inherited configuration is
identical in both phases. Host comparisons use Clang 20.1.2 through Zig 0.15.2
on Windows x86_64, C++17, `-O2`, assertions and no fast-math. Comparisons are
paired within each environment; host results are never substituted for C3/S3
timings. Build durations were not used as firmware performance measurements.

All emitted project-symbol entries were structurally surveyed for placement,
size, divisions/remainders, copies, fences and annotated calls. Detailed
assembly inspection concentrated on the changed optical and flush loops;
CRC/finalization/validation; ADC scaling; BME compensation/scheduling; FIFO
poll/read; acquisition and atomics; radio callbacks/channel attempts; journal
recovery/append/ACK; station history/archive readers/writers; registry live
ingestion/copies/settings; HTTP serialization; and the cloud worker.

This is not an instruction-by-instruction correctness proof of every linked
function. Inlining, aliases, compiler clones, merged/discarded functions,
indirect Xtensa calls and libraries prevent a one-to-one source/symbol coverage
percentage. The CSV direct-call counts omit unresolved indirect calls and do
not form a complete call graph. ROM, Wi-Fi and proprietary BSEC internals were
not re-audited. No cycle simulator, instruction trace or hardware profiler was
used. No handwritten assembly, intrinsics, global frequency/optimization
changes or weakened memory ordering were introduced.

## Optimization 1: exact optical centered mean

`PrecisionSensors::read()` previously resummed 25 overlapping integer samples
for each of 176 residuals. `pulse::detrend()` initializes one 25-sample sum and
slides it by removing/adding one value. Every 18-bit input window fits in a
32-bit unsigned sum (maximum 6,553,575), and integer additions are also exact
in the former double sum. The same double division, float residual and ordered
energy accumulation follow. Pulse correlation, thresholds, warmup, gain,
FIFO handling and update cadence are unchanged.

| Scenario / metric | Target/environment | Before | After | Change | Method / classification |
| --- | --- | ---: | ---: | ---: | --- |
| Logical sample reads per centered-mean window | V3/V4 algorithm, host accessor | 4,576 calls | 551 calls | -4,025 (-87.96%) | Production template instantiated with counting accessor; proven count, indirect MCU performance evidence |
| Residuals and energy | Host, all 200 heads x 96 patterns | Original loop | 19,200 bit-identical windows | 0 differing windows | `pulse_signal_test.cpp`; proven for tested inputs plus exact-sum argument |
| Centered-mean elapsed time | Windows host, Clang `-O2` | 3.279410 us/window median | 0.218480 us/window median | -3.060930 us (-93.34%) | Nine alternating runs x 20,000 windows/version; proven host-only |
| Observed elapsed-time range | Same nine runs | 3.208125-3.393345 us/window | 0.217970-0.219595 us/window | Reported spread | Same inputs/checksum; not a complete-estimator or MCU measurement |
| Emitted `PrecisionSensors::read()` size | C3 V3 and V4, each | 2,044 bytes | 2,072 bytes | +28 (+1.37%) | Actual ELF symbol sizes; proven cost |
| Emitted instructions in that function | C3 V3 and V4, each | 647 | 653 | +6 (+0.93%) | Static disassembly; not executed instruction counts |
| Function stack frame | C3 V3/V4 | 1,104 bytes | 1,104 bytes | 0 bytes | ELF prologues; V4 auxiliary GCC stack report |

The actual C3 assembly removes the inner floating-point addition loop and its
per-sample conversion/remainder. It initializes the sum with integer loads/adds
and wraps indices with comparisons; only the initial center uses a remainder.
The original mean loop dynamically reached 4,400 double-addition calls per
window; the new sliding sum reaches none for summing the mean. This is inferred
from fixed loop bounds and disassembly, not a measured instruction trace.
The remaining correlation and square-root work still exists.

The changed function is slightly larger, so this is deliberately not called
an instruction-count or image-size optimization. No heap allocation, new
sample array, sensor transaction or task was added. Existing precision tests
still pass nine synthetic pulse rates, weak/small signals, independent gain,
contact loss, overflow, brownout and partial FIFO faults. Faster complete
estimation, fewer acquisition gaps and lower charge on a real C3 are **not
demonstrated**. Measure those under concurrent recording/radio/flash load.

## Optimization 2: sequential history flush verification

`PagedHistory::flushNode()` now positions once for the complete contiguous
readback. It still truncates/positions the append, writes the same rows, flushes,
synchronizes, reads every row, compares every byte, decodes/CRC-checks each row,
and closes successfully before publishing durable counts/removing pending data.
A failed seek/read/verification retains the batch for retry. No storage ACK,
normal-history batching deadline, write count/volume or retention policy changed.

| Scenario / metric | Target | Before | After | Change | Method / classification |
| --- | --- | ---: | ---: | ---: | --- |
| Full 91-row environmental page flush | S3 production code, host files | 92 seeks | 2 seeks | -90 (-97.83%) | Counted production `fseek`; five repetitions, identical counts |
| Full 126-row TMP117 page flush | Same | 127 seeks | 2 seeks | -125 (-98.43%) | Same method, five repetitions |
| Full 63-row IMU page flush | Same | 64 seeks | 2 seeks | -62 (-96.88%) | Same method, five repetitions |
| Verified row reads, respectively | Same | 91 / 126 / 63 | 91 / 126 / 63 | 0 calls | Actual production `fread` counter |
| `flushNode()` symbol | S3 ELF | 1,138 bytes | 1,155 bytes | +17 (+1.49%) | Proven code-size cost |
| Function stack frame | S3 GCC | 976 bytes | 976 bytes | 0 bytes | Auxiliary compilation with unchanged code-generation options |

Counts are **proven host API counts**, with **indirect** support for an on-device
speed benefit. They are not physical flash transactions. No S3/LittleFS latency,
wear reduction or endurance improvement is claimed. The new suite also checks
reboot reads and seek failure, short middle read and corrupt readback for all
three widths, followed by complete retry. Existing full-space, torn-tail,
eviction/snapshot, exact-precision, archive-preservation and allocation-failure
tests still pass. Measure flush latency and radio response tails on LittleFS.

## Correctness corrections, without a speed claim

1. **Legacy gas conversion:** a finite extreme or unavailable gas placeholder
   reached an unchecked float-to-`uint32_t` assignment in
   `SensorRegistry::recordHistoryLocked()`, even without that capability.
   UBSan reproduced float-cast overflow. The encoder now maps unavailable,
   nonpositive values to zero and saturates finite high values before casting;
   ordinary values retain truncation. The registry regression checks 1234.75,
   maximum finite float, negative, NaN and infinity. The target function grows
   from 890 to 970 bytes (+80); no speed advantage is claimed. The normal
   paged encoder already had bounded conversion and is unchanged here.
2. **Packed JSON scalar references:** actual ArduinoJson serialization of an
   optical record bound a misaligned `const float&`; UBSan stopped at the pulse
   value. Numeric fields are now materialized as values before reaching the
   library in recording, status, live/paged history and OTA JSON. Packet/disk
   structures and output values are unchanged. The actual serializer is tested
   with each record type at byte offsets 0, 1, 2 and 3. Its S3 symbol grows from
   1,590 to 1,674 bytes (+84), with the same 320-byte local frame. Complete HTTP
   handlers are build/source checked and still need device integration tests.
3. **Conflicting operating guidance:** the hardware plan still instructed normal
   uploads to erase settings, firmware changes to clear pairing, per-client login
   blocking and obsolete fixed history limits. Those statements now match current
   code. Architecture/sensor guidance also distinguishes V4's protective ADC
   preflight on every wake from the report-only ADC path without that policy.
   These are documentation corrections, not new behavior or energy savings.

The first sanitizer run stopped at the packed-reference issue; it was fixed
and the complete sanitized suite subsequently passed. An initial tree-sitter
0.26 Windows access violation was resolved with 0.25.2. Initial auxiliary GCC
response-file quoting/dependency-output failures were corrected before taking
stack evidence. These tool failures are not hidden as successful firmware
checks. Normal baseline/final firmware builds did not fail.

## Combined target results

Flash below is PlatformIO's used-program metric, not the padded BIN file
length. Static RAM is the reported DRAM/noinit total, not runtime free heap,
PSRAM or stack headroom. Full section and file-size data are in the ELF JSONs
and artifact manifest. Percent changes use `(after-before)/before`.

| Target / metric | Before | After | Absolute / relative change |
| --- | ---: | ---: | ---: |
| V3 Flash | 1,076,713 bytes | 1,076,741 bytes | +28 / +0.0026% |
| V3 static RAM | 45,908 bytes | 45,908 bytes | 0 / 0% |
| V3 `.iram0.text` | 64,186 bytes | 64,186 bytes | 0 / 0% |
| V4 Flash | 1,077,641 bytes | 1,077,669 bytes | +28 / +0.0026% |
| V4 static RAM | 45,908 bytes | 45,908 bytes | 0 / 0% |
| V4 `.iram0.text` | 64,186 bytes | 64,186 bytes | 0 / 0% |
| S3 Flash, all changes | 1,589,738 bytes | 1,590,490 bytes | +752 / +0.0473% |
| S3 static RAM | 108,268 bytes | 108,268 bytes | 0 / 0% |
| S3 `.iram0.text` | 75,779 bytes | 75,779 bytes | 0 / 0% |

Both C3 targets gain the exact optical sum; the station gains fewer verification
seeks and the two correctness fixes. No additional persistent/runtime buffers,
tasks or per-operation heap allocations were introduced. No MCU timing, wake
duration, current, battery lifetime, physical flash wear or sensor accuracy
improvement is demonstrated for any target. Partition binary hashes match
before/after; layouts, settings, recording formats and V5 lengths are retained.

## Validation outcome

| Check | Baseline | Result / limit |
| --- | --- | --- |
| Normal PlatformIO targets | All three passed | All three final targets passed, with target-specific ELF inspection |
| Production C++ host runner | 24 suites passed | 26 suites passed normally and all 26 with `--sanitize` |
| Embedded-page browser programs | All ten passed | All ten passed again, using real pages and fixture APIs |
| Embedded JavaScript parsing | Eight scripts passed | Scripts unchanged; no JavaScript runtime edits |
| Real cryptographic OTA package tests | Six passed | Synthetic payloads/temporary test keys; installation key untouched |
| Station upload-preservation tests | Four passed | No actual upload; normal/application/reset paths substituted |
| Pinned library patch checks | All three passed | Pristine-input hashes, repeat application and rejection tested |
| Publication-guard regressions | Seven passed | Synthetic binary/ZIP/redaction/ignore/history cases |
| Source/ELF inventory, Python compilation, whitespace | Completed | No AST parse errors; generated evidence uses relative paths |

The final worktree publication scan also covers current binary and ZIP assets;
its scope and result are recorded with the evidence. It is a heuristic scan,
not a claim that historical exposures have been removed. Nothing was staged,
committed, pushed, merged, released or sent to another person.

The controlled dashboard performance fixture still completes its 4,400-point,
19-node scenario with nine history calls. Its before/after synthetic delays
are not evidence that this firmware made an installed station faster. No UI
performance improvement is claimed by this audit.

## Remaining risks and prioritized physical acceptance

| Priority | Work / remaining risk | Required before/after evidence |
| --- | --- | --- |
| 1 | V4 protection, supply and measurement integrity | Authorized controlled-supply checks at invalid ADC, 2799/2800 and 2949/2950 mV; capture J1, GPIO6/10, SDA/SCL and RF load; verify daily protection, bounded retry and stopped recordings |
| 1 | Concurrent flash/RF acquisition and real persistence | Power cuts during journal commit/ACK, archive append, history flush and OTA; compare committed samples/CRC, duplication, gaps, retries and unchanged mount-preservation behavior |
| 1 | Target latency and stack/heap reserves | Mixed 19-node RF traffic, browsing, TLS, OTA and nearly full storage; receive-to-reply tails against the 260-ms normal window; task watermarks, reset reasons, largest internal/PSRAM blocks |
| 2 | Optical optimization on both C3 revisions | Time the complete estimator and capture task service gaps with identical synthetic/replayed FIFO data, then real contact/light/motion trials; compare original samples and pulse decisions |
| 2 | Real energy | At least 20 identical cycles/mode: charge per complete wake, BME680 maintenance-only and due report, weak RF, discovery, normal precision, SW2, idle and actual OTA; measure sleep current separately |
| 2 | Accuracy and long uptime | Reference TMP117 temperatures, IMU poses/known rotation, pulse reference; multi-week mixed-load soak including Wi-Fi/I2C faults, wraps, archive filling, charge recovery and PSRAM failure |
| 3 | Cloud account response bounds | Slow/large and chunked/identity HTTP fixtures before a bounded reader or asynchronous account workflow; current synchronous handler and unbounded response `String` remain a latency/heap risk |

Large existing S3 local frames remain: `updateConfig()` 4,432 bytes,
`setMotionReference()` 3,968, profile sync 2,272 and profile removal 2,256.
The settings handler adds a 656-byte frame before entering `updateConfig()`;
SDK/NVS calls, RTOS context and other nesting add more. These local values do
not establish an overflow or sufficient remaining stack. The requested 8 KiB
Arduino loop stack is confirmed by the linked `getArduinoLoopTaskStackSize()`
return value; it and the other task budgets require measured low-water marks.
The unchanged sensor OTA local frame is 2,304 bytes, acquisition loop 512 and
journal recovery 832. No runtime stack/heap measurements were available.

Other retained boundaries are the bounded/drop-on-overload volatile telemetry
and cloud queues; loss of uncommitted normal history on power loss; restart
after a missing radio/acquisition completion; best-effort multi-key profile
rollback; cache/interrupt stalls during flash; and local HTTP/CRC-based radio
trust limits already documented in the architecture. Successful substitutes
do not prove real scheduling, storage atomicity or cryptographic device identity.

No device was accessed, flashed, erased, reset or physically measured. The
changes remain local and unreleased, with release identity 4.4.3 / 40403 intact.

## Changed maintained files

Paths below are relative to `Firmware/`. The evidence directory's README
describes every CSV/JSON artifact; no local binaries, keys or raw device data
were added to maintained content.

| Purpose | Files |
| --- | --- |
| Exact optical mean | `sensor/include/pulse_signal.h`, `sensor/src/precision_sensors.cpp` |
| Sequential flush verification | `station/src/paged_history.cpp` |
| Bounded legacy gas conversion | `station/src/sensor_registry.cpp` |
| Packed scalar JSON handling | `station/include/recording_json.h`, `station/src/web_portal.cpp`, `station/src/ota_service.cpp` |
| Numerical, persistence and fault regressions | `tests/pulse_signal_test.cpp`, `tests/history_flush_test.cpp`, `tests/file_seek_counter.h` |
| Expanded existing regressions and runner | `tests/recording_json_test.cpp`, `tests/registry_test.cpp`, `tests/run_tests.py` |
| Reproducible inventories and ELF survey | `tools/firmware_inventory.py`, `tools/elf_metrics.py` |
| Corrected operating/acceptance guidance | `ARCHITECTURE.md`, `HARDWARE_TESTPLAN.md`, `sensor/README.md` |
| Change record, test instructions and audit | `CHANGELOG.md`, `tests/README.md`, `EFFICIENCY_AUDIT.md` |
| Coverage, toolchain, target sections/symbols, frames, measurements and hashes | `audit/2026-11-07/README.md` and its CSV/JSON files |

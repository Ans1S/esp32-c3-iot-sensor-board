# Additional MCU and machine-code optimizations - 2026-11-07

Publication follow-up: these measurements preserve the pre-release 4.4.3 local
reference. The changes are now packaged as [4.4.4 / 40404](releases/README.md);
release-build hashes and checks are separate from the historical audit artifacts.

This document records the second local audit result. The later
[main/radio/OTA follow-up](FLOW_TRANSFER_REVIEW.md) contains the current flow
optimizations and comparisons against this preserved intermediate reference.

## Reference, coverage and method

This follow-up implements **five additional optimizations** after the
[first efficiency audit](EFFICIENCY_AUDIT.md). Its before column is that
previous **local worktree result**, not clean Git HEAD. Existing improvements
and correctness fixes are retained. HEAD remains
`ff7afa30032221ec961f2bc939c1dbba4c90a0c2`; no commit or release was created.
The complete maintained reference worktree is preserved in the ignored local
`.tools/firmware-mcu-20261107/before-worktree.zip`. Its hash and the before/after
ELF, BIN, map and partition hashes are recorded in the
[artifact manifest](audit/2026-11-07-mcu/artifact-manifest.json).

Both phases use the same GCC 14.2.0 / binutils 2.43.1, framework, `-Os`, public
path mapping and target settings as the first audit. C3 uses the RISC-V
`rv32imc_zicsr_zifencei` target at 80 MHz; S3 uses Xtensa at 240 MHz. The
effective language standard remains C++20 due to the inherited final compiler
flag. No CPU, bus or measurement frequency, global optimizer flag, memory
ordering or hardware protection was changed. The local `wcharger-release`
skill's applicable preservation and validation rules were retained.

The follow-up structurally surveys all emitted project-symbol entries in both
phases: 161 V3, 165 V4 and 289 station entries. Detailed assembly inspection
follows the changed arithmetic, square-root branches, ring loops, copies, CRC
calls and function prologues. GCC auxiliary `-fstack-usage` compilations cover
all 36 application translation units across V4 and S3 in each phase; selected
frames agree with actual ELF prologues, also checked independently for V3.
This is static inspection, not instruction-trace coverage or a WCET proof.
Indirect Xtensa calls, inlining/clones, SDK/ROM and proprietary BSEC retain
the first audit's limits. No device or cycle simulator was used.

The before inventory contains 210 modules / 1,858 definitions, including tests.
After contains 214 / 1,872; production/build scope is 101 modules / 1,126
definitions. [Module review](audit/2026-11-07-mcu/module-review.csv) distinguishes
changed-block review from the retained first-pass review of unchanged modules.
AST parsing has no errors. Counts are source definitions, not execution coverage.

## 1. BME280: exact integer input preparation

`Bme280Driver::read()` now forms `adcT - 16*t1`, the integer pressure offset,
and two integer trim scales before converting to double. These intermediates
fit signed 32-bit integers. The former temperature differences have only
power-of-two denominators and are exact binary values; their integer numerator
is also exact. The changed temperature product has fewer than 53 significant
bits. Thus this preserves Bosch's double compensation rather than reducing
its precision. Remaining expression order, division, output floats, validity,
forced-conversion sequence, waiting, coherent burst and shutdown are retained.

| Metric / environment | Before | After | Change / evidence |
| --- | ---: | ---: | --- |
| C3 V3/V4 function size, each | 2,484 bytes | 2,264 bytes | -220 (-8.86%); actual ELF |
| Static instructions, each | 769 | 702 | -67 (-8.71%); not executed instruction counts |
| Software arithmetic call sites | 103 | 95 | -8 (-7.77%): four multiplies, three subtractions, one unsigned conversion removed |
| Local stack frame | 432 bytes | 384 bytes | -48 (-11.11%); ELF/GCC |
| Compared complete readings | Original formula | 1,064,960 bit-identical outputs | All 20-bit temperature ADC codes at reference trims, plus 32 boundary/random trim sets x 512 raw triples |

The expanded production-driver test compares temperature, pressure, humidity,
capabilities and validity with the immutable original formula. Existing bus,
calibration-consistency, skipped-channel, timeout and no-extra-conversion tests
also pass. The cost improvement in machine code/frame is proven. MCU time and
charge per environmental cycle are not measured; the fixed conversion/settling
waits remain and may dominate total wake time.

## 2. IMU: defer gyro square root until window readout

`Lsm6dsoxDriver::poll()` retains the maximum squared gyro magnitude, then
`read()` takes one square root for that window. For finite nonnegative sensor
inputs, monotonic square root preserves the maximum and its resulting float.
The scaled axes and squared-sum order remain unchanged. Every acceleration
sample still gets its original square root for the motion-feedback filter.
No filter input, mean, peak, count, freshness check, FIFO boundary, reporting
interval or SW2 behavior was removed. Discard/reinitialization/readout reset
the added accumulator at the same window boundaries.

| Scenario / metric | Before | After | Change / evidence |
| --- | ---: | ---: | --- |
| Balanced 104-gyro/104-accel window | 208 sqrt calls | 105 sqrt calls | -103 (-49.52%); actual driver with counting wrapper |
| Gyro-heavy 104-gyro/1-accel window | 105 | 2 | -103 (-98.10%); same test, not the nominal balanced rate |
| Gyro peak equivalence | Original sqrt/max | 130,566 vectors in 1,024 windows match bits | Signed endpoints, quiet tails, empty/single windows and deterministic random vectors |
| C3 `poll()` symbol/frame | 914 / 96 bytes | 914 / 96 bytes | No size/frame cost; sqrt branch now belongs only to acceleration |
| C3 `read()` symbol/frame | 542 / 48 bytes | 560 / 48 bytes | +18 bytes of code; same frame |

The IMU object gains one four-byte float. Aggregate linked static RAM is
unchanged in both node targets. The target disassembly confirms the gyro branch
no longer invokes `sqrtf`, while readout invokes it once when gyro data exists.
The host wrapper uses the same square-root operation; this is a proven call
reduction and numerical regression result, not a measured C3 cycle saving.
Readout can contain one extra root call, so per-call readout latency still
needs measurement even though total window work falls.

## 3. Station: contiguous live-ring copies and wrap comparisons

`SensorRegistry::liveHistory()` checks every timestamp and groups physically
contiguous fresh rows into `memcpy` runs. It retains oldest-first order,
caller capacity limits, stale-row rejection and the existing mutex/lifetime.
It does not assume all timestamps are fresh or change stored samples. A fresh
full ring has at most two physical runs. Conditional index wrapping also
replaces the insertion remainder in `registerTelemetry()`.

| Metric / S3 | Before | After | Change / evidence |
| --- | ---: | ---: | --- |
| Reader division sites | 1 inside row loop | 0 | Target `quou` removed; formerly up to one execution per scanned row |
| Full fresh 1,201-row ring copies | 1 per returned row | At most 2 runs | Inferred from actual loop and call placement; not a device trace |
| Registration division sites | 2 | 1 | Ring-head remainder removed; other division retained |
| Reader symbol/frame | 272 / 64 bytes | 374 / 80 bytes | +102 code bytes and +16 local stack bytes |
| Registration symbol/frame | 1,516 / 464 bytes | 1,532 / 464 bytes | +16 code bytes; same frame |

The production registry test covers all three capacities (1,201 IMU, 61 TMP117,
301 optical), empty/partial/full rings, multiple wraps, truncated output,
expired prefixes and `millis()` rollover, comparing original sample bytes/order.
It also retains the earlier concurrency, long-outage and persistence checks.

Two host experiments used nine runs of 1,000 full-ring reads per type/version
each. Experiment A ran before then after; B reversed that order. Same Windows
x86_64 host, Clang 20.1.2 / Zig 0.15.2, C++17 `-O2 -UNDEBUG`, inputs and
checksums were used. Experiment B observations are:

| Ring / host median | Before (range), us/read | After (range), us/read | Median change |
| --- | ---: | ---: | ---: |
| IMU, 1,201 rows | 4.334800 (4.305100-4.408300) | 4.271700 (4.246100-4.360800) | -0.063100 (-1.46%) |
| TMP117, 61 rows | 0.169400 (0.169300-0.176500) | 0.129500 (0.129500-0.132800) | -0.039900 (-23.55%) |
| Optical, 301 rows | 1.040600 (1.038500-1.042200) | 1.012000 (1.008300-1.036000) | -0.028600 (-2.75%) |

Experiment A medians were 4.288200 -> 4.176900, 0.170200 -> 0.128300 and
1.053900 -> 1.008400 us/read respectively. Both experiments and ranges are
retained in [measurements](audit/2026-11-07-mcu/measurements.json).
The small IMU change overlaps observed ranges; it is not a robust general
speed claim. These are host observations. S3/PSRAM throughput and mutex/radio
latency remain unmeasured; native removal of per-row division/calls is indirect
support for a benefit there.

## 4. Shared protocol: validate CRC without copying the payload

`validatePacket()` copies only its 16-byte header, zeroes that copy's CRC field,
then chains CRC across it and the caller's original payload. Input remains
immutable and header/type/size checks precede CRC work as before. Wire bytes,
CRC polynomial/initial/final conventions and accepted V5 packet lengths remain
unchanged. The installed Espressif `esp_rom_crc.h` explicitly documents the
previous result as the next seed; target assembly retains ROM `crc32_le` calls.
The portable host path uses the matching complemented-seed convention.

| Metric | Before | After | Change / evidence |
| --- | ---: | ---: | --- |
| C3 validation frame, V3 and V4 | 272 bytes | 32 bytes | -240 (-88.24%); actual prologues; V4 GCC confirms |
| S3 validation frame | 288 bytes | 48 bytes | -240 (-83.33%); actual prologue/GCC |
| C3 function bytes / instructions | 204 / 67 | 184 / 63 | -20 / -4 |
| S3 function bytes / instructions | 212 / 76 | 175 / 64 | -37 / -12 |
| Bytes copied for a maximum 250-byte packet | 250 | 16 | -234 (-93.60%); payload not copied |
| CRC ROM invocations per valid packet | 1 | 2 | One extra invocation; same total bytes checked |

Tests accept 940 valid cases spanning every 16..250-byte length at all four
byte alignments, reject 125,020 single-byte corruptions and wrong types/lengths,
and verify immutable input plus oversized/null rejection. Existing transport,
recording and OTA regressions still pass. Stack/code/copy reductions are proven.
The extra ROM invocation is a tradeoff: no packet-throughput improvement is
claimed without device measurements, especially for short payloads. Actual ROM
execution is not reproduced by host CRC tests.

## 5. Optical FIFO: bounded complete-frame burst reads

The [MAX30102 datasheet, FIFO sections on pages 14-15](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX30102.pdf)
documents that burst reading register `0x07` keeps the register address fixed
while successive FIFO bytes are returned. `PrecisionSensors::poll()` now
requests up to eight complete six-byte red/IR frames per transaction (48 bytes;
the installed ESP32 Wire default is 128 bytes). It never waits to accumulate
samples and drains only the original pointer snapshot. Individual frame order,
18-bit masks, backdated ages, signal/gain logic and last-eight waveform policy
are retained. Ring-head wrapping also no longer needs `% 200`.

| Snapshot / FIFO data metric | Before | After | Change / evidence |
| --- | ---: | ---: | --- |
| Five queued frames | 5 reads / 30 payload bytes | 1 / 30 | -80% read transactions |
| Eight queued frames | 8 / 48 | 1 / 48 | -87.50% read transactions |
| 31 queued frames, bounds fixture | 31 / 186 | 4 / 186 | -87.10% read transactions |
| Five-frame address/register/data clocks | 405 bits | 297 bits | -108 (-26.67%); calculated from observed request sizes |
| Eight-frame clocks | 648 | 459 | -189 (-29.17%) |
| 31-frame clocks | 2,511 | 1,782 | -729 (-29.03%) |
| C3 poll code / frame | 934 / 64 bytes | 1,060 / 144 bytes | +126 code bytes; +80 local stack bytes |
| C3 poll remainder sites | 1 | 0 | Actual disassembly |

Both versions use the same enhanced FIFO model. Every 0..31-frame snapshot
checks masked values, order, ages, payload byte count and waveform limits.
Short reads at each of four batch positions discard all acquisition state and
require successful reinitialization. Existing overflow, full/ambiguous pointer,
brownout, gain, nine pulse rates and weak-signal tests still pass.

The bit count includes three address/register bytes per transaction and nine
clocks per byte, including ACK/NACK. At 400 kHz, the five-frame lower bound is
1.0125 -> 0.7425 ms, but this excludes start/stop timing, stretching, SDK and
scheduling overhead and is **not a measured bus duration**. Status/pointer
reads are unchanged and excluded. A normal single-frame poll gets no batching
benefit. The 31-frame fixture proves the software bound, not a claim that a
healthy 25-Hz capture reaches that backlog without triggering the unchanged
1.2-second gap rule. Confirm burst and partial-read behavior on real hardware.

## Combined target cost and validation

| Target | Follow-up before Flash | After Flash | Change | Static RAM before/after | `.iram0.text` before/after |
| --- | ---: | ---: | ---: | ---: | ---: |
| V3 | 1,076,741 bytes | 1,076,617 bytes | -124 (-0.0115%) | 45,908 / 45,908 bytes | 64,186 / 64,186 bytes |
| V4 | 1,077,669 bytes | 1,077,545 bytes | -124 (-0.0115%) | 45,908 / 45,908 bytes | 64,186 / 64,186 bytes |
| S3 | 1,590,490 bytes | 1,590,566 bytes | +76 (+0.0048%) | 108,268 / 108,268 bytes | 75,779 / 75,779 bytes |

Flash is PlatformIO's used-program metric. RAM is static DRAM/noinit, not
runtime free heap or stack headroom. Relative to the clean original audit
start, the cumulative Flash changes are -96 / -96 / +828 bytes. Partition
binary hashes remain identical. The additional IMU scalar and changed local
frames introduce no new tasks, persistent formats or per-operation allocations.

All three final normal target builds passed. All **29 production C++ suites**
passed normally and with undefined-behavior/alignment/float-cast sanitizers;
the expanded balanced IMU case also passed against both baseline and current
sources. All ten embedded-page browser programs passed again. Python/JS syntax,
whitespace, inventory and final publication checks are recorded in the evidence.
The unchanged package/upload/library-patch/publication-guard regression results
remain those documented by the first audit. No firmware build or functional
test failed in this follow-up. Auxiliary analysis output was regenerated after
all compiler probes completed; decimal/hex prologue parsing was cross-checked
against the corresponding GCC reports before publishing frame values.

## Remaining work and deliberately retained boundaries

1. **Authorized hardware compatibility and latency:** old/new node/station
   combinations for ROM CRC chaining; logic-analyzer FIFO bursts, snapshot races,
   truncated transfers, overflow/brownout recovery; acquisition service gaps and
   complete estimator times under concurrent RF/flash load.
2. **Stack, PSRAM and load:** measure task high-water marks, largest free blocks
   and response tails at 19 nodes with HTTP/TLS/OTA and nearly full storage.
   Burst polling adds 80 local bytes; the S3 reader adds 16. Existing large
   configuration frames and synchronous cloud responses remain first-audit risks.
3. **Power, precision and long uptime:** controlled V4 threshold/invalid-ADC and
   rail tests, real reference sensors, at least 20 paired energy cycles per mode,
   power cuts at durable boundaries and a multi-week mixed-load soak.

No global maximum-performance claim follows from these results. Arbitrary
reassociation, fast-math, narrower compensation, SIMD substitution for mixed
packed records, clock changes and altered batching intervals lack a sufficient
correctness/performance proof here. TMP117 conversion timing and proprietary
BSEC ULP work remain unchanged; both benefit from the shared stack reduction
when validating radio packets. Peripheral waits, RF and storage can still
dominate whole-cycle energy and latency.

No device was accessed, flashed, erased, reset or physically measured. No
installation key or existing release package was changed. All work remains
local, unreleased and on the existing 4.4.3 / 40403 identity.

## Changed files in this follow-up

- Production: `sensor/src/bme280_driver.cpp`, `sensor/include/lsm6dsox_driver.h`,
  `sensor/src/lsm6dsox_driver.cpp`, `sensor/src/precision_sensors.cpp`,
  `shared/src/lil_protocol.cpp`, `station/src/sensor_registry.cpp`.
- Tests: `tests/bme280_test.cpp`, `tests/imu_peak_test.cpp`,
  `tests/math_counter.h`, `tests/protocol_validation_test.cpp`,
  `tests/optical_fifo_test.cpp`, `tests/precision_stubs/Wire.h`,
  `tests/registry_test.cpp`, `tests/run_tests.py`.
- Documentation: `tests/README.md`, `CHANGELOG.md`, `EFFICIENCY_AUDIT.md`,
  this report and `audit/2026-11-07-mcu/` evidence files.

All paths in this list are relative to `Firmware/`. Earlier changes remain in
the worktree and are listed separately in the first audit.

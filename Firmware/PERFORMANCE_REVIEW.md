# Storage performance review

This review compares the local working tree immediately before and after the
changes below, including any earlier unpublished work already present in that
tree. These results do not describe an installed device.

## Measured software effects

The tests count calls made by the production code using a memory-backed flash
substitute and real temporary files. The same inputs and compiler were used for
both versions. The counts are not elapsed-time measurements on an ESP32 or
LittleFS.

| Scenario | Before | After | Fewer calls |
| --- | ---: | ---: | ---: |
| V3/V4 journal recovery, two full TMP117 pages, 32-byte records | 254 flash reads | 66 | 74.0% |
| V3/V4 journal recovery, two full LSM6DSOX pages, 64-byte records | 128 flash reads | 34 | 73.4% |
| V3/V4 journal recovery, two full MAX30102 pages, 128-byte records | 64 flash reads | 18 | 71.9% |
| Station saved-recording read, 100 consecutive records | 103 file seeks | 4 | 96.1% |
| Station normal-history read, 80 records from one file page | 80 file seeks | 1 | 98.8% |

Journal recovery now reads at most four consecutive records in one flash call,
then validates each record and its bitmap state individually. A failed batch
aborts recovery. The result of each record's CRC validation is reused within
the scan instead of recalculating it for each bitmap decision. The temporary
batch buffer uses 512 bytes on the recovery call's stack; stack headroom and
any startup-time or energy improvement remain to be
measured on both board revisions. Writes, readback and durable ACKs are
unchanged.

The station positions once before consuming a contiguous range, while retaining
per-record CRC and value checks. A bad archive record still stops at its own
offset; a failed history page still reports an error. History snapshot bounds,
retention, complete recording snapshots, original CSV values and the separate
normal/recording storage formats are unchanged.

## Feature coverage and remaining limits

The firmware documentation and regression suites cover environmental BME280 and
BME680 acquisition, BSEC ULP scheduling, battery protection, normal precision
windows, SW2 recordings, ESP-NOW discovery and recovery, station persistence,
history/recording charts and exports, OTA, ThingSpeak and mobile operation.
This change targets their shared storage paths; it does not claim a performance
gain for every feature or prove that every path has been audited exhaustively.

At the time of this storage comparison, ThingSpeak account requests executed
synchronously in the web handler and used an unbounded response string. The
[later body-reader comparison](PERFORMANCE_FOLLOWUP_2026-11-09.md#bounded-thingspeak-response-bodies)
records the size and time bounds now applied to response bodies.

Host tests and successful target builds do not establish hardware timings,
flash endurance, rail stability, sensor accuracy, RF behavior or battery life.
Use the [hardware test plan](HARDWARE_TESTPLAN.md) to measure those properties,
including recording recovery and dashboard loading under simultaneous radio and
flash load. No device was flashed or physically measured for this review.

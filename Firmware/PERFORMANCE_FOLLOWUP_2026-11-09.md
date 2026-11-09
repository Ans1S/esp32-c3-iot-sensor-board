# Station and V3/V4 firmware measurements · 2026-11-09

Reference source is `958ea812d121e49e7d9d3fda4ad17541d610c92f` (4.4.4).
The comparison uses separate builds with PlatformIO 6.1.19, the pinned
pioarduino platform, GCC 14.2.0 and binutils 2.43.1. The station target is
`seeed_xiao_esp32s3` (Xtensa). Both sensor targets use `seeed_xiao_esp32c3`
(RISC-V, 80 MHz). The builds use the repository's public, fail-closed OTA
configuration. Installed OTA acceptance requires the matching installation key.
The measured candidate still reported version 4.4.4. The source containing
these changes identifies itself as 4.4.5 / 40405.

## Before and after

| Scenario | 4.4.4 | Candidate | Method |
| --- | ---: | ---: | --- |
| Environmental discovery across 13 channels | 13 CRC finalizations, 780 bytes | 1 finalization, 60 bytes | Counted production ESP-NOW code. All 13 transmitted frames match byte for byte. |
| Failed unicast with a MAC callback after 2 ms and no application ACK | 40 ms | 32 ms | Simulated RTOS clock, including the 30 ms grace window. |
| Station receives another measurement after the preceding send callback times out | Second packet dropped after 80 ms of additional waiting | Second packet persisted with no additional wait | Production gateway callback, dispatch and persistence queue with deterministic RTOS/registry fixtures. A sensor retry triggers a fresh ACK without storing a duplicate. |
| ADC delay calls across 16 battery conversions | 16 × 150 µs | 15 × 150 µs | Programmed waits with identical spacing between conversions. 10,000 trimmed/calibrated windows match for each board revision. |
| MAX30102 autocorrelation | 20,355 products and 20,355 accumulator additions | 7,125 each | Algorithm counts for 176 detrended samples, lags 6–51. Excludes the final normalization. |
| MAX30102 autocorrelation per window | 8.820 µs | 6.632 µs | Median of nine alternating 1,000-window x86-64 runs. Ranges 8.615–9.270 µs and 6.555–6.914 µs. All 883,200 tested coefficients match the old float bits. |
| TMP117 batch after changing from 64-byte IMU rows | 62 of 125 pending, 63 flushed early | 125 of 125 pending | Production history buffer fixture. The subsequent explicit flush and reread retain every row. |
| Live response with 601 optical points | 1,202 body writes | 101 body writes | Extracted production renderer, real ArduinoJson 7.4.3, host string substitute and a captured `sendContent` boundary. Identical 361,102-byte point-array JSON. |
| Live response with 1,201 optical points | 2,402 body writes | 201 body writes | Same fixture. Identical 719,307-byte point-array JSON. Counts exclude the header and end-of-stream marker. |

The sensor wakes on telemetry send completion instead of repeatedly polling
the callback. The receiver still matches application ACKs by sequence and
sender, including an ACK queued while a MAC completion is missing. A new
send or channel change waits for the existing callback within its deadline.
The station keeps a single frame in flight and exposes `replyBusyDrops` and
`replyMacFailures` separately from received-packet and persistence drops in
`/api/status`. A stalled reply can still require a sender retry. Espressif
documents the separate MAC and application acknowledgements and recommends
waiting for completion before sending the next frame. [ESP-IDF ESP-NOW guide](https://docs.espressif.com/projects/esp-idf/en/v5.5.6/esp32c3/api-reference/network/esp_now.html)

TMP117 invalidates its cached temperature after an I²C failure and requires a
new conversion before marking a value valid. The live HTTP cursor now filters
before copying samples from the ring. Tests check rollover, ordering and the
exact copied suffix. Browser fixtures exercise startup, paging, live and saved
charts, interaction under failures and OTA status. They substitute the API and
cannot time the station's HTTP or flash implementation.

## Follow-up on recording, history and cloud backlog

This comparison uses the first candidate as its reference. The regression
fixtures drive the production control flow with deterministic replacements
for radio, time and filesystem operations.

| Scenario | First candidate | Updated candidate |
| --- | --- | --- |
| Recording ACK arrives at 1 ms, prior MAC callback at 5 ms | A second request immediately fails, leading to sync backoff in the live loop | The next request completes at 6 simulated ms. A missing callback still prevents any retune or additional send, with a 180 ms bound measured from the original send. |
| Live contacts get MAC acknowledgements but no configuration response | Two alternate-channel attempts across three simulated contacts | Zero alternate-channel attempts. The same test still exercises both recovery attempts when MAC delivery also fails. |
| Two requested history rows near the start of a populated 91-row page | 91 checked file reads | 13 checked file reads and the same two rows, with one seek in either build. The read stops at the snapshot ceiling. |
| 32 of 64 ThingSpeak jobs expire together | 1,520 queue entry assignments | 32 assignments. The retained jobs keep their FIFO order and the drop counter advances by 32. |

The connected cloud worker now expires the queue once per iteration and
checks the Wi-Fi status outside the queue mutex. The disconnected path keeps
expiring its backlog. A history read that hits a corrupt stored row stops
before appending any newer pending rows to the partial result.

## Recording archive faults and lost-ACK replay

The previous station candidate could treat a failed tail seek or short read
as a torn record, truncate a previously stored row and acknowledge a newer
record. The archive now distinguishes a complete corrupt row from a failed
I/O operation. It aborts an append on a seek or read error and creates a new
archive only when the open reports a missing file. The tests inject each
failure at the archive I/O boundary, verify the earlier samples survive and
confirm that the next attempt appends the missing sample. Recovery of an
actually corrupt tail, validation of older replays and durable readback still
pass. The format and ACK semantics are unchanged.

An exact replay of the newest row now compares against the already validated
tail. The 100-row fixture fell from 12 seeks and 10 reads to 3 seeks and 2
reads. This is a count of file API calls on the host. Older replays still use
binary search and require identical bytes.

When the archive cannot be read, the API marks the read error and the
dashboard shows the request failure instead of waiting for synchronization.
The four affected C++ suites with sanitizers, the recording dashboard test
and the eight embedded JavaScript syntax checks passed.

## ESP-NOW receive queue admission

The observed V5 wire lengths are 60, 95, 109, 203 and 207 bytes for the
accepted telemetry formats, 42 and 160 for recording requests, 41 for their
ACK, 48 for a configuration reply and 220 for OTA. The callbacks now check
magic, protocol version, declared length and supported message type before
using a queue slot. The station dispatches on message type. Full CRC,
sequence, sender and value checks stay on the existing paths before a frame
can change state.

| Scenario | Previous candidate | Updated candidate |
| --- | ---: | ---: |
| Station RX after 64 frames with malformed or unsupported headers | 64 occupied slots | 0 occupied slots. All 64 rejects are counted and a valid telemetry packet is accepted. |
| Station RX queue payload storage with 64 slots | 17,408 B | 15,616 B, a reduction of 1,792 B |
| Four malformed frames immediately before a valid sensor response | A valid OTA response was dropped with the four-slot queue full | The first OTA exchange and a single-attempt configuration exchange both accept their valid response. |

The gateway regression exercises every telemetry length, all three control
request types and a CRC-invalid telemetry frame with a valid header. These
fixtures measure queue behavior and data storage size. The V4 receive
callback's local stack frame grew from 320 to 336 B in the linked RISC-V
code. The S3 callback's local frame dropped from 320 to 288 B in the linked
Xtensa code. Real RF throughput, callback runtime and loss under interference
require device measurements.

## LSM6DSOX FIFO transactions

The LSM6DSOX automatically wraps from register `0x7E` to `0x78` during a
FIFO burst. The driver takes the existing status snapshot and drains it in
groups of at most eight 7-byte words. The maximum request is 56 bytes against
the current 128-byte Wire buffer. Each group has one readout timestamp for
the words it contains, and validity and age use the same clock snapshot.
[ST AN5272, section 4.7.1](https://www.st.com/resource/en/application_note/an5272-lsm6dsox-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf)

| 18-word FIFO fixture | Previous candidate | Updated candidate |
| --- | ---: | ---: |
| FIFO read requests | 18 | 3 (56, 56 and 14 bytes) |
| Read requests including status | 19 | 4 |
| Calculated address, register, payload and ACK/NACK clock bits | 1,665 | 1,260 |
| Calculated clock time at the configured 100 kHz | 16.65 ms | 12.60 ms |

The clock calculation excludes start/stop timing, clock stretching, driver
overhead and scheduling. The fixture verifies signed means across burst
boundaries and discards the complete partial window when the second request
returns short. Both C3 targets build. The LSM6DSOX, IMU peak, live acquisition
and environmental dispatch suites passed with sanitizers. Across 130,566 gyro
vectors the existing peak calculation still matches its reference bitwise.
The linked V4 RISC-V disassembly confirms that the FIFO timestamp call is
outside the word loop and the report path uses one clock call instead of up to
four. The local `poll` frame rises from 96 to 176 B with the burst buffer and
compiler spills. The local `read` frame remains 48 B.

## Sensor recording journal consistency

If the ACK bitmap write reached flash and its readback failed, the previous
journal kept counting the retired record in RAM. Replay could then stall on
a page with no pending records. The updated journal retains the original and
intended bitmap word, verifies which one is on flash and reconciles the page
and total counters before selecting the next record. If the write did not
reach flash, the record remains eligible for replay. The sensor can mark the
last record synchronized after deferred verification without a restart.

The ACK path reuses the bitmap word from the header read. A successful ACK
now uses three flash reads instead of four, while it still verifies the
oldest record and the written bitmap. The injected failure test blocks two
readbacks and then confirms that the next record is selected and the pending
count reaches zero. A failed write with an unchanged bitmap leaves the
record pending.

An uncertain write during acquisition now leaves the persisted session in
its recoverable recording state. Startup uses the journal to determine the
session's actual count and duration before replay. With an injected failure
after writing the second sample, both samples are recovered and both uploads
declare the same total of two. This comparison uses the production journal
and recording-store logic with NOR-flash and Preferences substitutes.
The C3 journal object adds 12 B for the pending ACK word and its offset.
The linked V4 local frames for `acknowledge` and `locate` change from
176 B and 112 B to 240 B and 32 B. The separate reconciliation function
uses a 32 B local frame when it has an ACK word to check.

## Compact OTA block requests and durable pause progress

The station advertises compact block reads in its existing offer. A capable
sensor sends the OTA header and the 12-byte request prefix. A legacy offer
selects the original 220-byte request. The station accepts both forms and
validates the CRC and the operation before serving any short request. The
response block still carries up to 192 bytes of image data. The client also
initializes the constant read-request fields once per transfer.

| Complete 1,310,720-byte slot fixture | Original block requests | Compact block requests |
| --- | ---: | ---: |
| Number of read requests | 6,827 | 6,827 |
| Bytes per request | 220 | 28 |
| Read request bytes | 1,501,940 | 191,156 |

The difference is 1,310,784 request bytes. These counts exclude ESP-NOW
framing, retransmissions, responses and status messages. The simulated full
transfer writes the same image. Both wire lengths, a wrong nonce, a bad CRC,
unsupported short operations and checkpoint resume are exercised.
The linked V4 RISC-V `otaExchange` frame grows from 304 to 320 B.

An interrupted transfer now reports the last persisted checkpoint. In the
injected disconnect case the client writes 20,480 bytes, reports 16,384
resumable bytes and resumes at 16,384. A failed first checkpoint write reports
zero resumable bytes and restarts from zero. The station saves the reported
offset together with the pause state. Actual RF airtime, energy, signed
installation and interruption behavior require device tests.

## Target machine code

The following are sizes and local frame adjustments from the linked ELFs.
They describe generated code and static storage. Timing on a target requires
executing the code on that target.

| Symbol or path | Before | After |
| --- | ---: | ---: |
| C3 V4 `AdcReader::finishBatteryMeasurement` | 336 B, 80 B local frame, calls introsort and insertion sort | 404 B, 64 B local frame, neither sort call |
| C3 `PrecisionSensors::read` plus out-of-line correlation | 2,074 B | 2,210 B |
| C3 local frames on correlation path | 1,104 B in `read` | 1,008 B in `read` plus 144 B in `correlate` (+48 B combined before arithmetic callees) |
| S3 gateway handler plus out-of-line reply helper | 2,444 B | 2,275 B |
| S3 `PagedHistory::append` | 480 B local frame | 432 B local frame |
| S3 `SensorRegistry::liveHistory` | 374 B | 424 B, including cursor handling |
| S3 `RecordingArchive::append` | 1,785 B, 1,120 B local frame | 1,750 B, 1,104 B local frame |
| C3 V3 application flash / static RAM | 1,075,813 B / 45,908 B | 1,076,221 B / 45,908 B |
| C3 V4 application flash / static RAM | 1,076,701 B / 45,908 B | 1,077,109 B / 45,908 B |
| S3 application flash / static RAM | 1,590,366 B / 108,284 B | 1,590,938 B / 108,292 B |

On C3 the optimized correlation dot-product loop retains the software
multiply, conversion and double accumulation for `xy`. It moves the two square
sums into initialization and one update per lag. ADC compilation removes
both libstdc++ sort helpers. On S3 the gateway handler delegates the three
reply paths to one helper. Its local frame stays at 1,104 B and the helper
adds a 32 B frame while active. No handwritten assembly was needed for these
changes. The byte counts exclude shared callees and padding in other symbols.
The extra completion marker costs four bytes of sensor queue payload storage
and one nonblocking enqueue per telemetry send. The station adds eight bytes
of static RAM for diagnostics and uses a transient HTTP batching buffer.

The ignored `.tools/firmware-audit-20261109/` directory holds the six ELF
inventories, hashes and the host before/after fixtures. Build trees, raw
disassemblies and logs are in `~/wcharger-audit-20261109/` in WSL. Reproduce
the host regressions with `python Firmware/tests/run_tests.py --sanitize`
after the three PlatformIO builds. Add `--suite pulse_signal --benchmark` for
the alternating autocorrelation benchmark.

The first candidate passed all 33 sanitized C++ suites, ten Chromium dashboard
suites and the embedded JavaScript, OTA package, upload and library-patch
checks. The second candidate passed all three builds and the five affected
sanitized suites covering transport, boot, paged history, history flush and
cloud scheduling. The archive follow-up passed a new station build. The
receive queue follow-up passed all three builds and the five affected
sanitized protocol, transport, gateway and OTA suites. The V4
linked `recordingExchange` frame is still 96 B after adding the bounded
callback wait. Protocol V5 and the existing 4.4.4
release identity are kept in the local candidate.

After the journal changes, all three target builds and all 33 C++ suites
with sanitizers passed on the consolidated source. The C++ run resumed at
`recording_store` after correcting the fixture's timestamp to account for
the simulated one-millisecond page erase delay. All ten Chromium suites and
the eight embedded script syntax checks passed on the consolidated dashboard.
After the OTA changes, the three builds, five affected sanitized C++ suites
and six OTA package checks passed.

## Read-only measurements on the installed station

On 2026-11-09, a Windows client measured the reachable LAN station with fresh
HTTP connections. The installed application image could not be identified by
hash. These timings describe that installation and network. The controlled
4.4.4-to-candidate comparison still requires both known images on the same
test station.

| Request | Successful / attempted | Full response p50 / p95 |
| --- | ---: | ---: |
| Status | 60 / 60 | 109 ms / 2,821 ms |
| V3 history, 512 points in each response | 30 / 30 | 770 ms / 905 ms |
| V4 history, 512 points in each response | 30 / 30 | 616 ms / 679 ms |

The status run overlapped with the start of the history captures. The history
rows represent different stored datasets. During the status run, the available
ESP-NOW receive and drop counters did not change. The station did not expose
the candidate's two new reply counters.

A follow-up using the default address-family lookup reproduced three requests
over one second in 24 attempts. Windows name resolution took 2.71–2.72 s on
those requests, followed by 19–42 ms for TCP and 33–53 ms to receive the HTTP
headers after connecting. All three connected on the first IPv4 attempt. In a
separate 24-request alternating run, fresh IPv4-only lookups had a 121 ms
full-response maximum and connections to the already resolved address had a
96 ms maximum, with 12 requests in each group and zero errors. The measured
multi-second delay occurred before TCP connection. For the controlled station
comparison, use the same direct LAN address for both images and measure
hostname access separately.

The numeric captures are retained locally under
`.tools/firmware-audit-20261109/live-*.json`.

## Continued firmware optimization

This comparison uses the software candidate above as its reference. The
optical waveform buffer on C3 now replaces its oldest slot in place. A full
31-sample FIFO drain previously moved seven retained samples for each of the
23 overflowing inserts. It now moves no retained samples and returns the same
last eight red/IR values, ages and gap flag. The target wraps the eight-slot
index with a bit mask. In the linked V4 ELF, the environmental sensor object
uses 16 fewer static bytes and the optical poll's local frame drops from
144 B to 112 B. Poll and read together shrink from 2,808 B to 2,770 B.

The S3 paged history reader now fetches up to eight stored rows at once and
validates each consumed row. The production filesystem fixture reads an
80-row page in 10 `fread` calls instead of 80, and the bounded two-row window
in two calls instead of 13. Both still use one seek. A short read or CRC error
inside the selected window stops at the valid prefix. Unconsumed bytes from
the last batch do not affect a completed window. The linked `read` local frame
grows from 864 B to 1,248 B for the larger buffer, with no added static RAM.
These are stdio call counts and compiled sizes.

Both C3 builds and the S3 build passed. The five targeted sanitized suites
passed for optical FIFO, precision sensors, paged history, flush and registry.
The before/after disassemblies are retained as `turn8-*.dis` in the existing
WSL audit directory.

### Cloud worker deadlines and IMU accumulation

When Wi-Fi is connected, the cloud worker waits for the first queued upload
that can run or expire. New jobs still notify the worker immediately. While
offline the worker checks for a reconnect every 250 ms when work is pending.
For a channel blocked by the 15 s rate limit, the deterministic scheduler
fixture needs one timed check instead of 60, excluding the initial check and
new-job notifications. A differential run against the 4.4.4 scheduler agreed
on 39,969 selection decisions, including 27,578 selected uploads. All 12,391
nonempty waits ended at the first eligibility or expiry boundary. The linked
S3 worker grows by 72 B of code and 16 B of local stack. No static RAM is added.

The LSM6DSOX driver now accumulates the six unscaled axes as signed integers
and converts them to physical units at the end of each window. Peak, step and
activity calculations retain their per-sample floating-point operations.
The C3 disassembly replaces a software floating-point addition in the
three-axis FIFO loop with integer additions and carry handling. It removes
three software additions per accepted FIFO word and moves the mean's scaling
to readout. Across 256 simulated windows covering 1–256 signed sample pairs,
the largest change from the previous mean was 0.00000024 g and 0.000031 °/s.
An additional window with 65,792 signed-extreme sample pairs checks totals
beyond a signed 32-bit sum. The C3 build uses 32 B more static RAM. In the V4
ELF the poll's local frame falls from 176 B to 160 B, while poll and read
together add 102 B of code. [ESP32-C3 datasheet](https://documentation.espressif.com/esp32-c3_datasheet_en.html)

Both C3 builds and the S3 build passed, along with the sanitized LSM6DSOX and
power/CRC/upload suites. The differential harness and disassemblies are in
the existing WSL audit directory as `turn9-*`.

### ESP-NOW response waits and streamed JSON

The OTA and recording consumers now block for the remaining response deadline
in FreeRTOS ticks. An arriving packet wakes them immediately. Source, packet,
sequence and application matching still occur within the original retry
budgets. With no recording ACK the simulated 500 ms wait uses one timed queue
receive instead of 50, including across tick rollover. With no OTA reply and
five successful MAC sends, the simulated 3,500 ms budget uses five timed
receives instead of 175. The existing late-MAC and stale/wrong-response cases
still pass. The target uses tick subtraction rather than converting a variable
millisecond remainder on each wait. The linked V4 local frames are 320 B for
OTA (unchanged) and 112 B for recording (previously 96 B). The linked V3/V4
builds add 36/32 B of flash and no static RAM.

The S3 saved, live and paged history renderers append point JSON directly into
their existing HTTP chunk, using 64 B of local staging for short serializer
writes. Saved TMP117 pages of 128 points use 384 string appends rather than
18,699. LSM6DSOX and MAX30102 pages of the same size use 512 rather than
28,972 and 28,615, respectively. Live and paged history no longer allocate
their intermediate point strings. The differential fixture extracted the
production rendering loops and used pinned ArduinoJson 7.4.3. It verified
byte-identical arrays and chunk counts for all tested sizes, including 445,988
bytes for 1,201 mixed live points and 76,644 bytes for 512 paged points. If a
point cannot be serialized into the chunk, the station closes the connection
before the final HTTP chunk so the client can detect truncation. The S3 adds
128 B of application flash and no static RAM. The append helper has a 128 B
local frame in the linked ELF. [ArduinoJson custom writers](https://arduinojson.org/v7/api/json/serializejson/#custom-writer)

Both C3 builds and the S3 build passed. The sanitized transport, OTA client and
recording JSON suites passed. The differential programs and disassemblies are
in the WSL audit directory as `turn10-*`.

### Station recovery and flash verification, BSEC scheduling

Station history startup, readback and API paging now share an eight-row
buffered reader. The startup scan still retains only the valid chronological
prefix. Flush still checks exact persisted bytes, CRC and decoded values for
every row after `fsync`, and retains all uncommitted rows on a readback fault.
It reuses the first validated timestamp instead of decoding that row again.

| Full page | Flush `fread` calls before / after | Startup `fread` calls before / after |
| --- | ---: | ---: |
| BME280, 91 rows | 91 / 12 | 92 / 13 |
| TMP117, 126 rows | 126 / 16 | 127 / 17 |
| LSM6DSOX, 63 rows | 63 / 8 | 64 / 9 |

These are production stdio calls on identical host files. The flush still
uses two seeks and the startup counts include the header. Short reads and
corruption inside a batch were exercised, along with torn tails and retries.
The generated S3 path buffers fit the fixed history filename and mount point
(42 B instead of 384 B in the target build). In the linked ELF the local
frames for startup, flush and API reads fall from 1,248/976/1,248 B to
976/848/928 B respectively. The application uses 64 B less flash.

BME680 rounds the absolute BSEC schedule to milliseconds once and derives
the remaining delay by subtraction. On a positive future schedule with no
new output, the previous C3 code invoked two signed 64-bit divisions and two
64-bit remainder helpers. The new code uses one unsigned 64-bit division.
The original and updated production drivers both pass the same fixture for
fractional deadlines, the exact budget boundary, early fallback and the
extreme positive timestamp. Both C3 builds use 116 B less application flash.
The linked V4 local frame for `readBsec()` stays at 112 B.

All three target builds and six affected sanitized suites passed. The local
baseline and candidate logs and disassemblies are in the WSL audit directory
as `turn11-*`.

### Bounded ThingSpeak response bodies

The station uses the existing Arduino HTTP client for the HTTPS request and
response headers, then reads the body with a bounded reader. It accepts
Content-Length, chunked bodies with extensions and trailers, and bodies that
end at connection close. The total body budget is seven seconds from entry,
including when the peer keeps sending occasional bytes. The payload limits are
64 KiB for account requests and 1 KiB for telemetry upload responses. Empty
success responses and operations that only need the status bypass body reads.
For HTTP errors the station reports the code without buffering the remote
error page. An incomplete successful mutation response asks the operator to
refresh account channels before retrying. [ThingSpeak account-list API](https://www.mathworks.com/help/thingspeak/listyourchannels.html)

The pinned previous HTTP client reserved advertised lengths without an
application cap. Its close-delimited read loop could wait for more data while
the connection remained open. The new reader rejects a declared oversized
body before reserving or reading payload bytes. It uses 512 B of local staging
instead of the SDK body reader's per-block heap buffer of up to 4,096 B.
A known allowed length reserves once. In the fragmented unknown-length fixture,
32,769 bytes needed eight capacity increases. The local TCP fixture retained
identical 32,003-byte payloads across the three framings, rejected a declared
9,000,000-byte response before reading any body and stopped an open idle
response at the configured 50 ms test deadline. Deterministic tests also cover
short reads, clock rollover, trickle data, bad framing and allocation failure.

The S3 build adds 2,740 B of flash and no static RAM. The linked body helper
uses a 624 B local frame, and chunk parsing can add a 176 B frame. The account
handler still runs on the station web task. Its TLS and header phases use the
existing client and have their own timing, separate from the body deadline.
The station build and sanitized HTTP body, upload scheduler and gateway suites
passed. Local loopback code and Xtensa disassembly are in the WSL audit
directory as `turn12-*`.

### Background ThingSpeak channel list

The dashboard starts a channel list with a CSRF-protected POST to
`/api/thingspeak/channels` and polls its ticket with GET. The existing GET
without a ticket remains available for synchronous API clients. Both paths
use the same renderer and return the same completed channel-list format.
The web task only submits or collects the request. The existing ThingSpeak
worker handles TLS, headers and response-body reads. The queue mutex is
released before network I/O, so telemetry can still be queued during a list.

One slot holds a pending or completed list. Requests from the same browser
session for the same account reuse its ticket until collection. A different
session or account cannot collect it. The worker reclaims an abandoned result
after its 30-second retention period when it next checks the queue, and
collection and new submissions also check expiry. A channel list can wait
behind an in-flight upload. Existing legacy requests and administrative
mutations can still occupy the web task.

The host fixture executed the extracted production task loop with a network
operation held behind a test gate. Polling, duplicate submission, session
rejection and telemetry admission all completed while the cloud reply was
held. One list ran, and the queued telemetry upload followed. The browser
fixture sent one POST for two concurrent dashboard callers, completed a
local status request while the list was pending, rendered the eventual list
and released the button after a busy response. Four affected sanitized
suites, the Chromium fixture, eight embedded-script checks and the S3 build
passed.

The S3 build adds 3,540 B of flash and 128 B of static RAM. The linked
worker's local frame grows from 656 B to 752 B. A station that previously did
not use cloud uploads now allocates the existing 10 KiB worker stack at its
first background list. The worker fixture and Xtensa disassembly are in the
WSL audit directory as `turn13-*`.

### Background ThingSpeak channel mutations

The dashboard submits quick-create, create, update, clear and delete as
ticketed requests. The existing POST routes still accept synchronous clients
that omit `requestId`. The account worker is shared with listing and upload
jobs, with one slot for account work. While a mutation is outstanding, a
second distinct mutation receives a busy response. An identical active
request identifier from the same session and account returns the same ticket.
The dashboard polls for the result and does not resubmit the mutation.

The worker performs network I/O without the queue mutex. The web task applies
the result against the current local state as soon as it becomes available,
independently of browser polling. A new profile uses the current first free
slot. Settings from a previous account or a result that arrives after a
factory reset cannot re-create local data. The result becomes available to
the browser after local completion. The job state and receipt live in RAM.
A reboot between remote success and local application can therefore still
require refreshing the account and reconciling local profiles.

The host worker fixture held a synthetic network response while admitting a
telemetry job and polling the pending mutation. The local apply fixture
executed the production completion handlers without browser polling. It
covered slot re-selection, NVS write failure, registry rollback, account and
factory-reset changes, clear and delete, and failed remote requests. The
Chromium fixture used all five mutation routes, obtained status during a
pending job, and confirmed that result polling did not repeat a POST. Five
affected sanitized suites, eight embedded-script checks and the S3 build
passed. All cloud operations in these fixtures were synthetic.

Compared with the preceding build, the S3 image uses 15,940 B less flash
and 64 B more static RAM. The worker's linked local frame grows from 752 B
to 816 B. Xtensa disassembly and the extracted worker/application fixtures
are retained as `turn14-*` in the WSL audit directory.

### Report-driven IMU FIFO and optical metadata burst

The acquisition task now drains the LSM6DSOX FIFO at its 50 or 100 ms report
boundary. It still runs the five-millisecond button loop between reports.
At 104 Hz for each of accelerometer and gyroscope, at most 22 FIFO words
arrive in a nominal 100 ms window. The FIFO holds 512 words. The driver
checks for overflow and rejects a window after a polling gap. [ST LSM6DSOX application note](https://www.st.com/resource/en/application_note/an5272-lsm6dsox-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf)

The comparison executes the production acquisition task and driver with a
104 Hz FIFO and RTOS substitute, including the switch from 50 to 100 ms after
five minutes. The 5,978 uninterrupted reports match in capture timestamp,
validity, sample count, axes, peaks, step count and activity counter. Task
polls fall from 66,738 to 5,978. Counted I²C read requests fall from 98,964
to 18,040, including 32,217 to 12,053 FIFO bursts. In a separate 230 ms stall
fixture, both versions report a gap and recover the same valid measurement
windows. The activity counter differs by one displayed second in one report
because fewer samples from the discarded window were processed before the
stall. Absolute sample ages are inferred by the driver at drain time.

The MAX30102 reads interrupt status and FIFO pointers in one auto-increment
transaction from 0x00 through 0x06. It also reads the disabled die-temperature
interrupt status. The metadata phase uses one read instead of two, with the
same minimum address/register/data clock count. The FIFO drain still reads
the same original snapshot. The fixture covers every 0–31-sample snapshot,
waveform order and ages, brownout/overflow and short burst recovery.
[Analog Devices MAX30102 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX30102.pdf)

Both C3 target builds and four affected sanitized suites passed. The linked
V4 local frames remain 512 B for the acquisition loop and 112 B for the
precision poll. Static RAM is unchanged, and V3/V4 use 4/8 B more application
flash. The local FIFO fixture and RISC-V disassembly are in the WSL audit
directory as `turn15-*`.

### Station status metadata and BME280 readiness

The station maintains durable history counts and time bounds when a page
write has been verified. Page removal invalidates that sensor's summary, so
the next query rebuilds it from the retained pages. The status handler uses
the same metadata snapshot for retention and revision and reads one clock
value for the sensor ages. The legacy history route still supplies its own
revision when the paged backend is absent.

In the warm 19-sensor status fixture, the history lookup uses 190 MAC
comparisons instead of 9,766 and returns identical retention fields and
revisions. Tests also compare reported bounds with data after quota eviction,
re-enable a retained history, check flush failures and reset the summary after
deletion. The S3 application uses 240 B more static RAM and 412 B more flash.
The linked status handler's local frame grows from 368 B to 384 B.

The BME280 measurement reads adjacent status and control registers together.
In the ready case, the driver uses two I²C reads instead of three, including
the separate pressure/temperature/humidity burst. It still validates the
measurement mode before reading results and reads an extra byte per status
check if the conversion is still busy. The baseline and updated drivers
pass the same delayed-conversion, short read and incorrect-control fixtures.
All 1,064,960 compensation cases retain the reference float bits and validity.
The linked V4 read frame remains 384 B. [Bosch BME280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)

All three target builds, all 35 sanitized C++ suites, all 11 Chromium suites,
eight script syntax checks, and the OTA package, library-patch, upload-helper
and benchmark-tool regressions passed on this combined source. The fixture and
disassemblies are in the WSL audit directory as `turn16-*`.

### Global history eviction under storage pressure

The station groups a temporary array of page indices by sensor MAC when it
needs to reclaim global history space. It chooses the oldest page of the
sensor with the most pages, using the oldest generation to break owner ties.
The underlying page array stays in generation order. Retained files from
retired sensors participate in the same policy.

The production selection matches the old algorithm on 10,000 distributions
from empty through full, including ties. With 256 pages, the number of MAC
comparisons falls from 65,536 to 1,594 for one owner, 2,275 for 19 owners and
2,760 for 256 owners. The filesystem fixture forces real eviction in the host
store and verifies the tied retired owner, the next active owner, and the
resulting retention metadata. The S3 build adds 856 B of flash and no static
RAM compared with the preceding candidate. The linked selector uses a 320 B
local frame, including its 256-byte index array. The fixture, station log and
Xtensa disassembly are in the WSL audit directory as `turn17-*`.

## Final software acceptance

The scope is the ESP32-S3 station and both ESP32-C3 board revisions. The
comparisons use the 4.4.4 reference or the preceding candidate identified in
each section. Counts from host fixtures describe the named operation, and
compiled sizes describe the linked binaries.

| Area reviewed | Evidence on the completed candidate |
| --- | --- |
| Acquisition, I²C and power | ADC settling and calibrated sample tests cover both dividers. BME280 and BME680 exercise conversion deadlines, bus failures and calibration. TMP117, MAX30102 and LSM6DSOX suites cover freshness, FIFO bursts, missed windows, recovery and rail handoff. The 104 Hz comparison runs the production acquisition loop and IMU driver. |
| Computation | Host comparisons cover the BME280 compensation, pulse correlations, signed IMU means and BSEC schedule boundaries. RISC-V output confirms the software floating-point and 64-bit arithmetic changes. |
| ESP-NOW and task flow | Deterministic transport and gateway fixtures cover MAC and application ACK ordering, V5 wire compatibility, environmental discovery, retry and retune, receive pressure, recording and OTA deadlines, and persistence while a reply stalls. The cloud-worker fixture covers telemetry admission while a remote request waits. |
| Web and API | Live, recording and paged JSON fixtures cover byte compatibility, chunk counts and corruption handling. Eleven Chromium suites cover responsive rendering, status/history concurrency, user actions, and the embedded dashboard's asynchronous cloud operations. Legacy ThingSpeak calls remain synchronous. |
| Storage and OTA | Sanitized tests cover both recording journals and station history/archives, short and corrupted reads, retry, quota eviction, power-loss substitutes and OTA cancellation/resume. The selected storage suites were rerun after the final eviction change. |
| Cloud data | Scheduler fixtures cover expiry, FIFO order, deadline waiting and retries. HTTP body tests and local TCP fixtures exercise fixed, chunked and close-delimited framing, oversize rejection and timeout. Worker, local-apply and browser fixtures cover background mutations, failed local saves and account/reset isolation. |

| Target | Application flash, 4.4.4 → final | Static RAM, 4.4.4 → final | Final build |
| --- | ---: | ---: | --- |
| ESP32-C3 V3 | 1,075,813 → 1,076,185 B | 45,908 → 45,924 B | `turn16-sensor.log`, `sensor_pcb_v3` |
| ESP32-C3 V4 | 1,076,701 → 1,077,073 B | 45,908 → 45,924 B | `turn16-sensor.log`, `sensor_pcb_v4` |
| ESP32-S3 station | 1,590,366 → 1,582,718 B | 108,284 → 108,724 B | `turn17-station.log`, `station_s3` |

The integrated source passed all 35 sanitized C++ suites, 11 Chromium suites,
eight embedded-script checks and the package, upload-helper, library-patch
and measurement-tool tests before the last station storage edit. That edit
passed the S3 build, four affected sanitized suites and the eviction
differential. A checksum-based comparison verified that the build and test
tree matched the workspace source. Sensor conversion timing and rail margins
remain tied to the existing hardware configuration. The separate device
qualification below is outside this software acceptance.

## Optional on-device qualification

The current optimization goal uses software tests, target builds and local
comparisons. The following device scenarios remain available for a separate
hardware qualification.

Use the same station, V3 and V4 hardware, power source, sensors, network and
firmware configuration for each paired run. Follow the existing
[hardware test plan](HARDWARE_TESTPLAN.md) for acquisition and recovery cases.
The read-only station capture client in `tools/capture_station_bench.py`
collects HTTP timing, radio and cloud counter deltas, and heap samples. The
four loopback tests cover real HTTP requests to a synthetic station, field
filtering, reboot detection, redirects and preservation of existing capture
files. The client records the SHA-256 of the supplied application binary.
The opt-in `SENSOR_RADIO_BENCHMARK` trace complements it on V3 and V4. It
records one numeric row after each telemetry contact, with boot and sequence
identifiers, accepted send requests, application ACK and elapsed microseconds.
`tools/summarize_radio_bench.py` reports per-mode counts, ACK rates and
latency percentiles. Three parser tests and a production formatter-to-parser
smoke test passed with synthetic serial data and a substituted clock.
The 4.4.4 reference and the candidate both build with identical trace
instrumentation on V3 and V4. The four sensor test binaries, two matching
station application binaries and verified checksums are in the ignored
`.tools/firmware-audit-20261109/radio-bench/` directory. The reference overlay
and build logs are retained locally. Both ordinary sensor builds passed the
boot and transport checks. Their linked `.flash.text` and `.iram0.text`
sections match the corresponding pre-instrumentation candidate byte for byte.

| Area | Record for both firmware versions |
| --- | --- |
| Radio | Application ACK success and retries by sequence, duplicate persistence, p50/p95/p99 contact latency, callback stalls and all drop counters. Test concurrent nodes, lost ACKs, interference and AP channel changes while serving history and uploading to ThingSpeak. |
| Acquisition and power | I²C transactions and timing with a logic analyzer, fresh sample rate and accuracy for each sensor, I²C interruption and recovery, V4 battery threshold/settling, wake time and charge per measurement. |
| CPU and memory | On-device cycle counts for ADC and correlation, radio and acquisition task high-water marks, internal heap minimum and largest free block during parallel work. |
| Web, storage and cloud | Actual time to first byte, complete graph load and chunk counts, flash flush latency and recovery after interrupted writes, upload backlog and long-running throughput. |
| OTA and reboot | Matching signed installation packages, interruption and resume, rollback and preserved configuration/history on all targets. |

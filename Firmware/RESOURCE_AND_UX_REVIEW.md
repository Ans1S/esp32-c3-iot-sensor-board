# Dashboard usability, long-running operation and storage review

Reviewed: 2026-10-25. Scope: the current station, PCB V3/V4 source, partition
tables, production host regressions and embedded browser pages. This review
preserves the existing visual language and the separate normal-measurement and
SW2-recording workflows. It is not a hardware endurance or accuracy certification.

## Interface research and application

The useful patterns are established interaction principles, rather than a
requirement to adopt a large application framework or a different appearance.

| Principle | Application to W-Charger |
| --- | --- |
| Progressive disclosure | Keep names, current readings and important faults in the overview. Open the selected sensor for history and put secondary chart controls in a labeled disclosure. Keep frequent measurement/time choices visible. |
| Recognition rather than recall | Use a short text label with unfamiliar icons. A battery value opens battery history; editing has its own explicit action. Restore the last sensor, source, window and scroll position. |
| Adaptive list/detail navigation | On a phone, show the sensor list or one sensor's details. Use a clear Back action; do not squeeze desktop sidebars into a narrow screen. |
| Prioritized mobile content | On a chart-focused detail screen, show the selected measurement and chart first. Other current readings and device information can expand below or above it through a labeled control. |
| Appropriate touch targets | Aim for 44 CSS px for chart actions and reorder controls on coarse pointers. WCAG 2.2 AA's minimum is 24 CSS px with exceptions; 44 px is a stronger usability target, not the AA minimum. |
| Stable interaction | A tap pins a sample. Vertical page scrolling remains available; horizontal movement scrubs samples. New status data must not remove the selection or move keyboard focus. |
| Clear hierarchy | Current values and units are prominent. Timestamps, sampling interval and status are secondary. Use one semantic status color with text, rather than decorating every tile differently. |

Research sources:

- [Nielsen Norman Group: Progressive disclosure](https://www.nngroup.com/articles/progressive-disclosure/)
- [Nielsen Norman Group: Recognition and recall](https://www.nngroup.com/articles/recognition-and-recall/)
- [Android Developers: Common layouts](https://developer.android.com/design/ui/mobile/guides/layout-and-content/common-layouts)
- [W3C: Target size minimum](https://www.w3.org/WAI/WCAG22/Understanding/target-size-minimum)
- [W3C: Target size enhanced](https://www.w3.org/WAI/WCAG22/Understanding/target-size-enhanced)
- [Apple: Charts](https://developer.apple.com/design/human-interface-guidelines/charts)
- [W3C: Pointer events](https://www.w3.org/TR/pointerevents3/)

For measurements over time, retain line plots with visible isolated points and
explicit gaps. Use a sparse neutral grid, familiar tick increments, readable
units and an exact readout outside the finger's contact area. A bar chart is
appropriate for totals, not a substitute for an irregular temperature trace.
Avoid 3D charts and unnecessary simultaneous plots. Do not force a temperature
or voltage line to start at zero when that would hide meaningful variation.

CSS Grid, media/container queries, native form controls and native disclosures
can implement this hierarchy without a framework. SVG remains suitable for
bounded plots. For longer histories, bound the display geometry to screen
width: retain the first/last/minimum/maximum per display column, in time order,
and process each gap-delimited segment separately. That is display reduction,
not averaging stored measurements. Exact inspection must use original samples,
and recording CSV must continue to export the full selected recording.

Long-history requests need bounded pages, cancellation, revision/snapshot
checks and a focused time range. A phone should not receive every sensor's
entire history before its first useful screen. Current values can share one
status poll, while selected history updates only when its revision changes.

## Confirmed faults corrected in this review

| Area | Failure mechanism | Correction and evidence |
| --- | --- | --- |
| Precision normal acquisition | An old signed retry deadline becomes a future deadline after 2^31 ms, approximately 24.86 days. Normal windows can then stop starting. | Use an explicitly active unsigned-elapsed retry backoff. The production acquisition loop is tested across a simulated long uptime and real interval change; the previous production source fails that regression. |
| Recording synchronization | The replay deadline could remain zero or long expired, producing the same multi-week stall when a later recording needs synchronization. | Idle replay has no deadline. Only a real failed exchange/acknowledgement starts a one-second backoff; success clears it. Test long idle periods and 32-bit wrap. |
| Login throttling | A long-expired 32-bit block could become active again after weeks without a login. | Use a tested 64-bit monotonic throttle with the existing five-attempt, five-minute-window and 30-second-block policy. |
| Station contact age | A sensor absent for a full 32-bit millis cycle could appear recently seen again; a stale motion reference could pass the age check. | Keep the short live-ring ID clock and add a 64-bit monotonic contact timestamp. Use it for status age and reference freshness; saturate the packet interval instead of wrapping. |
| Browser station uptime | A 32-bit uptime wrap looked like a station reboot and cleared client history state. | Report the monotonic 64-bit uptime already available from the ESP timer. |
| Touch inspection | Touch pointer-up is followed by pointer-leave; the old leave handler hid the selected marker. | Distinguish hover, scrolling and pinned selection. Add adjacent-sample/clear actions, preserve pinned original values during incremental updates, and identify a selection that leaves the rolling window instead of snapping it to a different sample. |
| Unused ThingSpeak | Startup always created the 10,240-byte upload task stack and woke the task every 250 ms even without uploads. | Create the worker only for a valid enabled upload. Notify it on new work and block indefinitely while its queue is empty. The bounded scheduler and TLS serialization remain in place. |

The cloud scheduler's statically allocated queue and the small initialization
mutexes still exist when cloud upload is unused. Once created, the upload task
keeps its stack and sleeps when idle. This change does not claim to reclaim
all cloud-related memory or measure an on-device power saving.

The status API additionally exposes free/minimum/largest-block internal heap
and free/largest-block PSRAM. Total free bytes alone do not establish that a
large contiguous allocation can succeed. See
[Espressif heap diagnostics](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32s3/api-reference/system/heap_debug.html).

## Where the measurements live today

| Data | Sensor PCB V3/V4 | Station |
| --- | --- | --- |
| Normal BME/battery telemetry | Latest working/RTC values; no persistent offline time-series backlog | Adaptive CRC-checked LittleFS pages; bounded pending batches below 60 s. |
| Normal TMP117/LSM6DSOX snapshots | Independent short acquisition windows; no automatic recording journal entries | Typed persistent normal-history pages preserving float values. Manual recording live data remains separate. |
| Active SW2 session | Bounded acquisition queue followed by a CRC/commit/ACK flash journal | Recent live RAM view; full journal replay starts after stop. |
| Synchronized SW2 session | Pages become reusable after durable station acknowledgement | Persistent recording files until explicit deletion; shared filesystem capacity. |
| Browser | No acquisition ownership | Temporary loaded history/recording snapshots. Closing it does not stop collection. |

Consequently, extending normal station history does not require duplicating it
on every sensor. It does not create offline delivery guarantees for ordinary
BME reports. Extending the recording duration is a separate capacity decision.

## Sensor flash and bounded RAM

Both targets use `sensor/partitions_recording.csv`: 4 MiB flash, two
`0x140000` application slots, a `0x160000` recording partition, separate NVS,
OTA metadata and coredump partitions. The recording partition contains 352
4,096-byte pages, with 64 bytes per page reserved for page identity, CRC and
separate commit/acknowledgement bits.

| Sensor | Bytes per stored report | Reports in a healthy empty journal | Nominal duration at the existing recording rate |
| --- | ---: | ---: | --- |
| TMP117 | 32 | 44,352 | About 12 h 19 min at one report/s |
| LSM6DSOX | 64 | 22,176 | About 31 min 58 s: first five minutes at 20 reports/s, then 10 reports/s |
| MAX30102 | 128 | 10,912 | About 36 min 22 s at five reports/s |

These are complete reports, not individual axes or optical ADC numbers.
Optical reports contain multiple red/IR pairs. A corrupt/unusable page or a
previous unsynchronized session can reduce usable capacity. Full capacity
stops a session; it does not overwrite pending reports or grow a RAM buffer.
The existing 30-minute motion profile needs at most 21,000 reports.

Source-derived type sizes were checked with a compiled C++ probe:
`TelemetryPayload=191`, `HistorySample=40`, `LiveSample=196`,
`EnvironmentalReading=184`, `LiveCapture=200` bytes. The acquisition queue
therefore contains 12,800 bytes of capture payload for its 64 slots, plus RTOS
metadata. Its task requests 6,144 stack bytes. Journal page bookkeeping is
approximately 4.1 KiB plus allocator/vector overhead. These structures stay
bounded as a recording grows; queued losses are marked as gaps.

No percentage of the recording partition needs to be borrowed for the Wi-Fi
heap: flash partitions and RAM are separate. There must still be runtime RAM
headroom and sufficient filesystem workspace on the station. Full or faulty
storage must be handled as a state, not as permission to format existing data.

## Implemented adaptive station retention

The station partition table is unchanged: LittleFS has `0x1a0000` bytes
(1.625 MiB); dedicated OTA staging is separate. Normal history and saved manual
recordings share this filesystem. Normal history uses at most 256 files of
4,096 bytes, with a 64-byte header and row CRCs. Admission budgets an additional
512 bytes per page for metadata, checks actual free bytes, and retains at least
256 KiB of workspace plus a temporary 64 KiB reserve during archive work.

Each provisioned normal-history consumer gets a fair page quota. Hidden nodes
remain consumers. Saved recordings and other files reduce the available quota.
When space is needed, only the oldest normal-history pages are recycled; a
saved recording is never silently removed. File/page errors remain bounded
failures and do not trigger formatting or growing buffers. Existing compatible
environmental rings are imported before their legacy file is removed; failed
migrations retain that file for retry after restart.

| Normal sample kind | Bytes per row, including CRC | Rows per page | Maximum rows with one consumer | Upper fair share with 19 consumers |
| --- | ---: | ---: | ---: | ---: |
| BME/battery | 44 | 91 | 23,296 | 1,183 |
| TMP117 | 32 | 126 | 32,256 | 1,638 |
| LSM6DSOX | 64 | 63 | 16,128 | 819 |

These are upper slot budgets from the 256-page cap, not promised collected
durations. Actual filesystem overhead, archives, interval/type changes and
failures can reduce them. For BME/battery, a healthy otherwise empty station
can therefore approach 6 h 28 min at 1 s or 242 days at 15 min with one node;
the upper fair share at 19 nodes is about 19 min 43 s or 12.3 days respectively.
The detail view shows actual oldest/newest data, saved/buffered points and
estimated capacity. Period presets extend beyond 24 h and All retained data
uses the currently available span.

Normal samples use at most one 4,032-byte pending buffer per node in PSRAM,
allocated only with more than 1 MiB left. If PSRAM is unavailable, the bounded
512-byte fallback requires 128 KiB of internal-heap headroom. No full normal
history is cached on the station. Short intervals flush on buffer capacity or
after 30 s through a round-robin worker; with 19 busy nodes this adds up to
about 5 s of scheduling delay, excluding storage faults or scheduling stalls.
A sudden power loss can lose that uncommitted tail. Long intervals flush each
normal report. Manual journal ACKs still require durable readback success.

A response contains at most 128 samples (about 25 KiB of snapshot payload),
with an internal-heap reserve if PSRAM allocation fails. A fixed until/oldest
cursor detects eviction during a multi-page read. The browser requests only
the opened node and updates incrementally; closing/changing details aborts the
read. Large SVG paths preserve time-ordered first/last/min/max values per
display bucket, separately across gaps. Raw inspection/CSV retain every loaded
sample. No long-term averaged tier replaces original values.

Normal TMP117/IMU samples use the recording codec's floats without extra
quantization. Environmental values retain the previous compact centidegree,
centipercent, decihectopascal and millivolt resolution. MAX30102 stays SW2-only.
Ordinary telemetry has no sensor-side persistent offline backlog; increasing
station retention does not enlarge sensor RAM or its manual journal.

The worst simultaneous manual live-ring payload remains about 4.27 MiB for
19 IMUs, plus task/queue/snapshot overhead. Those rings are not allocated for
normal precision windows. Failed large allocations are rejected with the
existing internal reserve. Actual PSRAM startup, largest free blocks, task
watermarks and concurrent radio/TLS peaks require hardware measurements.

## Validation and remaining hardware evidence

All 23 production host suites cover the existing drivers, battery protection,
transport, recording/OTA failures and new normal-history store. The new store
tests use real temporary files with 19 nodes, full-space refusal, fair retention,
preserved archives, exact precision values, snapshots and CRC/torn-tail recovery.
Registry tests exercise legacy migration and fresh normal precision samples.

Seven embedded-page browser suites cover the compact overview, full-card and
full-value-tile touch navigation, persisted layout, multi-week paging, geometry
reduction, raw values, pinned cursors, stable forms/menus, saved recordings and
OTA status. Builds cover the station and both PCB revisions. Packaging verifies
the original signing identity, prior trusted packages and final payload bytes.

Build sizes and checksums are recorded with the release. Static sections and
slot fit do not prove runtime free heap, RF latency, flash endurance, current,
sensor accuracy or physical power-cut recovery. Before hardware acceptance,
run the hardware plan with 19 mixed nodes, nearly full archives, failed PSRAM,
multiple clients and concurrent OTA/cloud traffic. The BME forced/ULP schedules
and V4 2800/2950 mV protection with 24-hour deep sleep are preserved.

No board was flashed or measured, no partition table was changed and no
installation signing identity was replaced during this implementation.

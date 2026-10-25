# Firmware change inventory

## 4.4.1 / 40401 — 2026-10-25

- Refresh the compact README with current overview, history, settings, saved
  recording and firmware-update views, plus a concise guide to each workflow.
  Render all examples locally from the embedded pages with fixed demo data.
- Extend local-data exclusions and publication guards to device exports,
  flash dumps and additional authentication files. Preserve release-text bytes
  across checkouts so distributed checksums also work for a fresh clone.
- Expire the station's live ring using a 64-bit monotonic timestamp. A full
  `millis()` wrap cannot revive old samples, and cached contacts or the first
  fresh sample after a long outage cannot reintroduce them. The existing
  one-minute window, sample IDs and recording formats remain unchanged.
- Preserve a reopened dashboard-layout draft when an earlier save completes.
- Show a retryable history timeout while retaining the previous complete
  snapshot. Navigation cancellation remains silent and cannot overwrite the
  selected sensor's state.
- Fix the dashboard status-error handler's obsolete notice-function call so
  a disconnected station reports its error and recovers without a script fault.
- Add interaction stress checks with 19 dashboard nodes, rapid navigation,
  failed/partial history reads, invalid cursors, timeouts and station restart.
  Extend host checks to both board power polarities, allocation failure,
  low-memory recovery, full PSRAM buffers and active-reader CRC faults.

These source corrections retain protocol V5, acquisition/reporting intervals,
SW2 recording, power limits and persistent data formats. This full release
includes all changes listed under 4.4.0 and the sensor fixes from earlier
releases. All three targets use 4.4.1 / 40401. Existing older packages remain
available. No board was flashed or measured during release preparation.

## 4.4.0 / 40400 — 2026-10-25

- Fix precision acquisition and recording replay stalls after long uptime by
  replacing inactive/expired signed retry deadlines with explicit unsigned
  elapsed backoffs. Failed durable recording acknowledgements also back off.
- Use 64-bit monotonic station uptime, sensor contact age and login blocking.
  A millis wrap cannot make an old contact fresh or revive an expired block.
- Pin touch-selected chart samples through pointer leave and status updates;
  retain the original readout when a point leaves the rolling window. Add
  adjacent-sample/clear actions and keep axis labels readable on small screens.
- Start the ThingSpeak upload task only for an enabled valid upload, and wake
  an idle worker through notifications instead of polling an empty queue.
- Add heap fragmentation diagnostics to the status API and document current
  V3/V4/station capacity and mobile interface research in
  [the resource review](RESOURCE_AND_UX_REVIEW.md).
- Make a compact current-readings overview the default. The whole sensor card
  opens details; every measurement tile opens its own history. Save visibility,
  order, column count and spacing on the station under a separate NVS key.
- Remove automatic orange security/cloud setup notices. Keep persistent errors
  visible and make settings, firmware and export explicit detail actions.
- Replace the normal 24-hour/900-point limit with bounded CRC-checked flash
  pages, fair per-node quotas and an adaptive filesystem reserve. Preserve
  TMP117/IMU float precision and separate manual recordings. Import legacy
  environmental history without changing sensor or Wi-Fi configuration.
- Load only the opened sensor's history in fixed pages. Offer longer periods
  and All retained data, reduce only SVG geometry, and keep raw cursor/CSV
  samples. Preserve expanded controls, focus and pinned values during updates.
- Add `station_s3_update` and `upload.py station --application-only` for USB
  application updates without rewriting bootloader or partitions. Normal
  uploads reject inherited full-erasure flags; explicit factory reset remains.

Protocol V5, NVS settings formats, partition layouts, acquisition schedules and
battery-protection policies are unchanged. PCB V3/V4 OTA packages retain the
existing installation signing identity. The station BIN is application-only at
`0x10000`. No device was flashed or measured during preparation.

## 4.3.5 / 40308 — 2026-10-17

Sensor PCB V4 OTA maintenance update for intermittent BME280 read failures.

- Require a valid BME280 forced-conversion sample before replenishing the
  bounded recovery budget. Initialization followed by a failed conversion now
  triggers the existing five-second deep-sleep retry path, including with
  automatic sensor detection. Persistent failures exhaust the three-attempt
  budget and return to the configured interval.
- Add optional Bosch probe and BME280 register/configuration/conversion
  diagnostics. Normal builds keep diagnostic logging disabled.
- Preserve BME280 power gating, BME680 maintenance, recording behavior and V4
  2800/2950 mV battery protection with 24-hour deep sleep.
- Regression tests cover failed conversions, retry exhaustion, valid-sample
  recovery, automatic detection and battery protection during recovery.

The package uses the existing installation signing identity and protocol V5.
The station and PCB V3 distribution remain at 4.3.4; no station update is
required for this sensor correction. This improves recovery latency and does
not establish the electrical cause of the reported intermittent fault.
No hardware was flashed or measured.

## 4.3.4 / 40307 — 2026-10-07

This inventory covers changes after commit `1f690bc` (firmware 4.3.1 / 40304)
and [merged PR #30](https://github.com/Ans1S/esp32-c3-iot-sensor-board/pull/30).
Its merge commit `f05381d` is the base for this release. The unpublished local
4.3.2/4.3.3 packages were intermediate checkpoints; 4.3.4 includes their fixes.

### Main improvements

- **More reliable sensor selection:** search both Bosch addresses for the
  explicitly selected type, release the previous bus/rail before replacement,
  and retry a slow module even after a positive initial mismatch. Three bounded
  startup attempts use five-second deep sleeps and prompt station reports.
  Success restores the configured cadence; V4 battery protection takes priority.
- **MAX30102 signal recovery:** independently adjust red/IR LED current, recover
  weak IR contact signals, permit pulse estimation with dim red, and retain more
  of slow small pulsations. Gain changes require new FIFO frames and a complete
  estimation window. No SpO2 or clinical accuracy claim; SW2 remains required.
- **Stable saved graphs:** wait for synchronization, load the complete selected
  recording before drawing, and retain the graph/cursor during status polling.
  Users can explicitly open an available partial snapshot or reload it. Full
  recording/minute navigation uses cached samples; CSV exports original data.
- **Durable configuration and storage:** acknowledge settings only after
  persistence succeeds, validate migration identities/complete reads, retain
  retryable cache state, preserve nonblank mount failures, and commit reset
  results before acknowledging commands.
- **Clearer station operation:** normal USB updates preserve settings/archives;
  only the factory-reset environment erases flash. Names and motion references
  survive removal/re-pairing. Pending sensor changes do not reuse old values or
  errors. BME280 omits IAQ/gas tiles; setup notices disappear after five seconds.
- **Safer publication:** remap compiler paths before building release binaries,
  withdraw older path-bearing binaries from the current tree, and check staged,
  pushed and historical blobs plus ZIP contents with redacted findings.

### Additional corrections

| Area | Changes |
| --- | --- |
| BME680 | Start periodic learning-state persistence even at accuracy zero; validate restored state/metadata, clear rejected readiness/retained IAQ, require fresh valid IAQ and stable raw heater/gas results, and avoid double clock-wrap adjustment. |
| LSM6DSOX | Reject stale FIFO backlog after long scheduling gaps, retain explicit gaps, validate maximum FIFO counts and reinitialize after bus faults. |
| Live acquisition | Cancel in-flight initialization/read safely on stop/profile changes; use an independent normal cadence and release the rail between normal windows. |
| Battery telemetry | Replace cached battery capability/failure flags together with voltage; keep V4 strict 2800/2950 mV hysteresis and 24-hour protection deep sleep. |
| Sensor settings | Check NVS migration version/identity, partial reads, durable revision/reset ordering and failed reset recovery; clear obsolete pairing snapshots after measurement changes. |
| Recording lifecycle | Freeze duration on first stop, make repeated stops idempotent, use the actual monotonic request time for late UTC anchoring, and match current request sequences for replay ACKs. |
| Recording archive | Serialize complete chunked JSON pages with real ArduinoJson, preserve unreadable/torn tails correctly, deduplicate durable appends and clamp finite extreme values before compact conversion. |
| Station registry | Validate telemetry semantics, preserve durable settings across failed/interleaved writes, remember MAC identities/references independently of pairing slots, and invalidate stale cloud jobs by generation. |
| Radio | Correlate configuration/recording responses with the active sequence and retain bounded recovery, existing V5 environmental lengths and trial-boot handling. |
| HTTP inputs | Require complete bounded decimal input before narrowing MAC/PCB/channel/field/interval/reset settings; reject non-finite and invalid ranges. |
| Browser forms | Preserve discovery drafts/focus; serialize discovery, save and export actions; keep immutable form snapshots and ignore delayed responses after reopening a form. |
| Browser recovery | Retry failed startup configuration, distinguish missing readings from zero, hide invalid IAQ advice, and show storage/connection failures. |
| Charts/export | Retain gaps and exact raw cursor/CSV values, optional display smoothing, readable sparse plots and XYZ overlays; cancel exports without retaining bandwidth reservations. |
| OTA | Use the actual `0x140000` sensor slot limit in manifest/package/browser validation; reject oversized signed images, rejected headers and file-selection races. Preserve the trusted installation key and rollback policy. |
| Release tooling | Supply V3/V4 4.3.4 signed packages and an application-only station image, verified payloads/identities/signatures and SHA-256 checksums; keep local keys/agent files/build outputs ignored. |
| Documentation | Refresh all six public READMEs, precision/recording/timing/power/architecture/OTA guides, reliability and MAX30102 investigations, and publication/change records. Local agent instructions and a project release skill remain unpublished. |
| Regression coverage | Expand to 22 production C++ suites, six real embedded-page Playwright suites, six cryptographic OTA tests, two upload-preservation tests, three library-patch checks and publication-guard regressions. |

### Compatibility and validation

Protocol V5 and the recording layout remain unchanged. Update the station for
dashboard/storage changes and the nodes for driver/recovery changes. Older
layouts still require the [one-time USB migration](RECORDINGS.md#one-time-usb-migration).
The station application is written at `0x10000`; it is not a sensor OTA package
or a factory image. See [current release files](releases/README.md).

The three PlatformIO targets and the documented host/browser/package checks
pass. No board was flashed. Physical accuracy, module replacement, RF/power-loss
behavior, sleep current, cutoff calibration and the reported BME280 low-voltage
hardware failure still require bench measurements. See
[reliability evidence](RELIABILITY_REVIEW.md), [MAX30102 research](MAX30102_REVIEW.md)
and [publication scope and historical limitations](PUBLICATION_REVIEW.md).

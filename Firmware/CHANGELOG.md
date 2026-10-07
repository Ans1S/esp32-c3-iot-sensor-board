# Firmware change inventory

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

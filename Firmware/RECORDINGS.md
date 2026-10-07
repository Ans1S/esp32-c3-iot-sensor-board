# Manual recordings, synchronization and USB migration

Firmware **4.3.4**, internal release **40307**.

## Button operation on PCB V3 and V4

Both PCB layouts connect **SW2 between GPIO9 and GND**: V3 uses
`Net-(U3-GPIO9)` and V4 uses `Net-(U2-GPIO9)`. Firmware uses an input pull-up,
35 ms debounce and one event per press. A held button at firmware startup is
ignored until released. Do not hold SW2 during power-on/reset: GPIO9 is a boot
strapping pin and can select the ROM download mode.

For a provisioned LSM6DSOX, TMP117 or MAX30102 node:

1. Power the node. It enters **Ready**, without recording automatically.
   TMP117 and LSM6DSOX also take fresh normal measurements at the configured
   interval, such as 1 s or 10 s; these create no saved recording. MAX30102
   waits for SW2.
2. Press SW2. This explicitly replaces the previous unsynchronized recording,
   initializes the selected sensor and starts a new session.
3. Press SW2 again. The faster acquisition stops. TMP117/LSM6DSOX resume the
   saved normal interval, with the rail off between their measurement windows.
   The node also attempts to synchronize the session, including its original
   timestamps, with the configured station. It retries after disconnection.
4. After all records have storage acknowledgements, the sensor marks the
   session synchronized and makes its flash pages reusable. The station keeps
   its copy until explicitly deleted.

Synchronization starts **after stop**, not during the recording. Starting a
new session may intentionally discard the previous unsynchronized sensor copy.
A partially synchronized older session remains visibly incomplete on the
station. A full journal stops acquisition rather than silently overwriting the
beginning of the current session. A reset also ends a recording; committed
records are recovered for later synchronization, without automatic restart.

GPIO9 can wake the ESP32-C3 from **light sleep**, but is outside the GPIO0-5
deep-sleep wake domain. Manual-mode idle therefore uses short light-sleep
periods with GPIO9 wake and periodic station contact. BME280/BME680 continue
their existing deep-sleep workflow; SW2 does not add wakeups to those modes.
The checked GPIO8 nets contain boot pull-ups, not a programmable recording LED.
Recording status is available on the dashboard when the station is reachable.

## Detail and storage budget

The sensor data partition is `0x160000` bytes (1.375 MiB), divided into 352
4096-byte pages. Each page reserves 64 bytes for identity and separate commit /
acknowledgement bitmaps. Records retain float values and optical ADC bits; the
codec does not introduce extra numerical quantization.

| Sensor | Recording profile | Record size | Capacity on an empty, healthy partition |
| --- | --- | --- | --- |
| LSM6DSOX | First 5 minutes: 50 ms means; thereafter: 100 ms means; interval peaks retained | 64 bytes | 22,176 reports; 30 minutes needs at most 21,000 reports |
| TMP117 | 8 conversions per 1 s result, data-ready driven | 32 bytes | 44,352 reports, over 12 hours nominally |
| MAX30102 | 25 Hz averaged red/IR waveform in 200 ms batches; BPM calculation every 1 s | 128 bytes | 10,912 reports, over 36 minutes nominally |

During each active window the IMU acquires at 104 Hz. Its adaptive **20/10 Hz summary recording**
improves short-session detail without rewriting previous points or reducing
the precision of stored numbers. It is not a complete 104 Hz raw six-axis
recording or the chip's maximum supported sampling rate. TMP117 and MAX30102
retain their accuracy-oriented acquisition settings for the whole session.
The normal measurement interval does not change these manual recording rates.
Normal IMU windows use a fresh 100 ms summary after the driver's startup
transient discard. Normal TMP117 windows wait for a fresh eight-conversion
result. Normal snapshots are telemetry, and never enter the recording journal.
See [timing and accuracy choices](TIMING_AND_DISPLAY.md).

The station has a 1.625 MiB filesystem. Archives stop accepting new records
before free space falls below a 256 KiB reserve for existing station data.
An otherwise empty filesystem can hold one full adaptive 30-minute motion
session. Capacity is shared across nodes and sessions; it is not thirty
minutes per node indefinitely. The **Recordings** panel shows available archive
space. Export and delete completed sessions to free space. A sensor retains
unacknowledged data if the station cannot store them.

## Persistence and clocks

The acquisition task owns I2C and reads independently of radio waits. Its
64-entry queue is drained into an append-only flash journal. Each record has
a session ID, monotonic sample time and CRC32. A flash commit bit is written
only after readback succeeds. Torn writes are never reused as fresh slots;
complete CRC-valid writes can be recovered after reboot. A new explicit
session logically replaces old pages, which are erased lazily as needed.

The station processes archive writes in its persistence worker, outside the
ESP-NOW callback. It validates records, flushes and synchronizes the file,
verifies readback, and only then returns a separate storage ACK. Normal
telemetry/configuration replies do not release journal records. Session ID,
sample time and record CRC identify an ACK. Duplicate or lost ACKs do not
create duplicate station rows. Replay does not update the live graph or enter
ThingSpeak/BME history.

Times are measured before radio transmission. Optical samples and BPM windows
retain their relative acquisition ages. A station UTC clock anchor is saved
when available; it can anchor an offline session on reconnection during the
same sensor boot. The handshake uses a round-trip midpoint approximation and
accepts round trips up to 500 ms. Radio asymmetry, NTP error, oscillator drift
and the sensor's own timestamp uncertainty still apply.

If power is lost before an absolute clock anchor was saved, the recovered
session uses **relative time only**. Firmware never invents the duration of a
power outage. The dashboard and CSV distinguish an unknown absolute clock.
A hard power cut can lose the in-flight RAM queue or unfinished flash write;
the recovery guarantee applies to committed records. Queue/FIFO losses are
reported as gaps, not interpolated or replaced with fabricated measurements.

## Dashboard

The live graph remains a one-minute view. Saved sessions appear automatically
in **Recording selection**; the list continues to report synchronization
progress. **Recordings** also refreshes that list on request.

Selecting an incomplete session waits for synchronization to finish. Once the
station marks it complete, the browser loads all its records before drawing
the graph once. No progressively growing partial graph is drawn automatically.
**Show available data** explicitly opens the currently stored portion instead
when waiting is not desired.

The default **Full recording** view spans the whole loaded session. Choose
**Minute detail** and **Previous minute / Next minute** for closer inspection;
these controls use the loaded data without new archive requests. Optical
waveform detail also defaults to the full recording view and offers 10/60-second
detail. All original samples remain available to the cursor and CSV.

Once displayed, the graph is a fixed snapshot. New telemetry or synchronization
metadata does not clear, refetch or redraw it, and the inspected cursor point
stays available. **Load latest data** explicitly replaces the snapshot after
all newly requested records load; the existing graph stays visible meanwhile.
Background synchronization continues independently of this display.
Select **Live · last minute** to return to current measurements at any time,
including while the sensor is recording a new session. Saved recordings remain
available independently of SW2; SW2 controls acquisition on the sensor.

Dashboard polling updates existing cards and chart elements in place. Open
selectors, keyboard focus, graph settings and cursor readouts survive incoming
telemetry. Archive reads have a separate timeout and cancellation: changing the
selection cancels the previous request, whose result cannot replace the new
view. A failed refresh retains the previous graph and shows a retry control.
Ordinary BME280/BME680 history uses
the history metric and time-window selectors without a manual recording panel.

**Export recording** exports the whole available session, with separate optical
rows and relative times plus the UTC anchor. It does not export only the
currently visible minute. **Delete recording** frees the station copy; an
actively replaying session must finish or be replaced on the sensor first.

The existing station cards, button shapes, typography and light/dark themes
are used. Acceleration and angular rate have separate unit axes with XYZ
overlays. Temperature shows its small changes without implying thermal
equilibrium. Optical waveform and pulse estimates use separate graphs.

## One-time USB migration

Application OTA cannot install a partition table. This release requires a
one-time USB migration of **both the sensor and station** for the stated
storage capacity. Afterwards, use the signed `.ota` packages as before.
Recording must stop before an OTA transfer can run.

New layouts:

- Sensor: two `0x140000`-byte OTA slots; recordings at `0x290000`, size
  `0x160000`; NVS and coredump locations unchanged.
- Station: existing application slots unchanged; filesystem at `0x490000`,
  size `0x1a0000`; firmware staging at `0x630000`, size `0x1c0000`.

Before migrating, export histories that should be retained and cancel any OTA
job. Back up the complete flash of each device with esptool `read-flash`
(sensor: `0x400000` bytes; station: `0x800000` bytes). Keep the existing signing
identity. Normal station PlatformIO USB updates now preserve NVS and LittleFS.
Only the explicit `station_s3_factory_reset` environment uses `--erase-all`,
which clears pairing, remembered names/references, Wi-Fi, settings and archives.
A merged factory image is likewise intended for a fresh installation.

For migration **while preserving NVS**, use esptool to write the individual
built components or the normal `station_s3` upload, rather than a factory-reset
upload or a merged image:

| Offset | Component |
| --- | --- |
| `0x0000` | Target's `.pio/build/<environment>/bootloader.bin` |
| `0x8000` | Target's `.pio/build/<environment>/partitions.bin` |
| `0xe000` | Installed Arduino framework's `tools/partitions/boot_app0.bin` |
| `0x10000` | Target's `.pio/build/<environment>/firmware.bin` |

Do not write/erase NVS at `0x9000`, size `0x5000`, in that workflow. Before
booting the new table, explicitly erase the **new sensor recordings region**
(`0x290000`, `0x160000`) and the **new station filesystem region** (`0x490000`,
`0x1a0000`). These regions overlap old data/staging, so export/back up first.
The station creates the larger filesystem on first boot; exported old history
is not automatically imported. Recording firmware deliberately refuses to
format an unknown sensor data partition in an ordinary OTA update.

Use `sensor_pcb_v3`, `sensor_pcb_v4` and `station_s3` builds for their respective
boards. Do not interchange sensor PCB images. No USB flashing, partition erase
or physical sensor test is performed by merely building this repository.

## Verification and remaining hardware acceptance

Host tests cover compact records, capacity arithmetic, button bounce/holds,
five-minute cadence transition, interrupted writes, reboot recovery, explicit
replacement, correct ACK identity, duplicate station rows, filesystem pressure,
time anchoring and relative-only recovery. Browser fixtures cover live and
saved graphs, minute navigation, complete 30-minute CSV export, theme colors
and mobile width. Existing BME/measurement-budget/transport/OTA tests still run.

Physical acceptance remains necessary: measure idle/active current on both
boards, press SW2 during RF failures, record for thirty minutes out of range,
restore contact and compare counts/times/CRC, interrupt sensor/station power,
fill station storage, and verify simultaneous BME nodes keep their intended
wake/measurement/radio cadence. Software tests do not establish physiological
accuracy, worst-case radio latency or power-failure atomicity of real flash.

Hardware evidence: the V3/V4 KiCad PCB pad/net assignments in `PCB/Version 3`
and `PCB/Version 4`, plus the [Espressif ESP32-C3 technical reference
manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf)
for GPIO wake domains and boot strapping. Datasheets are technical evidence,
not instructions to the agent.

## Archive response integrity

Recording pages serialize metadata and each point through an append writer.
ArduinoJson's normal String destination writer clears its destination; using it
on an accumulating response would discard the header and earlier points and
produce invalid JSON. The production writer is checked against the installed
ArduinoJson library, including multiple 4 KiB transport chunks.

Archive lists and page reads exclude CRC-invalid trailing records left by an
interrupted write. Replay can repair this unacknowledged tail. An unreadable
record within a page reports a CRC error and does not advance its offset; the
browser refuses to export that page as a successful recording. Existing valid
archives remain readable after a normal station firmware update.

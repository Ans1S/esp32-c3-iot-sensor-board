# Live precision sensors (4.3.1)

The same PCB V3/V4 sensor image includes BME280, BME680, LSM6DSOX, TMP117 and
MAX30102 drivers. One I2C sensor type is active per node. The three live types
form a dashboard category; this is not simultaneous acquisition of three
modules on one node. Explicit selection takes priority; automatic detection
retains Bosch-first behavior, followed by LSM6DSOX, TMP117 and MAX30102.

| Sensor | Acquisition | Normal measurement interval | SW2 recording cadence | Display |
| --- | --- | --- | --- | --- |
| LSM6DSOX | 104 Hz FIFO, +/-4 g, +/-500 degrees/s | Configured interval, for example 1 s or 10 s; fresh 100 ms summary after startup settling | 50 ms for 5 min, then 100 ms | XYZ interval means on two separate charts, plus peaks |
| TMP117 | 8 conversions averaged, nominal 124 ms active time | Configured interval, for example 1 s or 10 s; wait for fresh data-ready | Continuous 1 s cycle | Temperature, three decimal places |
| MAX30102 | Red and infrared, 100 Hz, 4-sample FIFO average, 18-bit ADC | SW2 only | 200 ms waveform packets; 1 s BPM | Pulse trend, optical waveform and signal status |

TMP117 and LSM6DSOX take normal measurements immediately after startup, then
at the configured measurement interval. Each normal window initializes the
sensor, waits for fresh data and turns the external rail off afterward. These
snapshots create no recording. Sparse IMU snapshots do not establish continuous
step counts or active time. Five-second station heartbeats retain the measured
value with its increasing age; they do not create fresh measurement points.

A debounced SW2 press starts the faster recording profile; the next press stops
it. The normal interval remains saved and cannot slow down the manual session.
After stopping, normal measurements resume while the node replays its flash
journal to the station. The acquisition task owns I2C independently of radio
waits. Each recording is initialized afresh. A new start
explicitly replaces a previous unsynchronized session. Closing the browser
neither starts nor stops a recording.

The dashboard retains the last minute in RAM (1201 IMU, 301 optical, 61 TMP117
reports), using the existing cards, controls, colors and light/dark themes.
Completed recordings can be selected, viewed in one-minute windows, exported
and deleted on the station. Live/recorded precision data never enter ThingSpeak.
BME280/BME680 retain their existing forced/BSEC and deep-sleep policies.

Both targets require a one-time USB partition migration for the new storage
capacity, followed by normal signed OTA updates. See [manual recording and USB
migration](RECORDINGS.md) for the button wiring, capacity, replacement policy,
clock uncertainty and backup requirements.

## Assignment changes and the motion reference

A changed sensor type, interval or temperature offset is applied after the next
successful station exchange. The sleep path wakes again after one second to
initialize the new selection. A running precision driver restarts for a type
or offset change, including an explicit type changed back to auto detection.
Until a sleeping node next contacts the station, the dashboard shows the chosen
type, the last detected type and pending configuration without showing old
sensor values as new readings. Live chart caches reset when the type changes.

Initialization releases the old I2C bus and sensor rail before changing an
active driver. If the requested chip does not respond, including when another
type responds first, one complete power cycle repeats detection with a 200 ms
settling delay. A failed startup gets at most three attempts across boots,
separated by five-second deep sleeps. Recovery wakes also report to the station
before the normal reporting deadline. Success immediately restores the normal
interval; exhausted attempts stop short recovery wakes until a later successful
startup or a changed measurement configuration. Cold boots also allow this
bounded recovery. V4 battery protection overrides every retry.

Pending type changes display an initialization hint with empty new-sensor
values. They do not inherit error flags or recording state from the previous
driver. A mismatch reported after applying the selection remains an error.
Explicit selection still requires the selected chip; it does not silently
replace it with a different sensor. Select **Auto detect** when the physical
module type should be detected automatically.

On LSM6DSOX nodes, **Zero axes here** saves the current resting XYZ acceleration
and XYZ angular-rate means as a station display reference. Put the sensor still
on the table and wait for a recent measurement first. The station rejects stale
or failed data, FIFO gaps, non-finite axes, gravity magnitude outside 0.8-1.2 g
and angular rates above 5 degrees/s on any axis. This is a convenient relative
reference, not a factory calibration or an estimate of orientation angles.

The six value tiles, live/saved plots and cursor readouts subtract this reference.
Peak magnitudes, step feedback, the sensor journal and CSV retain original data.
**Clear reference** restores the original axis display. The reference is saved
with the MAC address, survives station reboot/re-pairing and does not require
reflashing the node. BME sensors continue using deep sleep and have no manual
button measurement mode.

## TMP117 accuracy choices

- Probe device ID `0x117` (revision bits ignored) at `0x48` through `0x4B`.
- Software reset, then verify configuration `0x0220`: 8-sample averaging,
  nominal 124 ms active conversion and 876 ms standby per second. This matches
  TI accuracy test conditions and reduces self-heating. More averages do not
  automatically improve the assembled thermometer's absolute accuracy.
- Preserve factory calibration and EEPROM, including a pre-existing hardware
  offset. The BME680 software temperature compensation is not applied.
- Only accept data-ready measurements; decode signed 16-bit data at
  0.0078125 degrees C/LSB. Expire measurements after 1.5 s without a fresh result.
  Normal measurements wait for the first complete eight-conversion result
  after each startup. A long normal interval does not reuse an old result.
- The device has no reference-free self-calibration procedure. Extra decimal
  places represent output resolution, not demonstrated system accuracy.

The chip measures its own temperature. Thermal coupling, placement, ambient
conditions, supply and heat from the ESP32 determine how well it follows the
measurement target. Allow contact to equilibrate and compare the assembled
probe against a reference before interpreting readings as body temperature.

## MAX30102 signal acquisition

- The fixed **7-bit I2C address is `0x57`**. Arduino Wire requires this address;
  datasheet write/read bytes `0xAE`/`0xAF` already include the R/W bit and must
  not be passed as Wire addresses. Read PART_ID at register `0xFF`, value `0x15`.
  The firmware probes this ID, verifies reset completion and configuration,
  and uses a 100 kHz bus for the MAX30102, including auto detection.
  Confirm the actual module marking: a shared part ID is not proof that a
  third-party breakout contains exactly the advertised device.
- Use red/IR mode with a 4096 nA ADC range and four-conversion FIFO averaging to obtain
  a filtered 25 Hz signal. Drain FIFO continuously between radio reports.
- Start both LEDs at register `0x24` (nominal 7.2 mA). Adjust current in bounded
  proportional steps of at most eight register counts per second, separately
  for red and IR, to avoid conflicting gain changes between the optical paths.
  Weak reflections above 2000 ADC counts can raise gain even before reaching
  the 10000-count IR analysis threshold; dark signals do not raise LED current.
  Current is bounded to `0x04`–`0x60` (nominal 0.8–19.2 mA per LED).
  Every adjustment, IR contact loss
  or FIFO overflow discards the pulse-analysis window.
- Press SW2 to start measurement and recording; press again to stop. MAX30102
  does not collect pulse data while idle. After gain settling, require eight
  seconds of uninterrupted usable IR samples. A dim red channel does not veto
  heart rate: the estimator uses IR only and does not calculate SpO2.
  Detrend with a centered 25-sample / 1-second mean,
  search normalized autocorrelation peaks with sub-sample interpolation over 30-200 bpm and reject weak or
  implausibly large modulation. Correlation is a signal metric, not a probability
  of clinical accuracy. Movement can still produce false periodic signals.
- Display no contact, settling/poor signal, periodic signal or recording gap.
  Withhold BPM when the checks fail; never replace it with a plausible number.
  The device does not perform automatic physiological calibration. No calibrated
  SpO2 value is produced by this implementation.
- Ambient-light overflow, FIFO full/overflow, brownout and long polling gaps
  invalidate the pulse window. Brownout requires complete reinitialization.

See [the targeted MAX30102 investigation](MAX30102_REVIEW.md) for reproduced
missing-value cases, corrections, operating checks and physical limits.

Use the module's specified power input and I2C voltage levels. The bare MAX30102
requires separate 1.8 V core and LED supply domains; do not assume that an
unregulated module can be connected directly to a 3.3 V sensor rail.

For an unreachable module, check an ACK at `0x57` and then `0xFF -> 0x15` with
SDA/SCL referenced to a common ground. The bare IC requires a nominal 1.8 V VDD
and separate nominal 3.3 V VLED+; a breakout's input voltage depends on its
regulators and level shifting. Do not infer its input voltage from the IC's
rail names. No address jumper selects another MAX30102 address. Manufacturer
references: [MAX30102 datasheet, register map, supply rails and Table 17](https://www.analog.com/media/en/technical-documentation/data-sheets/max30102.pdf).


## Installation and verification

Install the station and matching V3/V4 sensor base images using the migration
procedure in [RECORDINGS.md](RECORDINGS.md). The current internal release is
40304. Subsequent application updates use the matching signed `.ota` package.
The station still accepts older V5 environmental/motion/precision lengths;
recordings use separate message types and storage acknowledgements.

Host checks cover register configuration, temperature sign/resolution, stale
data, an artificial 120 bpm waveform, pulse warmup, contact loss, I2C failures,
FIFO overflow, fresh-versus-cached estimates, transport lengths and a full
minute of RAM samples, durable recording recovery and archive deduplication.
The production acquisition-loop tests also cover normal 1/10 s schedules,
rail shutdown, recording independence and resumption after a recording.
Browser checks exercise the embedded dashboard and mobile layout. These checks
do not establish measurement accuracy on hardware.

Before relying on the new modules, verify I2C wiring and supply, compare TMP117
with a reference at stable temperatures, compare MAX30102 with a pulse reference
at rest and during movement, and check acquisition under simultaneous Wi-Fi/OTA
load. LSM6DSOX reset, configuration readback and startup-transient rejection
remain active; no stationary bias subtraction is attempted without a known
stationary pose. Step counting is not implemented.

Sources: supplied TI TMP117 datasheet SNOSD82D, sections 7.4 and 7.6
(configuration, data-ready, conversion times and calibration); supplied Maxim
MAX30102 datasheet, register map, FIFO, mode, SpO2 configuration and LED-current
sections. Document content was used as technical evidence, not as instructions.

For calculations, timestamp uncertainty and chart choices for all five sensor
types, see [Timing and display](TIMING_AND_DISPLAY.md).

The [2026-09-26 review](SENSOR_REVIEW.md) records the manufacturer references,
fault-handling corrections, expanded tests and V4's default 2.8 V protection.
Low battery stops live acquisition and permits only daily battery reporting.

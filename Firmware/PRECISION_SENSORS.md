# Live precision sensors (4.3.1)

The same PCB V3/V4 sensor image includes BME280, BME680, LSM6DSOX, TMP117 and
MAX30102 drivers. One I2C sensor type is active per node. The three live types
form a dashboard category; this is not simultaneous acquisition of three
modules on one node. Explicit selection takes priority; automatic detection
retains Bosch-first behavior, followed by LSM6DSOX, TMP117 and MAX30102.

| Sensor | Acquisition | Radio target cadence | Display |
| --- | --- | --- | --- |
| LSM6DSOX | 104 Hz FIFO, +/-4 g, +/-500 degrees/s | 50 ms for 5 min, then 100 ms | XYZ interval means on two separate charts, plus peaks |
| TMP117 | Continuous 1 s cycle, 8 conversions averaged | 1 s | Temperature, three decimal places |
| MAX30102 | Red and infrared, 100 Hz, 4-sample FIFO average, 18-bit ADC | 200 ms waveform packets; 1 s BPM | Pulse trend, optical waveform and signal status |

Live acquisition starts only after a debounced SW2 press and stops at the next
press. Each session is initialized afresh for accuracy. Its acquisition task
owns I2C independently of radio waits. After stopping, the sensor powers down
the external module and replays its flash journal to the station. A new start
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
- The device has no reference-free self-calibration procedure. Extra decimal
  places represent output resolution, not demonstrated system accuracy.

The chip measures its own temperature. Thermal coupling, placement, ambient
conditions, supply and heat from the ESP32 determine how well it follows the
measurement target. Allow contact to equilibrate and compare the assembled
probe against a reference before interpreting readings as body temperature.

## MAX30102 signal acquisition

- Probe `0x57`, part ID `0x15`; verify reset completion and configuration.
  Confirm the actual module marking: a shared part ID is not proof that a
  third-party breakout contains exactly the advertised device.
- Use red/IR mode with a 4096 nA ADC range and four-conversion FIFO averaging to obtain
  a filtered 25 Hz signal. Drain FIFO continuously between radio reports.
- Start both LEDs at register `0x24` (nominal 7.2 mA). Adjust current in bounded
  steps to avoid weak/saturated optical levels. Every adjustment, contact loss
  or FIFO overflow discards the pulse-analysis window.
- Require eight seconds of uninterrupted usable samples. Detrend the IR signal,
  search normalized autocorrelation peaks with sub-sample interpolation over 30-200 bpm and reject weak or
  implausibly large modulation. Correlation is a signal metric, not a probability
  of clinical accuracy. Movement can still produce false periodic signals.
- Display no contact, settling/poor signal, periodic signal or recording gap.
  Withhold BPM when the checks fail; never replace it with a plausible number.
  The device does not perform automatic physiological calibration. No calibrated
  SpO2 value is produced by this implementation.
- Ambient-light overflow, FIFO full/overflow, brownout and long polling gaps
  invalidate the pulse window. Brownout requires complete reinitialization.

Use the module's specified power input and I2C voltage levels. The bare MAX30102
requires separate 1.8 V core and LED supply domains; do not assume that an
unregulated module can be connected directly to a 3.3 V sensor rail.

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

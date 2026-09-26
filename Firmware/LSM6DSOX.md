# LSM6DSOX support in firmware 4.3.1

PCB V3 and V4 use the same I2C driver, with their existing, different sensor
power-gate polarities. The station and both sensor images identify as 4.3.1
(release integer 40304).

## Connect and configure

Use the board's sensor supply connector, ground, SDA (ESP32 GPIO5) and SCL
(GPIO4). The bus uses 3.3 V logic and 400 kHz. On the breakout, CS must be high
for I2C and SA0 must select a defined address: low for 0x6A, high for 0x6B.
Check the actual breakout pin labels, pull-ups and supply circuitry; the chip
datasheet alone cannot identify the wiring of an AliExpress module. Do not
apply 5 V directly to the chip or I2C signals. Interrupt pins are not required.

Upgrade the station first, then install the matching sensor V3/V4 image. In
the station, select LSM6DSOX or Auto detect. Provisioned IMUs
use manual SW2 sessions, with 50 ms reports for the first five minutes and 100 ms thereafter. Auto detection tries Bosch devices first if multiple
supported devices share the bus; explicitly select LSM6DSOX in that case.
This firmware handles one selected external sensor per node.

The driver checks WHO_AM_I (0x6C), resets with a 100 ms timeout, enables block
data update and register increment, disables unused I3C and reads back the
configuration. Both sensing chains run at 104 Hz in high-performance mode:

| Setting | Value |
| --- | --- |
| Accelerometer | +/-4 g; 0.122 mg/LSB; CTRL1_XL = 0x48 |
| Gyroscope | +/-500 degrees/s; 17.5 mdps/LSB; CTRL2_G = 0x44 |
| FIFO | Uncompressed, both chains at 104 Hz, continuous overwrite mode |
| Startup | Discard first 100 ms before FIFO collection |
| Filtering | Default first-stage filtering; no high-pass removal of gravity |
| Acquisition | Drain FIFO about every 5 ms outside blocking radio/ADC/OTA work |

During a manual recording, IMUs remain powered and the MCU stays awake between reports.
Battery voltage is checked every ten seconds on V4 with protection enabled,
and once per minute on V3. V4 stops acquisition below 2.8 V; see the
[sensor review](SENSOR_REVIEW.md). FIFO overruns discard the incomplete window.
Environmental sensors retain their existing sleep behavior, including the
BME680 five-minute BSEC requirement. Continuous motion consumes substantially
more board energy than the environmental deep-sleep mode.

## Timing and recording

Sensor reports target **50 ms for five minutes, then 100 ms**, and the visible dashboard polls every 100 ms.
The chart displays a fixed one-minute live window.

Each report carries the mean acceleration and angular-rate XYZ values over
the report interval and
the maximum vector magnitude for each chain since the preceding report.
Acceleration includes gravity: a stationary board should measure a magnitude
near 1 g, not zero. These are sensor-frame measurements, not position, yaw or
a fused orientation estimate. No automatic zero-bias calibration is applied
while the board might be moving. Range and filter settings are a general
motion starting point, not a universal optimum for impacts or vibration.

**Reports and CSV contain interval summaries, not all 104 Hz raw samples.**
Short peaks sampled by the chip can survive a quieter latest sample. Waveform,
vibration-spectrum and precise event-timing work needs a separate raw-data
stream and appropriate sample rate/filter design. Live radio delivery is best effort, but the manual recording is journaled
locally and replayed after stopping. A dedicated task owns acquisition, so
normal radio waits and ADC settling do not determine sampling cadence.
FIFO/queue loss remains explicitly flagged. I2C errors or stale/missing axes
produce failed measurements, not fresh zeros. OTA waits for recording to stop.

The dashboard refreshes active IMU graphs every 100 ms and retains a one-minute
window. XYZ axes share an acceleration or angular-rate plot; the two physical
units remain separate. Saved sessions can be selected and paged in one-minute
windows. Fast live updates transfer only the recent tail after the initial load.

Live history uses a bounded 1201-slot RAM ring for the last minute. Manual
recordings are persisted on the sensor and synchronized after stopping; saved
station sessions are separate from the live ring. See [recordings](RECORDINGS.md)
and [timing](TIMING_AND_DISPLAY.md) for capacity, clocks and validation.

Session steps, active time and cadence are described in [quick feedback](SENSOR_FEEDBACK.md).

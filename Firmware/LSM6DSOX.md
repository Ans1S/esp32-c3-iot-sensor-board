# LSM6DSOX support in firmware 4.1.3

PCB V3 and V4 use the same I2C driver, with their existing, different sensor
power-gate polarities. The station and both sensor images identify as 4.1.3
(release integer 40103).

## Connect and configure

Use the board's sensor supply connector, ground, SDA (ESP32 GPIO5) and SCL
(GPIO4). The bus uses 3.3 V logic and 400 kHz. On the breakout, CS must be high
for I2C and SA0 must select a defined address: low for 0x6A, high for 0x6B.
Check the actual breakout pin labels, pull-ups and supply circuitry; the chip
datasheet alone cannot identify the wiring of an AliExpress module. Do not
apply 5 V directly to the chip or I2C signals. Interrupt pins are not required.

Upgrade the station first, then install the matching sensor V3/V4 image. In
the station, select LSM6DSOX or Auto detect. Newly provisioned, detected IMUs
default to a one-second report interval. An existing sensor retains its saved
interval until edited. Auto detection tries Bosch devices first if multiple
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

Provisioned IMUs remain powered and the MCU stays awake between reports, even
for longer reporting intervals. Battery voltage is refreshed once per minute.
Environmental sensors retain their existing sleep behavior, including the
BME680 five-minute BSEC requirement. Continuous motion consumes substantially
more board energy than the environmental deep-sleep mode.

## Timing and recording

Start with **one-second sensor reports and one-second dashboard refresh**.
This is suitable for observing position changes and activity. For a fluid
gesture display, 100-200 ms reports would be useful, but are not implemented
in 4.1.3: the configuration and station history use integer seconds. Intervals
of 2-5 seconds suit slower monitoring and reduce radio traffic. They do not
put the continuously sampling IMU to sleep.

Each report carries the latest acceleration and angular-rate XYZ values and
the maximum vector magnitude for each chain since the preceding report.
Acceleration includes gravity: a stationary board should measure a magnitude
near 1 g, not zero. These are sensor-frame measurements, not position, yaw or
a fused orientation estimate. No automatic zero-bias calibration is applied
while the board might be moving. Range and filter settings are a general
motion starting point, not a universal optimum for impacts or vibration.

**Reports and CSV contain interval summaries, not all 104 Hz raw samples.**
Short peaks sampled by the chip can survive a quieter latest sample. Waveform,
vibration-spectrum and precise event-timing work needs a separate raw-data
stream and appropriate sample rate/filter design. Radio reports are best
effort; an undelivered interval is not replayed. FIFO buffering bridges normal
radio and ADC pauses, but a long pause (especially OTA) can overflow. The
dashboard and CSV expose overflow instead of claiming an uninterrupted record.
I2C errors or stale/missing axes produce failed measurements, not fresh zeros.

The dashboard offers one-second minimum reports, refresh choices of 1/2/5/10
seconds and chart windows of 30 seconds, 1/5/15 minutes, 1 hour or 24 hours.
Axis values and peak magnitudes can be graphed separately. Fast-history
updates fetch only the recent tail after the initial load.

For report intervals below 60 seconds, the station keeps a bounded 900-slot
RAM ring: about 15 minutes at one second. It is lost on station restart and
is not written to flash per report. CSV exports the currently loaded retained
history; it does not start a permanent recording. At 60 seconds and above,
the existing persistent 24-hour history remains available. Epoch timestamps
require a synchronized station clock. Actual update latency includes radio,
HTTP polling and rendering delays, so one second is a target cadence, not a
hard real-time guarantee. Existing ThingSpeak rate limits remain in force;
motion axes currently have no cloud field mapping (battery upload is available).

## Compatibility and verification

The V5 configuration and OTA packets are unchanged. Telemetry adds a motion
suffix; 4.1.0 stations accept both the 4.0.0 prefix and extended packets. Old
stations cannot receive extended telemetry: **update the station first**.
Persistent V3 history is migrated to V4, retaining environmental samples;
older station firmware cannot read the new history format. Configuration and
pairing schemas are unchanged. Latest-value cache from an older image can be
replaced by the next report; history migration is separate.

Automated checks cover driver identity/reset faults, signed conversion, FIFO
tags, peaks, stale reads and overflow; bounded RAM history, no per-report fast
flash writes and V3 history migration; V5 prefix CRC compatibility; existing
OTA, radio, timing, ADC, BME680 and upload regression suites. Browser checks
exercise seconds inputs, motion tiles, chart windows, CSV and mobile layout.
All three target builds must succeed before packaging.

Hardware acceptance remains necessary on the actual breakout and both PCBs:

1. Confirm both address straps and chip detection; verify supply and I2C levels.
2. Rotate through six static faces: the corresponding axis should approach
   +/-1 g, the other axes near zero, and stationary angular rates near zero.
3. Rotate and briefly shake: verify axis signs, response and retained peaks.
4. Observe a one-second run for at least 20 minutes, including ring wrap, CSV,
   browser closure/reopening, station restart and a Wi-Fi interruption.
5. Disconnect/reconnect I2C and confirm error reporting and recovery. Test
   OTA interruption/overflow and power consumption before battery deployment.

Source: STMicroelectronics, [LSM6DSOX datasheet DS12814 Rev 4, June 2024](https://www.st.com/resource/en/datasheet/lsm6dsox.pdf),
especially sensitivity table, I2C interface, FIFO sections and register tables
36-39, 49-57, 73-74, 118-121 and 194-196. The supplied `DS_lsm6dsox.pdf` is the
same revision used for register verification.

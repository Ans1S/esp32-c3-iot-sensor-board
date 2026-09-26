# Sensor accuracy and PCB V4 low-voltage review

Reviewed on 2026-09-26 against the working firmware, manufacturer documents,
and the V4 production BOM/netlist. This is a software and design review, not a
measurement of assembled-board accuracy. Existing unfinished recording and
dashboard work was preserved. Release 4.3.1 packages were rebuilt for the pull
request after this review; no board was flashed.

## Results for the three live sensors

| Sensor | Checked operating point | Result and correction |
| --- | --- | --- |
| TMP117 | ID `0x117` with revision masked; addresses `0x48`-`0x4B`; reset followed by `CONFIG=0x0220`; signed 16-bit temperature / 128 | Conversion, eight-sample averaging, 1 Hz cycle and data-ready handling agree with TI. EEPROM and offset registers are not written. Failed transfers clear the pending freshness flag. Tests cover all addresses, negative temperatures, resolution, range endpoints and the invalid startup value. |
| MAX30102 | Part ID `0x15`; red/IR mode `0x03`; `SPO2_CONFIG=0x27`; 100 conversions/s averaged by four into a 25 Hz FIFO | Acquisition and scaling agree with ADI. Added ambient-light overflow and brownout detection, exact-full FIFO detection and long-poll-gap rejection. Pointer/current changes stop conversions temporarily. I2C faults, including partial FIFO reads, require reinitialization. Any discontinuity restarts pulse estimation. The peak search now includes the interpolation neighbor needed near the upper rate boundary. |
| LSM6DSOX | WHO_AM_I `0x6C`; `CTRL1_XL=0x48`, `CTRL2_G=0x44`, `CTRL3_C=0x44`, `CTRL9_XL=0xE2`; FIFO batching `0x44`, continuous mode `0x06` | 104 Hz, +/-4 g, +/-500 degrees/s, BDU, increment and scales agree with ST. Both current and latched overflow flags are now checked. An incomplete interval is discarded and the FIFO restarted; I2C faults require reinitialization. |

The live task also marks sensor initialization as active work. A stop request
must finish releasing the I2C bus and sensor rail before the battery-protection
path can proceed. A failed start now powers the module off during its retry delay.

For temperature, 0.0078125 C is resolution, not proof of system accuracy. The
sensor measures its own die; placement, thermal contact and ESP32 heat remain
part of the error budget. For pulse, a periodic optical signal is not proof of
an accurate physiological rate. Movement, pressure, ambient light and harmonics
can fool autocorrelation. The tests use nine artificial rates from 35 to 195 bpm;
they do not validate people, SpO2 or the entire nominal 30-200 bpm range. For
motion, the output is an interval mean plus peaks in sensor coordinates, including
gravity. It is not calibrated orientation or a raw 104 Hz waveform. No guessed
zero-bias or body-temperature correction was introduced.

Sources: [TI TMP117 SNOSD82D, sections 6.5, 7.4 and 7.6](https://www.ti.com/lit/ds/symlink/tmp117.pdf),
[ADI MAX30102 Rev 1, register map and FIFO/mode sections](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX30102.pdf),
[ST LSM6DSOX DS12814 Rev 4, sections 4 and 9](https://www.st.com/resource/en/datasheet/dm00557899.pdf).
ST's [register driver](https://github.com/STMicroelectronics/lsm6dsox-pid)
was also checked for signed scale factors and the latched overflow bit.

## BME280 at a 3.1-3.2 V battery voltage

There is no demonstrated 3.3 V battery cutoff in the sensor interface. The V4
netlist routes the battery through Q1 to `V_LDO`, which feeds the TPS63900 (U6).
U6 supplies `+3.3V`, and Q5 switches that rail to J1 pin 1. Its gate is active
low on GPIO10. The sensor connector is not wired directly to battery voltage.

R5 is 36.5 kOhm from CFG1 to ground; CFG2 is grounded and SEL is tied to VIN.
This selects 3.3 V with the unlimited input-current setting, rather than a
small programmable input-current limit. The regulator's specified input range
includes 3.1-3.2 V. The bare BME280 itself supports VDD down to 1.71 V.
These facts rule out a nominal 3.3 V battery minimum as the explanation; they
do not prove that the assembled board maintains its rail under load.

The actual external BME280 module is unidentified. A regulator or level shifter
on that module, wiring, solder joints, battery impedance, regulator transients,
or insufficient effective capacitance remain unverified possibilities.

Sources: V4 [BOM](../PCB/Version%204/ESP32-C3-V4/production/bom.csv) and
[netlist](../PCB/Version%204/ESP32-C3-V4/production/netlist.ipc),
[TI TPS63900, sections 6.3 and 7.3.6, Table 7-2](https://www.ti.com/lit/ds/symlink/tps63900.pdf),
[Bosch BME280 BST-BME280-DS002](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf).

Confirmed software weaknesses were corrected:

- The prior Adafruit wrapper ignored I2C transfer failures in register and
  calibration reads. Its forced-conversion poll could also see idle before
  conversion startup. A checked register driver now validates every transfer,
  reads calibration blocks twice, verifies configuration and waits 10 ms before
  checking completion. T/P/H come from one burst and share Bosch compensation.
  Humidity's signed 12-bit trim fields are decoded explicitly. Timeouts remain
  bounded by the overall measurement budget. Initialization stays asleep.
- Bosch/automatic startup now uses 100 kHz. V4's 5.1 kOhm pull-ups permit only
  about 69 pF at the 400 kHz/300 ns rise-time limit, ignoring parallel pull-ups.
  That is a plausible cable/module sensitivity, not a measured violation.
  Live recording sessions still use 400 kHz with their explicit sensor type.
- I2C pads are released without internal pull-ups before removing sensor power.
  GPIO10's inactive latch is set with the ESP-IDF GPIO API before output enable,
  avoiding an active-low power pulse on V4. The existing bounded startup power
  cycle remains available.
- A missing response no longer gets mislabeled as a positively identified wrong
  sensor type. Failed values are hidden in the dashboard.

The pull-up estimate uses `t_r = 0.8473 * R * C` from
[NXP UM10204](https://www.nxp.com/docs/en/user-guide/UM10204.pdf).

## V4 battery protection

Normal V4 builds now enable the policy in `sensor/include/power_options.h`.
V3 keeps its previous opt-in policy.

1. Read the calibrated battery voltage before enabling any external sensor.
   This includes BME680 maintenance wakes and discovery wakes. An invalid ADC
   reading also enters protection instead of authorizing sensor operation.
2. Enter protection below 2800 mV. Send an immediate battery-only telemetry
   report on the saved station channel, then deep-sleep for 86400 seconds.
   Each window uses the transport's bounded saved-channel attempts; failure
   does not shorten sleep. An unpaired node sleeps without scanning channels.
3. On each subsequent wake, check battery voltage first. Remain paused until a
   valid reading reaches 2950 mV. The hysteresis latch survives deep sleep in
   RTC memory; a complete reset/power loss reinitializes it.
4. During live operation, check every ten seconds. Stop acquisition, finish the
   journal, discard pending live transmission and enter the same daily path.
5. Do not initialize sensors, maintain BSEC, replay recordings, scan channels,
   apply station commands or download OTA while paused. The existing trial-boot
   guard can confirm station contact without requesting an update. Configuration
   commands remain pending until normal operation resumes.
6. Report a separate protection mode/flag. The station shows only the battery
   value, preserves it in storage and allows 26 hours before treating the daily
   reporter as offline. An invalid battery reading is flagged, not advertised
   as a valid voltage. The user's normal reporting interval is retained.

This is a repeating interval, not a calendar-day alarm. RTC clock tolerance
affects the wake time; manual resets can trigger another initial report. A
station channel change during protection is not actively searched for. A new
station configuration or charge recovery can take up to the next daily wake.
The existing first-boot OTA guard still rolls back an unconfirmed trial image
if station contact cannot be established; that exceptional boot is not a
normal daily reporting wake.

Deep sleep and daily radio still consume energy. Software at 2.8 V cannot
guarantee that the cell stays above 2.75 V indefinitely, especially with only
50 mV of nominal margin, ADC error and load-induced voltage sag. Verify the
existing U5 battery-protection device's fitted variant and actual disconnect
threshold independently. No PCB or protection-IC changes were made here.

## Verification and hardware acceptance

The host runner executes production drivers and the actual `main.cpp` control
flow with deterministic hardware substitutes. It covers low/high boundaries,
RTC hysteresis, ADC failure, offline/unpaired operation, deferred commands,
battery-only packets, live shutdown, Bosch reference compensation, I2C faults,
sensor FIFO/status faults and existing transport/recording/OTA behavior.
The browser test checks the protection badge and battery-only display.
All 15 host C++ suites passed, including partial MAX30102 FIFO transfers of
one through five bytes. The dashboard browser test passed and all eight
embedded station scripts parsed successfully. PlatformIO builds passed for
`sensor_pcb_v3`, `sensor_pcb_v4` and the station target. The working-tree
whitespace check passed. See [test commands](tests/README.md).

Hardware work still needed:

| Check | What distinguishes the possible causes |
| --- | --- |
| Controlled voltage sweep: 3.4, 3.3, 3.2, 3.1, 3.0, 2.95, 2.8, 2.79 V | Capture battery, U6 VIN/VOUT and J1 supply during startup and RF. Use a suitable supply with current limiting and battery disconnected. A stable battery reading alone does not exclude a brief rail collapse. |
| I2C capture at a failing voltage | Observe SDA/SCL high levels, rise times, ACKs at 0x76/0x77 and chip ID 0x60. Compare supply/bus failure with a valid sensor measurement that fails only in radio delivery. |
| Protection threshold | Compare ADC with a reference near 2.8 V; calibrate the board. Verify sensor rail remains off, one reporting window per 24 h, failed-station cadence and 2.95 V recovery, including a stopped live recording. Measure sleep current. |
| TMP117 | Compare the assembled probe after thermal equilibrium at several stable temperatures against a calibrated reference, including radio on/off. |
| MAX30102 | Compare against a pulse reference at rest and during movement/contact/light changes; inspect waveforms and deliberately interrupt acquisition. |
| LSM6DSOX | Check six stationary orientations, gravity magnitude, stationary gyro offset and known-rate rotation. Repeat under radio/flash load and confirm gaps never masquerade as complete intervals. |

No board was connected or flashed for this review. Software checks establish
configuration, calculations and fault handling; physical accuracy, the reported
intermittent failure and actual battery lifetime still require these measurements.

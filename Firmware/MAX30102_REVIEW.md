# MAX30102 missing readings and interrupted pulse estimates

Reviewed on 2026-10-07 against the production V3/V4 firmware and primary ADI
documentation. The reported symptom is missing values or frequent dropouts;
whether SW2 was pressed, the installed release, PCB revision and module model
are not yet known. Software reproductions identify genuine weaknesses, but do
not establish the cause of this particular physical installation.

## First distinguish idle operation from a measurement failure

MAX30102 acquisition is manual. A provisioned node waits for SW2 / GPIO9;
pressing SW2 starts acquisition and recording, and pressing again stops it.
Normal TMP117/LSM6DSOX interval measurements do not apply to MAX30102.
The idle optical node does not acquire eight seconds of pulse samples. This
is intentional, not an I2C failure, and automatic recording was not introduced.

The initial investigation suspected a 200 ms automatic window. Following the
call through `LiveAcquisition::normal()` showed that MAX30102 is excluded from
that profile entirely. The driver can acquire continuously during a manual
session. A regression test now verifies idle rail shutdown, continuous manual
acquisition for over eight seconds, and shutdown after stop.

The sensor-selection hint previously described continuous acquisition without
explaining SW2. It now explicitly describes start, stop and idle behavior.
The dashboard distinguishes stopped measurement from an unusable IR contact
signal and a sensor/bus fault, and hides retained values in the stopped state.

## Confirmed software weaknesses and corrections

| Case | Previous behavior | Correction and regression |
| --- | --- | --- |
| Dim red, usable IR | Both channels needed at least 10000 counts even though only IR enters the estimator. A clean IR pulse with red at 1500 counts produced no BPM. | Use the IR analysis limits independently. The exact production test failed before the correction and now reports the expected 60 bpm. Red samples remain available; no SpO2 inference is made. |
| Unequal optical paths | One LED-current variable controlled both channels. Bright red requested a decrease while weak IR requested an increase, allowing repeated opposing changes and repeated eight-second restarts. | Control `LED1_PA` and `LED2_PA` independently. A current-dependent red/IR model settles with different currents and then reports 60 bpm. |
| Weak initial IR | Gain could increase only after IR was already above the contact threshold. A weaker but recoverable reflection could stay below it forever. | Allow proportional gain recovery from 2000 counts, while keeping signals below that floor from increasing LED current. A 6000-count initial reflection recovers within the existing current limit; zero red does not increase its LED current. |
| Slow, small pulsation | The 11-sample centered mean attenuated a slow fundamental before the fixed 20-count RMS check. A clean 35 bpm signal with 200-count amplitude at 100000-count DC was rejected. | Use a 25-sample centered mean (one second at the retained FIFO rate). The regression now recognizes 35 bpm; existing supported-rate tests remain successful. |
| Gain discontinuity | Pre-adjustment ADC values and queued waveform points could still be returned immediately after current changes. | Clear waveform validity as well as the estimator/FIFO. Require a new frame before optical values become valid again, and a complete new window before BPM. |

Gain changes remain limited to once per second and at most eight current-register
counts per change. Each channel keeps the existing `0x04`–`0x60` current bounds,
with initial current `0x24`. The wide 50000–220000-count settling band prevents
gain chasing each individual pulse. The proportional target is an engineering
choice, not an ADI physiological calibration. Contact limits and noise checks
still require measurements on the actual optical assembly.

## Manufacturer checks and retained behavior

The [MAX30102 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX30102.pdf)
confirms fixed 7-bit address `0x57`, part ID `0x15`, separate LED-current
registers, six-byte red/IR FIFO frames and four-sample averaging/decimation.
The configured 100 conversions/s therefore yields 25 FIFO pairs/s. The
firmware retains 411 us / 18-bit conversion, 4096 nA ADC range, 200 ms waveform
reports, an eight-second analysis window and one-second estimate updates.

The datasheet specifies separate core and LED supplies: VDD 1.7–2.0 V and
VLED+ 3.1–5.0 V. A breakout's allowable input depends on its regulators and
level shifting; its VIN label is not a measurement of the IC's supplies.
ALC overflow, full FIFO, brownout, partial I2C frames and polling gaps continue
to invalidate affected data. Brownout/partial frames require reinitialization.

[ADI UG6409](https://www.analog.com/media/en/technical-documentation/user-guides/max3010x-ev-kits-recommended-configurations-and-operating-profiles.pdf)
describes independently setting LED currents, checking optical DC/AC signals,
minimizing motion and shielding ambient light. It also distinguishes systolic
and diastolic peaks. Strong periodic correlation alone does not establish a
human pulse or eliminate harmonic and motion errors. The current estimator
remains experimental and does not produce calibrated SpO2.

## Reproduction and validation

- Production driver tests: existing rate, warmup, FIFO, brownout, partial-frame
  and recovery tests, plus dim red, unequal current-dependent channels, weak
  initial IR, small slow AC, static reflection, linear drift, deterministic
  random noise and post-gain freshness.
- Production acquisition-loop test: MAX30102 remains idle without SW2/start,
  runs continuously during a manual session, and releases power after stop.
- Actual embedded dashboard browser test: stopped measurement instructions
  appear and retained BPM is hidden; active waveform, pulse graphs and contact
  status still work. Embedded script parsing is also checked.
- All 22 host suites, both affected browser suites, eight embedded script
  blocks, six cryptographic package tests and V3/V4/station builds passed.
  These checks accompany release 4.3.3 / 40306.
  New signed sensor OTA packages use the existing installation identity;
  the earlier 4.3.2 packages do not contain these targeted corrections.

No hardware was flashed or measured. Synthetic signals are regression fixtures,
not physical pulse-accuracy, motion rejection or power-consumption validation.

## Physical checks for the reported installation

1. Confirm the configured/detected type is MAX30102 and the node is provisioned.
   Release SW2 after boot, press it once, and check that recording is running.
   Avoid holding GPIO9 low during reset, because it is a boot strap.
2. Cover the optical area with a still fingertip, shield side light and avoid
   changing finger pressure. Allow LED gain to settle, then at least eight
   more seconds. A weak initial reflection may take longer than 15 seconds.
3. If no optical values appear during recording, check the module's documented
   input, common ground, ACK at `0x57` and part ID. Measure VDD and VLED+ during
   LED pulses, and verify SDA/SCL high levels on the actual breakout.
4. If optical values exist but no BPM appears, inspect IR level, AC waveform,
   clipping, gain discontinuities and reported gaps. Export a stopped recording
   for analysis; distinguish genuine acquisition gaps from radio delivery loss.
5. Compare a stationary recording against a suitable pulse reference, then test
   finger removal, motion, bright light, weak battery and RF interruptions.
   A module model/marking, PCB revision and installed version are still needed
   to narrow the hardware-specific diagnosis.

The BME280/BME680 acquisition, BSEC ULP schedule, NVS cadence and V4 daily
battery-protection deep sleep are unchanged by these targeted modifications.

# Measurement timing and dashboard design

Firmware 4.3.1, internal OTA release 40304. Acquisition, estimation, transport
and browser refresh have separate clocks. A refreshed page is not a new
measurement. The implementation now transports acquisition-age information,
and the pulse trend receives points only when a new estimate was calculated.

## Selected operating points

| Sensor | When new data exist | Delivery | Useful visualization |
| --- | --- | --- | --- |
| LSM6DSOX | 104 Hz per axis chain, nominal 9.615 ms/sample while powered | Normal: fresh 100 ms summary at configured interval. Recording: 50 ms axis means for five minutes, then 100 ms (approximately 5-6 or 10-11 samples/chain); raw interval peaks retained | XYZ acceleration on a shared g axis; XYZ angular rate on a separate degrees/s axis; zero references and peak tiles |
| TMP117 | 8 conversions: 8 x 15.5 = 124 ms typical active time; 1 s conversion cycle when recording | Normal: first fresh eight-conversion result at configured interval. Recording: on data-ready, polled every 10 ms, then nominally once/s | One-minute temperature trend, >=0.1 C vertical span to avoid magnifying quantization, range/change and recent stability |
| MAX30102 | 100 Hz red/IR conversions, 411 us pulses / 18 bits; four-sample average gives 25 Hz FIFO data, every 40 ms | Up to eight waveform samples/packet, normally five every 200 ms; estimate recalculated once/s from 200 FIFO samples (8 s) | BPM trend over one minute; red/IR optical pulse detail over 10 s, switchable to the full retained minute; contact, warmup and quality |
| BME280 | Configured x1 T/P/H forced conversion: nominal calculation 1.25 + 2.3 + (2.3 + 0.575) + (2.3 + 0.575) = 9.3 ms | Preserve configured room-monitoring interval and deep sleep | Separate selectable T, RH and pressure trends with their own units; persistent 24 h history |
| BME680 | BSEC ULP requests measurements every 300 s; heater/conversion depend on the BSEC request; raw fallback uses 150 ms heater plus Bosch-computed conversion time | Preserve BSEC maintenance cadence and configured radio/cloud interval | T/RH/pressure individually, IAQ with learning/accuracy state, gas resistance as a diagnostic; persistent 24 h history |

104 Hz is the configured IMU rate, not its maximum supported rate. Increasing
ODR is not automatically an accuracy improvement. The 50/100 ms mean is appropriate
for human-scale motion and reduces noise while the separate peak values retain
brief events. It is not a 104 Hz waveform export, precision vibration recording,
orientation fusion or a step-count algorithm. Movements above the displayed
bandwidth can still alias; the boxcar mean is not a complete anti-alias filter.

The TMP117 operating point follows the manufacturer's eight-average, 1 Hz
accuracy test condition. One conversion can take 13-17.5 ms, giving roughly
104-140 ms for eight, with nominal 876 ms standby in the one-second cycle.
Always read the readiness flag rather than relying on this estimate. Sixty-four
averages occupy about a full second without standby and can increase
self-heating. The selected eight-average mode is a reasoned starting point,
not a claim of universal optimum on every breakout. Digital conversion
completion does not mean a probe has reached thermal equilibrium with skin.

The MAX30102 FIFO rate includes averaging: 100 / 4 = 25 Hz, not 100 Hz.
This preserves the pulse shape far better than the old one-value-per-second
optical display. Increasing averaging to 32 at 100 Hz would reduce the signal
to 3.125 Hz, inappropriate for the intended pulse waveform. Eight seconds of
usable data are required after contact, gain adjustment or FIFO loss. Sliding
estimates overlap; a newly displayed BPM is not an independent eight-second
measurement. Window-mean baseline removal in the graph is for visibility only;
the estimator uses its own signal processing. Correlation is not confidence in
clinical accuracy. No SpO2 calibration is claimed.

## Bus, buffering and firmware checks

Explicit live sensor sessions use 400 kHz I2C; Bosch and automatic startup use
100 kHz for more rise-time margin. For the live profiles, ignoring API, START/STOP and clock-stretch
overheads, reading N register bytes takes at least `(N + 3) * 9 / 400000` s.
V4 now checks battery voltage before sensor startup, including maintenance
wakes. Its 100 ms divider wait no longer overlaps environmental conversion;
this is intentional so an undervoltage battery cannot power the sensor first.

| Read | Nominal wire-time lower bound |
| --- | --- |
| TMP117 two-byte result or configuration | 112.5 us |
| LSM6DSOX seven-byte tagged FIFO entry | 225 us |
| MAX30102 six-byte red/IR FIFO pair | 202.5 us |

For the IMU, 208 tagged entries/s use approximately 4.68% of the bus before
status polling. Polling two status bytes every 5 ms adds about 2.25%. MAX30102
data use about 0.51%; three-byte FIFO status reads every 5 ms add 2.7%. TMP117
configuration polling every 10 ms uses about 1.13%. These are separate nodes
with one selected external sensor, not summed loads on one shared bus.

The MAX FIFO holds 32 output samples, about 1.28 s at 25 Hz. Normal 200 ms
delivery is well inside that reserve. The eight-sample radio batch holds
320 ms; if blocked longer, older display samples are dropped and a gap is
reported. The independent pulse-analysis ring still drains available FIFO
samples. FIFO overflow resets the estimator. The acquisition task drains independently of radio waits. OTA transfer is
deferred until the manual session stops. Acquisition queue/FIFO losses are
flagged rather than represented as uninterrupted data.

The complete extended telemetry packet remains below the 250-byte ESP-NOW
limit (enforced by static assertion). Live radio stays initialized between
reports. The configured XIAO ESP32-S3 board enables OPI PSRAM; live rings
prefer PSRAM and internal allocations retain a 96 KiB reserve. The enlarged
dashboard status snapshot is kept off the web task stack. Compile-time checks
cover bus bounds, conversion/cycle compatibility and FIFO reserve.

These checks establish configured capability, not measured RF latency or a
worst-case scheduling bound. The dashboard shows actual last packet spacing,
sample age and estimator processing time to help hardware validation. A busy
Wi-Fi channel, retries or HTTP rendering can still exceed the nominal cadence.

## Timestamps and graphs

Relative ages are generated on the sensor and adjusted while a packet waits
for transmission. The station anchors them to its receive uptime; the browser
maps these relative ages to display time, without requiring NTP. Both arrival
and estimated acquisition times are retained in CSV, with separate optical
rows. Older telemetry prefixes remain readable but lack acquisition timing.

Timestamps are estimates, not synchronized hardware capture timestamps:
TMP117 readiness polling contributes up to roughly 10 ms uncertainty outside
blocking work; MAX FIFO entries are backdated at nominal 40 ms spacing and
its newest entry can be up to a sample period old; IMU times mark FIFO read
completion at the end of an averaging interval. Radio queue/airtime and HTTP
latency add uncertainty. The eight-second pulse window ends at the acquisition
time of its last contributing sample. Do not interpret a packet arrival as
the time all of its samples were acquired.

All live charts retain one minute, show missing-data gaps and avoid invented
interpolation. Pulse waveform detail defaults to ten seconds because individual
pulses are difficult to inspect when a full minute is compressed onto a phone.
The full minute remains selectable and exportable. Temperature stability text
describes recent signal variation only, not readiness for a medical measurement.

## Sources and verification

- [ST LSM6DSOX datasheet DS12814 Rev 4](https://www.st.com/resource/en/datasheet/lsm6dsox.pdf): ODR, FIFO and high-performance configuration.
- [TI TMP117 datasheet SNOSD82D](https://www.ti.com/lit/ds/symlink/tmp117.pdf): sections 6.5, 7.4, 7.6 and 8.2.2; conversion time, averaging, accuracy conditions and self-heating.
- [ADI MAX30102 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX30102.pdf): FIFO, averaging, sample rate, ADC range and pulse width.
- [ADI averaging-rate clarification](https://ez.analog.com/optical_sensing/a/documents/DO19535/if-i-enable-sample-averaging-in-max30101-max30102-and-keep-the-sampling-rate-setting-to-be-the-same-does-the-effective-sampling-rate-go-down): effective FIFO rate falls with averaging.
- [Bosch BME280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf): forced-conversion timing formula. BME680/BSEC cadence follows the pinned production libraries and existing ULP configuration.

Tests exercise data-ready consumption, signed temperature, pulse warmup,
fresh-versus-cached estimates, waveform batches, mean/peak separation, packet
sizes and one-minute retention. Browser fixtures verify all live chart types,
XYZ overlays, dense optical points, hover values and mobile layout. Before
claiming measured accuracy or latency, compare physical sensors with references
and inspect I2C/ESP-NOW timing under the intended simultaneous-node load.

## Manual recording and adaptive motion detail

TMP117/LSM6DSOX normal measurements use the stored measurement interval,
independently of SW2 recordings. The sensor rail is switched off between normal
windows. TMP117 retains eight-conversion averaging and data-ready gating; IMU
startup retains the 100 ms transient discard and FIFO priming, followed by a
100 ms summary window. The next window is scheduled from acquisition startup;
processing and radio delays can add timing uncertainty. No catch-up burst
invents samples for missed windows. The GPIO9 recording button remains armed
through short light-sleep periods; BME280/BME680 keep their deep-sleep policy.

The station may contact a normal precision node every five seconds for status
and configuration. A heartbeat reuses the measured value with an increased
acquisition age and clears freshness flags. It creates no measurement or
recording. Raw-rate motion feedback is omitted from sparse normal windows.
Recording stops restore the configured normal interval, while durable replay
continues separately. OTA pauses normal acquisition and remains deferred while
a manual recording is running. Low-battery protection stops all acquisition
and retains the 24-hour deep-sleep policy.

SW2 controls the three live types. The first five minutes of IMU data use
50 ms means; subsequent data use 100 ms means. Already stored points are not
rewritten or rounded. All IMU samples still enter the 104 Hz peak/mean
calculation. This is a deliberate 20/10 Hz recording profile, not a claim to
record all 104 Hz raw six-axis samples or the sensor's maximum ODR.
TMP117 averaging and MAX30102 waveform/estimator rates stay unchanged for
accuracy; making them less frequent is unnecessary for thirty-minute storage.

Persistent session times are generated before radio transmission and retain
relative optical/BPM ages. A station UTC anchor is saved when available.
After a reset without a saved anchor, the original session retains relative
times and is explicitly marked as having no absolute clock. See
[RECORDINGS.md](RECORDINGS.md) for storage and timestamp bounds.

## Dashboard graph inspection

Incoming measurements update the existing controls and charts in place, so a
selected sensor, time range, display filter and cursor remain usable during
polling. A failed history request retains the last loaded measurements until
the next successful refresh. The one-minute live graph still expires old
samples; retaining a failed request never creates new data.

XYZ traces use different colors and line patterns. The optional three-sample
display average operates within each continuous segment; the faint trace,
cursor readout and CSV retain raw samples. Invalid samples and gaps are never
interpolated by this filter. TMP117 stability feedback still requires real,
continuous data and reports when more samples or a shorter interval are needed.

Ordinary history charts include axes and a dot even for one valid sample.
Intervals from 1 s to 24 h limit the displayed data; a short interval may
correctly contain no reading. Storage buckets retain actual reception
timestamps for new points. Previously saved timestamps retain their existing
resolution. Previous/next window controls browse the history currently retained
on the station, and Now returns to a moving view of the latest data. At normal
intervals below one minute, history retains the latest 900 points in RAM; at
longer intervals it retains the existing 24-hour persistent history.

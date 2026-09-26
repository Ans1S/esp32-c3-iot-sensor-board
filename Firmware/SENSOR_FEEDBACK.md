# Quick sensor feedback

Firmware 4.3.1, internal release 40304. Each live sensor card has a themed
**Quick feedback** panel above its graphs. Window statistics refer to the last
minute, or the selected minute of a saved recording. Missing or invalid data
produce a dash rather than an invented zero.

## LSM6DSOX

- Steps and active seconds are cumulative within a manually started session.
- Cadence is the step-count difference over approximately ten seconds, expressed
  in steps/minute; at least eight seconds of recent, gap-free data are required.
- The acceleration peak is the maximum retained magnitude in the displayed
  minute. It includes gravity.

A constant-memory software heuristic processes every acceleration FIFO sample
at 104 Hz, before interval averaging. It removes a slowly varying magnitude
baseline, smooths the remaining signal, and requires three rhythmic positive
peaks before confirming a walking/running bout (including its first two steps).
Thresholds are 0.12 g for a peak and 0.03 g for rearming, with 26–156 samples
between candidate steps. Active time counts samples whose smoothed absolute
dynamic magnitude exceeds 0.05 g. This is movement time, not time since start.

These are estimated activity metrics for a fixed body-mounted sensor, not a
validated gait classifier or ST's embedded pedometer. Shaking and placement
affect the result. Counters survive driver recovery within a session, while
filter/rhythm state resets at errors. Missing FIFO samples cannot be recovered
by the estimator. Starting a new session resets the counters.

The counters saturate at 65,535 steps/seconds. Recording capacity is reached
well before these limits under the current motion profile. The archive codec
retains the same 64-byte record size and exact existing float/sample-count
values. A format flag distinguishes new feedback-bearing records from older
motion records, which still open without fabricated counters. Both counters
are included in live and recording CSV exports.

## TMP117

The stable value is the mean of the recent ten-second temperature window.
At least eight measurements spanning eight seconds are required, with no gap
over 1.8 seconds or explicit gap marker. A result is stable only when its
range is at most 0.03 °C and its fitted slope is at most 0.06 °C/min in magnitude.
The rate tile uses a least-squares slope, not a noisy two-point derivative.
Stale data invalidate the stability result.

Measurement duration shows session elapsed time during acquisition and full
session duration for saved recordings. It is unavailable in an idle live view.
Signal stability does not establish thermal equilibrium or body-temperature
accuracy. No sensor configuration or conversion timing changes are required.

## MAX30102

The panel shows contact/signal state, the existing correlation score, and
minimum/mean/maximum of valid pulse estimates in the displayed minute. Optical
packets without a new estimate are not additional pulse observations. Current
firmware estimates must have periodic status and correlation of at least 75;
gaps and values outside the driver's 30–240 bpm range are excluded. A correlation
score is not a probability of correctness or a medical accuracy percentage.

For live recovery, press **Start recovery** at the end of exercise and continue
recording for 60 seconds. The baseline is the mean of at least three recent
estimates from the preceding five seconds. The result is baseline minus the
mean around the end of the recovery minute; positive means a pulse decrease.
Insufficient coverage, long estimate gaps or explicit recording gaps invalidate
the result. The marker is scoped to the current session and browser page; it
is not a durable event stored on the sensor or station.

Saved recordings show **Pulse drop · selected minute**, comparing its first
and last five seconds. It can be interpreted as recovery only when the selected
minute actually follows exercise. No automatic exercise-end detection, SpO2
or pulse-rate-variability algorithm is added.

## Compatibility and verification

Update the station before the live sensor firmware. The new station accepts
previous telemetry lengths and both motion archive encodings. The new motion
feedback extends telemetry by four bytes, within the ESP-NOW limit; BME packets
retain their original prefix length. No new partition migration is needed if
the 40302 recording layout is already installed. Existing 30-minute budgets
remain unchanged.

Host checks cover stationary noise, rhythmic motion, isolated impacts, counter
recovery and codec compatibility. Browser checks cover stable/drifting/stale
temperature, valid and interrupted pulse recovery, exports, and desktop/mobile
layouts. On-body step counts and physiological accuracy still require hardware
validation with the intended mounting/contact arrangement.

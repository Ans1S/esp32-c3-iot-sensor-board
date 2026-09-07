# Hardware Test Plan: PCB V3 First, PCB V4 Second

Successful compiler builds validate interfaces and memory limits, but they do
not replace tests on real hardware. Record each result with the date, firmware
commit, board revision, power source and measurement equipment.

## 1. Station only

1. Upload the station target. Confirm that the upload erases the old flash/NVS
   state and that the `W-Charger-XXXXXX` setup network appears.
2. Connect with `W-Charger-Setup`, open `http://192.168.4.1/` and complete the
   unconfigured portal without a website login. Confirm that its ThingSpeak
   step only asks for the optional User API Key; channel IDs and Read/Write API
   Keys must remain in the later dashboard settings.
3. Enter an invalid home Wi-Fi password before the first successful setup and
   wait through at least two reconnect cycles. The setup access point must
   remain available so the credentials can be corrected.
4. Save valid home Wi-Fi credentials. Confirm that the setup AP switches off
   after connection and that `http://w-charger.local/` or the LAN IP reaches
   the station.
5. Set a website password of at least eight characters. Verify that dashboard,
   configuration, history, Wi-Fi scan and ThingSpeak APIs require login. Check
   that CSRF or cross-origin state-changing requests are rejected and that the
   dashboard, API and Wi-Fi-scan responses include the configured no-cache,
   frame, MIME-sniffing and content-security headers.
6. Log in from two clients. Five failed attempts from one client must trigger a
   30-second block without blocking the other client. Signing out one client
   must invalidate only its server-side session; the other session must remain
   valid. Confirm that an idle session expires after 30 minutes.
7. Restart the station and confirm that settings and local history persist,
   while all previous session cookies no longer authenticate.
8. Generate several simultaneous sensor reports while exercising history and
   settings pages. Every sensor must receive its configuration response within
   its normal response window, and the corresponding latest value and history
   point must still appear after the asynchronous persistence task runs.

## 2. Sensor PCB V3

1. Upload `sensor_pcb_v3`. An unprovisioned sensor must find the station without
   a compiled-in station MAC address or radio channel. Verify this once during
   the fast pairing window and once after it has expired; every discovery wake
   must cover all 13 channels.
2. Confirm that the sensor advertises every ten seconds during the initial
   ten-minute fast-pairing window, but refreshes its sensor/battery snapshot
   only every five minutes. Both the captive setup page and the dashboard's
   pending-sensor list must show **Setup mode**.
3. Compare dashboard temperature and humidity with reference instruments.
4. Measure battery voltage simultaneously with a multimeter. The current V3
   firmware intentionally uses the field-validated base scaling factor 1.67,
   which takes precedence over a nominal schematic ratio. Record the residual
   error before applying any per-sensor calibration.
5. Set the interval to one minute in the web interface. Confirm that
   `appliedRevision` matches the station revision no later than the second
   subsequent contact and that the dashboard changes to **Energy-saving mode**
   only after the sensor reports that mode.
6. Switch the station off during a transmission. The sensor must still power
   down safely, enter deep sleep and retry only at the next configured report
   interval.
   Repeat once after the station has acknowledged the unicast at MAC level but
   before its configuration response is received: the sensor must not scan
   other channels in that wake.
7. Add a sensor while the station is still using its captive-portal setup AP,
   then finish Wi-Fi setup on a different channel. At the next due report, the
   sensor must try its saved setup channel, scan all remaining channels in the
   same wake, receive the station response and persist the new channel. Change
   the router channel again after this transition; steady-state recovery must
   then use at most three rotating channels per scheduled exchange. During a
   received recovery packet, the dashboard must show **Channel recovery** and
   return to **Energy-saving mode** after the next normal report.
8. Queue a sensor factory reset, interrupt the station during its response and
   verify that the command remains pending until the sensor acknowledges the
   matching revision.
9. Delete a configured sensor from the station without resetting its power.
   At its next scheduled known-channel check-in, the station must return
   `provisioned=false`. The sensor must clear the station identity, wake again
   after one second and reappear as an unprovisioned sensor using a complete
   13-channel discovery scan.
10. Upload a new sensor firmware image without erasing NVS manually. The ELF SHA
   change must clear the old station assignment once; another restart with the
   same image must not clear it again.
11. With diagnostic logging enabled for this test only, force a failed unicast
    send and confirm that the next attempt starts without the additional
    260 ms configuration-response timeout. Repeat with broadcast discovery and
    confirm that discovery still retains its response window.

## 3. BME680 and IAQ on PCB V3

1. Connect the BME680 to the switched I2C supply, restart the sensor and verify
   that the add-sensor wizard detects `BME680`. Then select BME680 explicitly
   and initially use a five-minute interval.
2. Temporarily build with `SENSOR_DIAGNOSTIC_LOGGING=1` and check the serial log
   and timing: BSEC must use only `ULP/300 s` from the first start. There must
   be no LP/three-second cycle and no later sample-rate switch. Permanent direct
   raw measurements indicate a fault.
   During the heater/conversion wait, timer Light-sleep must return at the
   expected measurement deadline without resetting I2C or interrupting power
   on GPIO10.
3. The dashboard must show temperature, humidity, pressure, gas resistance and,
   after BSEC learning, Static IAQ. While accuracy is 0, the dashboard must show
   the real elapsed learning time without a fixed-duration progress bar, and
   ThingSpeak must not receive IAQ. When accuracy first reaches 1, the large
   learning notice must disappear permanently and the compact Accuracy badge
   must remain visible during normal operation.
4. Operate the sensor in a normally used room for at least 24 hours and observe
   whether accuracy progresses through 1, 2 and eventually 3. Ventilation
   should improve IAQ with a delay, while ordinary indoor pollution should
   worsen it. Do not expose the sensor directly to liquids or concentrated
   vapours.
5. After reaching accuracy 2 or 3, disconnect the battery briefly. A saved BSEC
   state must load after restart; restarting the complete long-term learning
   process indicates a persistence fault.
6. After thermal equilibrium, compare temperature and humidity with a known
   reference instrument. Change the temperature offset only in small steps and
   evaluate at least three further cycles.
7. Set the reporting interval to 30 minutes. The BME680 must still measure every
   five minutes, while ESP-NOW and ThingSpeak operate only every 30 minutes.
   Include several seconds of awake processing in the test and verify that the
   logical report clock does not postpone transmission by an extra five-minute
   cycle.
8. Alternate between BME680 and BME280 and update the station selection. An
   incorrect explicit type must appear as a read failure/type mismatch;
   **Automatic** must recognize both chip IDs at `0x76` and `0x77`.
9. Measure the sensor rail directly during deep sleep on both PCB V3 and V4; it
   must be 0 V. After at least ten fully power-cycled sensor measurements, IAQ
   learning must continue instead of starting with a fresh algorithm state on
   every wake-up.
10. Reset the IAQ learning state in the wizard. After the next contact,
    accuracy must intentionally restart at 0 and then begin increasing again.
11. Force one BSEC cycle to provide only a direct Bosch raw fallback. The last
    genuine IAQ may remain visible as stale, but it must not create a new
    history point or ThingSpeak upload.

## 4. ThingSpeak

1. Store only the User API Key during initial setup. In the dashboard, load the
   account channels and create a private channel for a sensor.
2. Confirm that channel ID, channel profile and write key remain available
   after a station restart.
3. Map every supported quantity to a unique field and verify its unit and
   destination. Battery voltage is uploaded in volts, gas resistance in ohms
   and pressure in hPa. Fields configured as `Do not upload` must remain empty.
4. Confirm that an HTTP 200 response without a positive entry ID is shown as a
   failure rather than **Connected**.
5. Disconnect the internet for several measurements and restore it. Observe
   `droppedJobs`; the current RAM queue is not persistent across restarts and
   does not guarantee delivery of offline samples.
6. Verify that unprovisioned discovery telemetry and stale BSEC fallback IAQ
   values are never uploaded.
7. After a successful upload, confirm that the overview status remains
   **Connected** while the redundant success message is hidden. A later upload
   or station-connection error must make its warning visible again.

## 5. Local station history

1. Set one BME280 sensor to a 10-minute measurement interval. Confirm that the
   dashboard and `/api/history` report 10-minute steps while retaining at most
   the rolling last 24 hours.
2. Change the same sensor to 20 minutes. Existing recent history should remain,
   be rebucketed to 20-minute steps and survive a station restart.
3. After collecting several 10-minute points, change the interval to two
   minutes. The existing line must remain plotted while new two-minute points
   are appended; cursor inspection alone is not sufficient.
4. Repeat with a second sensor at a different interval and confirm that each
   chart keeps its own step size. Missing measurements must create a visible
   line gap instead of being connected as continuous data.
5. With website protection enabled, confirm that the protected-access banner
   disappears automatically after its initial display. The unprotected warning
   must remain visible when no password is set.

## 6. Sensor energy profile

Before measuring current, verify the configured-mode radio policy:

1. Add a BME280 sensor, wait for the station to acknowledge provisioning and
   select a ten-minute interval. Across three successful cycles, verify that it
   wakes at the configured interval, sends unicast on the saved channel and
   emits neither discovery beacons nor scans on other channels.
2. Power-cycle that sensor without changing its firmware. The persisted
   assignment must keep it in the same energy-saving mode; it must not return
   to ten-second discovery.
3. Repeat with a reporting interval longer than five minutes on a BME680. The
   node may wake every five minutes for BSEC ULP, but ESP-NOW must remain off on
   intermediate wakes and transmit only when the configured report is due.
   On PCB V4, GPIO6 must remain low throughout those intermediate wakes.
4. Delete the sensor or complete its factory reset and verify the opposite:
   ten-second discovery and complete 13-channel scans must resume.
5. Observe `/api/status` for a strong, stationary V4 link. The
   `sensorTxPowerDbm` request may start at 11 dBm and must reduce to 8.5 dBm
   only after eight reports at or above -55 dBm. Attenuate the link: two
   reports at or below -72 dBm must raise one step, while a report at or below
   -82 dBm must raise one step immediately. Packet delivery must continue
   throughout.

Use a power analyser to measure at least 20 cycles of each case:

- wake-up to sensor power-on
- BME280 measurement duration
- BME680 ULP measurement including heater phase
- ESP-NOW transmission on a known channel
- BME680 intermediate cycle without ESP-NOW at a longer reporting interval
- stale-channel recovery scan as the radio worst case
- deep-sleep current
- charge consumed by a normal measurement cycle
- discovery mode during both the ten-second and five-minute phases

Derive runtime only from measured charge per cycle, the battery's measured
usable capacity and self-discharge. Runtime estimates in documentation are not
acceptance values.

## 7. Battery-voltage calibration

1. With USB disconnected, measure the battery simultaneously using a calibrated
   multimeter and the dashboard.
2. Enter the multimeter value in the sensor wizard and save it. The new factor
   must arrive at the next sensor wake-up.
3. Compare again at at least three voltages, for example 4.15 V, 3.70 V and
   3.25 V. One-point calibration may correct gain but must not conceal strong
   non-linearity or an incorrectly populated resistor network.
4. Confirm that the calculated correction factor is rejected outside the
   permitted range of 0.7 to 1.3.
5. Reset calibration to its default and verify that correction factor 1.0000 is
   active again on top of the board-specific base scaling.

## 8. Moving to PCB V4

1. Before assembly, continuity-check GPIO6/`ADC_EN` and
   GPIO10/`SENSOR_PWR_EN` against both schematic and PCB.
2. During sleep, GPIO6 must be low and the battery divider must draw no
   measurable continuous current.
   With a BME680 report interval longer than five minutes, confirm the same on
   every intermediate measurement wake and confirm that a due report still
   contains a valid battery value.
3. GPIO10 is active low on V4: low enables the PMOS sensor rail and high
   disables it.
4. Compare battery voltage near 4.2 V, 3.7 V and 3.2 V. V4 uses 100 kOhm over
   150 kOhm and a base scaling factor of 1.667.
5. Repeat BME280, BME680, discovery, stale-channel recovery and factory-reset
   tests with the `sensor_pcb_v4` target.
6. Perform brownout, USB connect/disconnect and charger/battery switchover
   tests.
7. Use the V4 environment for longer field tests only after pin, voltage,
   polarity and current tests have passed.


## Optimization regression acceptance (2026-09-07)

Run the host checks in `tests/README.md` before flashing. Build success does not
replace these physical acceptance checks. Exercise both V3 and V4, BME280,
BME680 and disabled environmental sensing.

1. Capture battery current, sensor rail, SDA/SCL and V4 ADC enable through a
   complete wake. Verify unchanged settling, stable gate levels during all
   Light-sleeps, no rail glitches/back-powering, and return to the sleep baseline.
2. Compare battery readings with the previous image and a multimeter at several
   cell voltages and calibration factors. Retain the 16-sample rejection test;
   allow the documented at-most-1-mV arithmetic rounding change.
3. Disconnect the environmental sensor and hold I2C/status signals in fault
   states. Confirm bounded acquisition, rail shutdown and normal retry cadence.
   In-flight I2C transactions can finish after the cooperative deadline.
4. With a provisioned BME680 and a one-hour report interval, cold-start while the
   station is off. Confirm that five-minute maintenance wakes do not transmit
   every time and that failed acquisition does not cause one-second wake loops.
5. Pair on the setup AP, then complete home-Wi-Fi setup on a different channel.
   Check full initial recovery, subsequent three-channel rotating recovery,
   stable-channel replies, and recovery when only the application ACK is lost.
6. Inject delayed/missing send callbacks and delayed application responses.
   Verify no overlapping sends or channel change before completion, and that
   valid late replies avoid an unnecessary retry.
7. Load the station with history downloads, configuration changes and HTTPS
   activity while sensors report. Measure receive-to-response latency against
   the sensor's 260-ms response window. Under artificial persistence overload,
   verify `espnow.persistenceDrops` counts discarded oldest events and radio
   replies continue. Confirm no unexpected loss at the supported normal load.
8. Add/delete more than 20 distinct MAC addresses over time (maximum 16 at once).
   Confirm responses continue and inspect `espnow.peerFailures`. Delete and
   re-add a MAC with queued samples; old generations must not repopulate history.
9. Verify interval-dependent 24-hour history, reception-time buckets, restart
   persistence, interval changes and exclusion of stale fallback IAQ. Exercise
   memory pressure and confirm history failure is reported without consuming
   the internal heap reserve needed by radio/TLS.
10. Queue multiple cloud channels: one channel's rate limit must not delay other
    eligible channels. Check per-channel order, transient retries (three maximum),
    permanent errors, overflow and 24-hour expiry during a prolonged outage.
    Inspect original receive timestamps, shared-channel `status` timestamps and
    behavior before NTP synchronization. Account for an uncertain remote commit
    if the HTTP response is lost.
11. Keep a BME680 running through learning and compare accuracy progression,
    fresh IAQ, raw fallback, NVS checkpointing, reset and re-pairing with the
    baseline. Confirm configured report deadlines and ULP maintenance cadence.
12. Verify visible-page refresh and immediate refresh after returning from a
    hidden browser tab. Recheck setup, sensor editing, deletion, calibration
    reset, cloud field mapping and existing login behavior.
13. Only on a controlled supply, test an explicitly selected voltage-guard
    threshold, hysteresis, ADC failure while paused and recovery after charging.
    Default firmware keeps the guard disabled. The storage experiment must
    retain full channel coverage despite longer discovery intervals.
14. Compare normal `-Os`/80-MHz, `-O2`, 160-MHz and LTO images using integrated
    charge per complete cycle, timing, code size and repeated cold/deep-sleep
    starts. Exercise BSEC and poor RF before selecting another default. Do not
    infer an energy reduction from image size or CPU clock alone.

Keep the existing fresh-install behavior in mind: flashing a new sensor image
clears its assignment/learning state, and station USB uploads erase settings.
Record build environment, battery voltage, sensor, RF conditions, sample count,
mean/tail wake duration and charge for each comparison.
# OTA hardware acceptance

The signed ESP-NOW update implementation requires the V3/V4 fault-injection
checks in [OTA.md](OTA.md#verification), including actual flash identification,
power cuts, trial-boot rollback, low-voltage operation and concurrent telemetry.
Host tests and successful builds do not mark these physical checks as passed.

## LSM6DSOX and energy acceptance

On both PCB revisions, verify I2C identity at 0x6A and 0x6B, axis signs and scale
with known stationary orientations and controlled rotation. Check interval
peaks, FIFO overflow indication and stale-data faults after disconnection.
Exercise 1 s and 5 s reports, short graph windows and CSV summaries; verify the
900-entry RAM limit and restart behavior without subminute history flash writes.

Compare BME280/BME680 charge per reporting cycle and deep-sleep current with
the preceding release under identical supply, interval and RF conditions.
Measure maintenance-only BME680 wakes separately. Confirm sensor rail and V4
ADC shutdown, no normal diagnostic serial activity, and no trial-boot retry
extension after acceptance. Account separately for the added OTA offer query.
A provisioned LSM6DSOX intentionally stays awake; do not apply environmental
battery-life expectations to continuous motion mode. See
[power management](POWER_MANAGEMENT.md) for the audit scope.

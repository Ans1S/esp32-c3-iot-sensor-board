# Hardware Test Plan: PCB V3 First, PCB V4 Second

Successful compiler builds validate interfaces and memory limits, but they do
not replace tests on real hardware. Record each result with the date, firmware
commit, board revision, power source and measurement equipment.

## 1. Station only

1. Upload the station target. Confirm that the upload erases the old flash/NVS
   state and that the `W-Charger-XXXXXX` setup network appears.
2. Connect with `W-Charger-Setup`, open `http://192.168.4.1/` and complete the
   unconfigured portal without a website login.
3. Enter an invalid home Wi-Fi password before the first successful setup and
   wait through at least two reconnect cycles. The setup access point must
   remain available so the credentials can be corrected.
4. Save valid home Wi-Fi credentials. Confirm that the setup AP switches off
   after connection and that `http://w-charger.local/` or the LAN IP reaches
   the station.
5. Set a website password of at least eight characters. Verify that dashboard,
   configuration, history, Wi-Fi scan and ThingSpeak APIs require login, failed
   attempts are rate-limited and sign-out invalidates the session.
6. Restart the station and confirm that settings and local history persist,
   while the previous session cookie no longer authenticates.

## 2. Sensor PCB V3

1. Upload `sensor_pcb_v3`. An unprovisioned sensor must find the station without
   a compiled-in station MAC address or radio channel.
2. Confirm that the sensor advertises every ten seconds during the initial
   ten-minute fast-pairing window, but refreshes its sensor/battery snapshot
   only every five minutes.
3. Compare dashboard temperature and humidity with reference instruments.
4. Measure battery voltage simultaneously with a multimeter. The current V3
   firmware intentionally uses the field-validated base scaling factor 1.67,
   which takes precedence over a nominal schematic ratio. Record the residual
   error before applying any per-sensor calibration.
5. Set the interval to one minute in the web interface. Confirm that
   `appliedRevision` matches the station revision no later than the second
   subsequent contact.
6. Switch the station off during a transmission. The sensor must still power
   down safely, enter deep sleep and retry only at the next configured report
   interval.
7. Change the router's Wi-Fi channel. At the next due report, the configured
   sensor must try its saved channel and then find the station by scanning the
   remaining channels during the same exchange.
8. Queue a sensor factory reset, interrupt the station during its response and
   verify that the command remains pending until the sensor acknowledges the
   matching revision.
9. Upload a new sensor firmware image without erasing NVS manually. The ELF SHA
   change must clear the old station assignment once; another restart with the
   same image must not clear it again.

## 3. BME680 and IAQ on PCB V3

1. Connect the BME680 to the switched I2C supply, restart the sensor and verify
   that the add-sensor wizard detects `BME680`. Then select BME680 explicitly
   and initially use a five-minute interval.
2. Check the serial log and timing: BSEC must use only `ULP/300 s` from the
   first start. There must be no LP/three-second cycle and no later sample-rate
   switch. Permanent direct raw measurements indicate a fault.
3. The dashboard must show temperature, humidity, pressure, gas resistance and,
   after BSEC stabilization, Static IAQ. IAQ begins at accuracy 0/learning; the
   initial ULP stabilization normally takes about 20 minutes. ThingSpeak must
   not receive IAQ while accuracy is 0.
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

1. Store the user API key in the station and create a private channel for a
   sensor.
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

## 5. PCB V3 energy profile

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

## 6. Battery-voltage calibration

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

## 7. Moving to PCB V4

1. Before assembly, continuity-check GPIO6/`ADC_EN` and
   GPIO10/`SENSOR_PWR_EN` against both schematic and PCB.
2. During sleep, GPIO6 must be low and the battery divider must draw no
   measurable continuous current.
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

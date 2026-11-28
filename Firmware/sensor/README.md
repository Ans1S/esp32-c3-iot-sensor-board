# W-Charger Sensor Firmware

PCB V3 and V4 support ten selectable I²C sensors. The five additions in **4.4.10** are listed below.

| Sensor | Measurements | Operation |
| --- | --- | --- |
| **BMP280** | Temperature and barometric pressure | Forced measurement at the reporting interval |
| **BME688** | Temperature, humidity, pressure, gas resistance and Static IAQ | Five-minute Bosch BSEC maintenance with separate reporting |
| **CCS811** | Estimated CO₂ equivalent (eCO₂) and total volatile organic compounds (TVOC) | Paired nodes keep the heater powered between reports for warm-up and baseline stability |
| **VL53L1X (TOF400C)** | Time-of-flight distance, range status, signal and ambient rates | Ranging at the configured interval |
| **AS7341** | Eight visible spectral bands plus clear and near-infrared counts | Two measurement passes with gain and saturation status |

LSM6DSOX, TMP117 and MAX30102 also support [SW2 recording](../PRECISION_SENSORS.md).

One shared firmware codebase supports PCB V3 and V4. The only differences are
the build environment and the `hardware_profile`.

```text
pio run -e sensor_pcb_v3 -t upload   # existing test board
pio run -e sensor_pcb_v4 -t upload   # current board
```

No Wi-Fi credentials, station MAC address, radio channel or measurement
interval must be written into the source code. An unpaired node can visit all 13 channels when no station answers. It sends
at most one discovery frame per visited channel and can finish early on an
assignment. After an idle station replies, a slow wake or every fourth rapid
discovery sequence can continue searching for another station. Initial sleeps
last ten seconds for ten minutes, then five minutes. Once an older node hears
an explicit enrollment invitation, it can schedule up to three ten-second
sleeps within 45 seconds. The first invitation starts a ten-minute cooldown.
Pairing persists after the node saves the assignment.

Firmware changes are identified by the ELF SHA fingerprint. A new image
invalidates the RTC configuration cache but preserves station assignment and
compatible calibration data in NVS. A station-requested factory reset clears
pairing, calibration and the node's local recording archive. A local station
transfer is available when the previous station is unreachable. Signed firmware updates can now be uploaded through the station;
see [the OTA guide](../OTA.md) for initial USB installation and recovery.
For each scheduled transmission, the sensor first tries the known
channel twice. A newly paired sensor retains a commissioning recovery guard: if
the saved setup-AP channel stops responding, it scans all 12 other channels in
the same wake. This can recover contact when the station joins the configured
home Wi-Fi network and the channel change is heard during that sweep. Once a new
channel is found, later stale-channel events test at most three other channels
per scheduled report, with one maximum-power attempt each, and continue from a
retained RTC cursor. This bounds steady-state failure energy without adding a
separate retry wake. A one-time sub-second offset in the first deep-sleep
interval staggers paired nodes without keeping their radios active. Both PCB
targets run the CPU at 80 MHz, the lowest supported maximum frequency while
ESP-NOW keeps the 80 MHz APB clock available.

The operating modes are intentionally mutually exclusive. Energy-saving mode
requires all three persisted facts: `provisioned=true`, a valid station MAC and
a valid saved channel. In that mode the sensor uses its configured measurement
interval, addresses only the station and starts on the saved channel. If any
part of the assignment is absent, or the station returns `provisioned=false`
after deletion, the sensor clears the station identity and enters discovery
mode with broadcast discovery across the supported channels.

| Scenario | Selected mode | Radio and wake behaviour |
|---|---|---|
| Fresh flash, erased NVS or factory reset | Discovery | Up to 13 channels per wake, with early completion and invitation timing described below |
| Deleted by the station and deletion response received | Discovery | Station identity cleared; all-channel broadcast resumes |
| Incomplete or invalid persisted assignment | Discovery | Safe fallback; energy-saving mode is rejected |
| Fully provisioned BME280 or no environmental sensor | Energy saving | Configured interval; saved-channel unicast only while communication succeeds |
| Fully provisioned BME680/BME688 | Energy saving | Five-minute BSEC ULP wake; radio only at the configured report interval |
| Fully provisioned CCS811 | Continuous gas sensing | Heated sensor stays powered between interval reports; V4 battery protection takes priority |
| Saved station channel no longer responds | Recovery | Bounded recovery, except the protected setup-AP transition |

Choose **Automatic**, the attached module, or **No environmental sensor** in the station. The node receives the selection at its next exchange. Automatic detection checks the Bosch IDs and variant first, followed by the IMU, precision temperature, optical, spectral, distance and gas drivers.

A normal BME280 cycle performs only these steps:

1. On V4, settle and read the battery divider before enabling the sensor.
   Below 2.8 V (or on an invalid ADC reading), send only battery status and
   deep-sleep for 24 hours. Resume at 2.95 V. V3 retains overlapping ADC settling.
2. Read the BME280 with checked I2C transfers, one forced x1 T/P/H conversion
   and a coherent data burst. Bosch/automatic startup uses 100 kHz I2C.
3. Send a validated telemetry packet over ESP-NOW.
4. Wait briefly for the station's configuration response.
5. Write the new configuration to NVS only when its revision has changed.
6. Safely disable the sensor and ADC paths and enter deep sleep.

The V4 policy also applies before discovery/BME680 maintenance and checks live
sessions every ten seconds. The station displays an explicit battery-protection
state. See [sensor and low-voltage review](../SENSOR_REVIEW.md) for register
checks, recovery behavior, limitations and the hardware acceptance procedure.

The BME280 configuration deliberately follows Bosch's energy-efficient weather
monitoring profile: forced mode, 1x oversampling for temperature, pressure and
humidity, and no IIR filter. Higher oversampling adds little practical value to
slow-changing indoor measurements but increases measurement and active time.
The station therefore wakes this sensor type only at the selected reporting
interval.

## Hardware profiles

| Function | PCB V3 | PCB V4 |
|---|---:|---:|
| ADC | GPIO3 | GPIO3 |
| ADC enable | always active | GPIO6, active high |
| Sensor power | GPIO10, active high | GPIO10, PMOS active low |
| I2C SDA / SCL | GPIO5 / GPIO4 | GPIO5 / GPIO4 |
| Battery scaling | field-validated software factor 1.67 | 100 kOhm / 150 kOhm, factor 1.667 |

J1 uses pin 1 for sensor power, pin 2 for SDA, pin 3 for SCL and pin 4 for
ground on both boards. On V3, GPIO10 drives pin 1 directly. R9 and R10
(5.1 kOhm) pull SDA and SCL up to the permanent 3.3 V supply. Releasing the
MCU's I2C pins therefore leaves an external path to a connected module.
On V4, GPIO10 controls Q5. R18 pulls the gate high to turn Q5 off.
R9 and R10 connect to J1 pin 1 on this revision, so the sensor and the
board's I2C pull-ups share the switched supply. GPIO6 controls the V4
battery divider independently.

The station can logically restore a sensor to factory settings on its next
contact. The web interface allows integer-second intervals from one second to 24 hours,
covering both short tests and energy-efficient long-term operation.

## BME680, BME688 and indoor air quality

BME680 and BME688 run Bosch's BSEC2 algorithm and use **Static IAQ** for a stationary
indoor device. An index calculated independently from gas resistance would not
be equivalent: humidity compensation, sensor ageing, drift and the adaptive
baseline are part of the algorithm. The index ranges from 0 (clean air) to 500
(heavily polluted), and the associated accuracy from 0 to 3 appears directly
in the dashboard. While accuracy is 0, the dashboard displays "calibrating"
and the station does not send IAQ to ThingSpeak. Temperature, humidity,
pressure and gas resistance remain available.

Commissioning and long-term operation deliberately use the same routine:

- `BSEC_SAMPLE_RATE_ULP` is the only mode used from the first start throughout
  the device's lifetime, with exactly one internal BME680 measurement every
  five minutes;
- while the BME680 completes its forced TPH/heater conversion, the ESP32-C3
  uses timer Light-sleep with the switched sensor rail latched at its active
  level; a failed Light-sleep setup falls back to the original timed delay;
- there is no switch between LP and ULP, avoiding a jump in the learned BSEC
  baseline caused by an operating-mode change;
- sensor power is off and the ESP32-C3 is in deep sleep between measurements;
- radio transmission and ThingSpeak strictly follow the configured reporting
  interval, such as 10, 30 or 60 minutes;
- the logical reporting clock includes both deep-sleep and awake time, so a
  report is not delayed by another five-minute BME680 cycle;
- BSEC state and logical time remain in RTC RAM; a flash checkpoint is written
  when accuracy improves and then no more than once every six hours;
- after a battery replacement, the latest flash state is restored instead of
  losing the adaptive baseline completely.

The firmware loads only Bosch's ULP configuration blob intended for a 3.3 V
sensor supply. The direct Bosch sensor API remains an availability fallback for
temperature, humidity, pressure and raw gas resistance. It intentionally does
**not** generate a substitute IAQ value: only a Static IAQ value actually
provided by BSEC is marked or uploaded as IAQ.

The firmware commands sensor power off between measurement windows. On V3,
verify the residual voltage at J1 pin 1 with the actual module connected
before assuming a complete power-on reset. The BME680 is reinitialized after
every wake-up, while the previously serialized BSEC learning state and a
continuous logical time are saved before shutdown and restored on the next
start. This follows Bosch's intended `getState()`/`setState()` flow for complete
system shutdowns.

Initial learning remains active while BSEC Accuracy is 0 and ends permanently
after Accuracy first reaches 1. Its duration depends on the observed air
conditions and is therefore not represented by a fixed countdown. BSEC
continues learning the gas/IAQ path in normal operation and can improve through
Accuracy 2 to 3. There is no separate fast-start mode with increased heater or
radio load.

When BME680 or BME688 is selected explicitly, the reporting interval must be five minutes
or a multiple of five minutes. The default temperature offset of `0.466 degC`
comes from Bosch's ULP example and can be adjusted on the station for each
installation. It should be changed only after comparison in an enclosure that
has reached thermal equilibrium.

The implementation is based on Bosch's official
[BME68x Sensor API](https://github.com/boschsensortec/BME68x_SensorAPI),
[BME68x Arduino library](https://github.com/boschsensortec/Bosch-BME68x-Library)
and [BSEC2](https://github.com/boschsensortec/Bosch-BSEC2-Library). The BME680
limits, IAQ scale and ULP power figures are documented in the
[BME680 data sheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme680-ds001.pdf).
BSEC2 contains a proprietary binary library; use and redistribution are also
subject to the [Bosch BSEC license](https://github.com/boschsensortec/Bosch-BSEC2-Library/blob/master/LICENSE.md).

During a build, the pinned BSEC2 2.1.5 source is automatically adjusted with a
small timing patch. In forced mode, BSEC2 initially starts only the measurement;
the patch waits for the configured temperature/pressure/humidity and heater
duration plus a conservative margin in Light-sleep, then retries `NO_NEW_DATA`
inside a 500-ms fetch budget propagated into the I2C and wait callbacks. One
in-flight I2C transaction may finish after a deadline (20-ms bus timeout).
Initialization, recovery and acquisition share a cooperative six-second budget;
the outer BSEC output wait also retains its five-second limit. Only fresh BSEC
output is accepted. A current future ULP deadline controls sleep, rounded up to
whole seconds. An expired or unavailable schedule falls back to the normal
cadence, avoiding repeated one-second error wakes. This is required because this design removes sensor power
after every cycle. The downloaded Bosch library itself is not copied into the
repository.

If a cycle nevertheless provides only a direct Bosch raw measurement, the last
genuine BSEC IAQ value remains visible on the station and is marked as stale.
That old value is neither added as a new history point nor uploaded to
ThingSpeak. The next scheduled BSEC cycle tries again to produce a fresh IAQ
value.

The station displays an initial learning notice only until Accuracy first
reaches 1. Afterwards a permanent compact Accuracy 0-to-3 status remains in the
normal sensor header. Measurement, radio and cloud operation use the same ULP
routine from the beginning.

## CCS811 gas measurements

Paired nodes select the slowest CCS811 drive that supports the reporting interval.
For example, a one-minute report uses Mode 3 with one heater pulse cycle per minute.
The ESP32-C3 enters timer Light-sleep with the sensor rail held on between radio
and battery checks. Discovery identifies the module without starting its heater.
Changing to a slower drive includes ten minutes in idle. After starting a drive,
the node withholds gas values for 20 minutes of conditioning. A new sensor may
need 60 minutes of initial operation before its estimates are usable.

STATUS `0x90` means the application is running and no new sample is ready.
Only a fresh, error-free algorithm frame supplies eCO₂ and TVOC to history.
A missing frame past the drive period plus timing margin raises a read fault.
Mode-specific baselines are saved daily in RTC memory. A matching baseline up
to seven days old is restored after conditioning. The first published reading
then comes from a subsequent conversion. Full loss of RTC power clears this cache.
Use the baseline reset when replacing the module at the same address.

J1 carries power, ground, SDA and SCL. The breakout determines the state of
`nWAKE`. A permanently grounded `nWAKE` allows I²C access and raises standby
current. Measure the assembled board to determine battery consumption.

[ams CCS811 datasheet, pp. 11–12 and 18–20](https://cdn.sparkfun.com/assets/2/c/c/6/5/CN04-2019_attachment_CCS811_Datasheet_v1-06.pdf)
· [ams AN000370 baseline save and restore](https://dfimg.dfrobot.com/nobody/wiki/ab83d61ed52c66c2dd4067eed25b0c35.pdf)

## Discovery and re-adding sensors

An unpaired node schedules ten-second sleeps in its first ten minutes and
five-minute sleeps afterwards with the production settings. The radio, boot
and sensor work add to these intervals. Discovery can reuse an environmental
snapshot for five minutes. V4 reports the fresh voltage from its protection
check on each wake. A sleeping node that has been deleted on its station first
has to transmit on its existing schedule to learn that its pairing was removed.

Opening **Nearby sensors** or clicking **Find sensors** on a station starts a
six-minute enrollment window. After hearing an invitation, an older unpaired
node can schedule three ten-second sleeps within 45 seconds. The ten-minute
cooldown starts with the first invitation. Lost replies do not refill the
budget. An older V5 station or node continues ordinary discovery. Sensor
settings take effect after the node saves the assigned revision.

An unpaired node briefly listens for another offer after a pending reply. An
accepted assignment takes priority. On every fourth fast wake, or on a slow
wake that hears an idle station, it can search past that station for an
operator on another channel. A wake sends at most one discovery frame per
channel. A paired node continues to use its saved station through outages.

**To replace an unreachable station**, power-cycle the paired node with SW2
released. Immediately after power is applied, press SW2 within the first
second, hold it for about seven seconds and release. The firmware accepts a
continuous five-to-ten-second hold measured from when it samples the pressed
switch. It saves the cleared assignment and enables automatic module
detection. Local recordings and the battery calibration factor survive the
transfer. Select the node on the replacement station. Short presses during
normal precision operation continue to control recordings. V4 battery
protection and an outstanding OTA trial take precedence. GPIO9 selects the ROM
bootloader if it is low at reset. [Espressif boot-mode reference](https://docs.espressif.com/projects/esptool/en/latest/esp32c3/advanced-topics/boot-mode-selection.html)

## Extending the drivers

`environmental_sensor` is only the shared facade. Measurements use the neutral
`EnvironmentalReading` type, while BME280, BME680 and BSEC persistence each
have separate header and CPP files. A future I2C sensor therefore needs a new
driver, a new `EnvironmentalSensorType` value, probe and initialization paths in
`EnvironmentalSensor`, and matching protocol, persistence, dashboard and power-policy handling.

## Calibrating battery voltage

`analogReadMilliVolts()` already uses the ESP32-C3 ADC calibration data. A
multimeter reference value can additionally be entered for each sensor. The
station uses it to calculate a one-point gain factor that corrects external
voltage-divider tolerance and the remaining measurement error:

`new factor = previous factor x reference voltage / latest measurement`

The permitted range of 0.7 to 1.3 prevents accidental extreme values. The
calibration corrects gain under the reasonable assumption of a zero point at
0 V; a true two-point characteristic would require two precise reference
voltages. Default V4 protection reads the battery before every sensor wake,
including BME680 maintenance-only wakes, and disables the divider afterward.
A due report reuses that preflight result. With protection disabled, the ADC
path is needed only for a due report and its settling can overlap acquisition.

Release builds compile sensor serial logging out and do not initialize UART.
For hardware diagnosis, add `-DSENSOR_DIAGNOSTIC_LOGGING=1` to the desired
sensor environment's `build_flags`; remove it again for battery-life testing.


## Performance changes and verification

See [power management](../POWER_MANAGEMENT.md) for default operating modes,
energy tradeoffs and validation limits. Reproducible host and browser checks
are in [tests/README.md](../tests/README.md); physical acceptance checks are in
[HARDWARE_TESTPLAN.md](../HARDWARE_TESTPLAN.md). Current package information is
in [releases](../releases/README.md).

## Live precision sensors

TMP117 and LSM6DSOX take normal measurements at the configured interval,
such as 1 s or 10 s, with the rail off between fresh acquisition windows.
SW2 / GPIO9 starts a separate recording: 1 s TMP117 results, 200 ms MAX30102
waveform packets, or 50 ms LSM6DSOX means for five minutes followed by 100 ms
means. Pulse estimates update every second. Stopping a recording restores
the normal interval and starts durable replay. See
[operation, accuracy limits and OTA migration](../PRECISION_SENSORS.md).


## Updating a node

Install the [4.4.11 station application and matching PCB package](../releases/README.md). The normal node update preserves compatible settings. BME280/BMP280 acquisitions with repeated calibration words are rejected before compensation. A module with damaged trim or unstable power still needs hardware diagnosis.

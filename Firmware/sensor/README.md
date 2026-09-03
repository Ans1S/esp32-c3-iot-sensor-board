# W-Charger Sensor Firmware

One shared firmware codebase supports PCB V3 and V4. The only differences are
the build environment and the `hardware_profile`.

```text
pio run -e sensor_pcb_v3 -t upload   # existing test board
pio run -e sensor_pcb_v4 -t upload   # current board
```

No Wi-Fi credentials, station MAC address, radio channel or measurement
interval must be written into the source code. A new sensor scans all 13
ESP-NOW channels for the station. During the initial ten-minute pairing window,
it repeats this complete broadcast scan every ten seconds. This continues until
the sensor is actually added in the dashboard (`provisioned=true`); a station
response alone does not end discovery mode. After it is added, the sensor
stores the station MAC address and channel in NVS and normally starts each
future exchange on that saved channel. After the initial ten-minute window, an
unprovisioned sensor sleeps for five minutes between discovery attempts but
still scans all 13 channels on every attempt. A short randomized radio delay is
used only during discovery so that sensors powered together do not repeatedly
collide.

A newly flashed firmware image is identified by its ELF SHA fingerprint. On
the first start of that image, the old station assignment and initial IAQ state
are cleared even if the upload tool did not erase the NVS partition. Restarts,
deep sleep and complete power loss with the same firmware image preserve the
stored state. For each scheduled transmission, the sensor first tries the known
channel twice. A newly paired sensor retains a commissioning recovery guard: if
the saved setup-AP channel stops responding, it scans all 12 other channels in
the same wake. This guarantees recovery when the station joins the configured
home Wi-Fi network and its radio moves to that network's channel. Once a new
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
mode with broadcast traffic and complete channel scans.

| Scenario | Selected mode | Radio and wake behaviour |
|---|---|---|
| Fresh flash, erased NVS or factory reset | Discovery | All 13 channels; 10-second then 5-minute sleep |
| Deleted by the station and deletion response received | Discovery | Station identity cleared; all-channel broadcast resumes |
| Incomplete or invalid persisted assignment | Discovery | Safe fallback; energy-saving mode is rejected |
| Fully provisioned BME280 or no environmental sensor | Energy saving | Configured interval; saved-channel unicast only while communication succeeds |
| Fully provisioned BME680 | Energy saving | Five-minute BSEC ULP wake; radio only at the configured report interval |
| Saved station channel no longer responds | Recovery | Bounded recovery, except the protected setup-AP transition |

The sensor type is no longer compiled into a separate firmware image. When a
sensor is added or edited on the station, select **Automatic**, **BME280**,
**BME680** or **No environmental sensor**. The selection and BME680 temperature
correction are transferred with the next ESP-NOW response. Automatic mode
checks the Bosch chip IDs at `0x76` and `0x77`.

A normal BME280 cycle performs only these steps:

1. Enable sensor power and read the BME280 in forced mode with 1x oversampling.
2. Read battery voltage; on V4, enable the divider only for the measurement and
   wait 100 ms for its 100 kOhm / 100 nF ADC filter to settle.
3. Send a validated telemetry packet over ESP-NOW.
4. Wait briefly for the station's configuration response.
5. Write the new configuration to NVS only when its revision has changed.
6. Safely disable the sensor and ADC paths and enter deep sleep.

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

The station can logically restore a sensor to factory settings on its next
contact. The web interface allows any interval from one minute to 24 hours,
covering both short tests and energy-efficient long-term operation.

## BME680 and indoor air quality

The BME680 runs Bosch's BSEC2 algorithm and uses **Static IAQ** for a stationary
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

The I2C sensor power rail is **fully switched off on PCB V3 and V4**. The BME680
itself retains no state while unpowered. It is therefore reinitialized after
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

When BME680 is selected explicitly, the reporting interval must be five minutes
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
duration plus a conservative margin, then retries `NO_NEW_DATA` for up to
500 ms. The driver accepts only a genuinely new BSEC output and keeps sensor
power enabled for no more than five seconds in total. If the next ULP deadline
is further in the future, the board and BME680 are switched off completely and
woken exactly at that deadline; the board does not wait for five minutes with
the sensor powered. This is required because this design removes sensor power
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

## Discovery and re-adding sensors

An unconfigured sensor, or one deleted from the station, announces itself every
ten seconds for ten minutes. To avoid operating the sensor heater and ADC
unnecessarily, its measurement snapshot is refreshed only every five minutes.
After ten minutes, it enters deep sleep for five minutes between discovery
attempts. When it is added successfully, the selected BME680 starts with a
fresh learning state directly in ULP mode. The sensor stores its provisioning
status persistently in NVS. The station already keeps it in the configured list
during radio confirmation, so delayed pairing packets from before confirmation
cannot mark it as a new device again.

## Extending the drivers

`environmental_sensor` is only the shared facade. Measurements use the neutral
`EnvironmentalReading` type, while BME280, BME680 and BSEC persistence each
have separate header and CPP files. A future I2C sensor therefore needs a new
driver, a new value in `EnvironmentalSensorType` and an entry in `beginType()`;
radio, ADC, deep sleep and configuration storage remain unchanged.

## Calibrating battery voltage

`analogReadMilliVolts()` already uses the ESP32-C3 ADC calibration data. A
multimeter reference value can additionally be entered for each sensor. The
station uses it to calculate a one-point gain factor that corrects external
voltage-divider tolerance and the remaining measurement error:

`new factor = previous factor x reference voltage / latest measurement`

The permitted range of 0.7 to 1.3 prevents accidental extreme values. The
calibration corrects gain under the reasonable assumption of a zero point at
0 V; a true two-point characteristic would require two precise reference
voltages. The ADC and voltage-divider paths remain enabled only for the few
milliseconds required by a measurement.

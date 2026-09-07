# Firmware Architecture

## System model

The W-Charger station is the continuously available control point. During
initial setup or network recovery it runs a protected setup access point; after
the first successful home Wi-Fi connection it normally operates as a Wi-Fi
station. ESP-NOW remains active alongside the local web interface and the
optional ThingSpeak uploader. Environmental sensor nodes wake briefly,
measure, exchange one telemetry/configuration pair with the station and return
to deep sleep. LSM6DSOX nodes instead acquire motion continuously while provisioned.

```text
PCB V3/V4 sensor
  Measure -> telemetry + applied configuration revision
          -> ESP-NOW -> W-Charger station -> immediate configuration response
                                      -> deferred flash/history + cloud queues
          <- ESP-NOW <- interval, channel, station MAC, sensor/ADC calibration
  Read configuration from RTC cache; write NVS only on change
  -> disable loads -> deep sleep
```

`shared/include/lil_protocol.h` defines telemetry/configuration;
`shared/include/ota_protocol.h` defines signed updates for both projects.
Protocol version 5 packets contain a magic value, protocol version, message
type, payload length, sequence number and CRC32. Unknown, malformed or damaged
packets are discarded. Any incompatible packet-layout change requires a new
protocol version.

## Responsibilities

### Station

- `config_store`: versioned station and sensor data in NVS
- `wifi_service`: setup AP plus station mode, reconnect, captive DNS and mDNS
- `web_portal`: setup wizard, responsive dashboard, sensor provisioning,
  history, channel profiles and account management
- `sensor_registry`: MAC-based device management, configuration revisions and
  a persistent rolling 24-hour history in LittleFS
- `espnow_gateway`: short radio callbacks, packet validation and response-first
  configuration replies followed by deferred persistence
- `thingspeak_service`: a separate FreeRTOS task for blocking HTTPS requests

New sensor MAC addresses are initially registered as unprovisioned devices.
Their telemetry is visible in the dashboard but is never uploaded as normal
cloud data. Only the sensor wizard sets the provisioning state and optionally
enables uploads. A pending provisioning acknowledgement prevents delayed
pairing packets from moving a newly configured sensor back into the discovery
list.

The station stores up to 19 sensors and six reusable ThingSpeak channel
profiles. Each profile contains a channel ID and optional read and write keys.
The profile assignment and field mapping are stored per sensor on the station.
Temperature, humidity, pressure, Static IAQ, gas resistance and battery voltage
can be mapped independently to ThingSpeak fields 1 through 8. Multiple sensors
can therefore share one channel and its credentials while using different
fields. Unsupported, stale or invalid measurements are not uploaded. Uploads
use only the write key; the read key is reserved for private-channel access.

Cloud status does more than inspect the HTTP status. An upload is successful
only when ThingSpeak returns a positive entry ID. ESP-NOW remains active during
cloud requests. When the RAM-only cloud queue is full, the oldest cloud job is
dropped in a controlled way and counted in the dashboard rather than blocking
the radio callback.

Telemetry is registered in RAM before the response is built. The station then
waits only for completion of that response transmission and delegates latest
telemetry, configuration and history writes to a low-priority persistence
task. The 64-entry queue drops its oldest event on overload and exposes a
separate persistence-drop counter. It never performs a synchronous fallback on
the radio task. Storage takes a separate mutex; radio registration performs no
flash operations or history allocation. Slot generations invalidate queued
samples after deletion/reuse. An immediate station power loss can still lose
an acknowledged sample that has not reached flash.

At intervals of at least 60 seconds, the station keeps a 24-hour ring of
40-byte V4 samples in LittleFS. Existing 23-byte V3 environmental samples are
migrated. Reception time determines the bucket independently of flash latency;
raw-fallback IAQ is excluded. Normal restart preserves this history; factory
reset or full flash erase removes it. Subminute intervals use a bounded
900-entry RAM ring and avoid per-report flash persistence; restart clears it.


### Sensor

- `hardware_profile`: only PCB V3/V4 pins, polarities and battery scaling
- `power_controller`: sensor and ADC power paths
- `environmental_sensor`: shared, extensible I2C sensor facade
- `bme280_driver`: BME280 in energy-efficient forced mode
- `bme680_driver`: Bosch BSEC2 Static IAQ with the 3.3 V ULP profile, a
  continuous five-minute algorithm schedule, timer Light-sleep during the
  forced heater/conversion wait, deep sleep between measurements and
  persistent learning state; the BSEC operating mode never changes
- `bsec_state_store`: CRC-protected, wear-conscious IAQ state checkpoints
- `adc_reader`: filtered and calibrated millivolt measurement
- `espnow_transport`: saved channel first, followed by same-cycle channel
  recovery when that channel is stale
- `sensor_config_store`: write NVS only when data actually changes
- `sleep_controller`: defined power-down and timer deep sleep

PCB V3 uses its field-validated battery scaling factor of 1.67. PCB V4 adds a
GPIO6-controlled battery divider and an active-low GPIO10 PMOS sensor rail with
a nominal divider factor of 1.667. Its 100 kOhm ADC series resistor and 100 nF
filter capacitor produce an approximately 16 ms charging time constant with the
divider, so the firmware waits 100 ms after enabling the path before sampling.
That settling period starts before the environmental conversion and is skipped
entirely on BME680 maintenance wakes where no report is due.

Unprovisioned sensors advertise every ten seconds for the first ten minutes and
scan all 13 channels during each discovery wake. Afterwards, they wake every
five minutes but still scan all 13 channels, so commissioning never depends on
the current rotating subset. Discovery alone uses a short random radio delay to
prevent sensors powered together from repeatedly colliding.

Configured sensors try the saved channel twice. A commissioning recovery guard
remains active after a sensor is added: if the setup-AP channel stops
responding, the sensor scans all 12 alternatives in the same wake and stores
the channel returned by the station after it joins the home Wi-Fi network.
After that transition succeeds, stale-channel recovery is limited to three
rotating channels once each per scheduled report. The recovery cursor survives
in RTC RAM, so later reports continue the scan without permanently extending
the active radio window. A completely failed configured exchange does not
create an extra retry wake; the next attempt follows the configured report
interval. A one-time sub-second deep-sleep phase offset staggers sensors that
started together, and both ESP32-C3 PCB targets run at 80 MHz.

A failed unicast send callback skips the application-response timeout because
no station response can follow a failed MAC transmission. Conversely, a
successful unicast MAC acknowledgement without a configuration response proves
that the saved channel is still correct, so it does not trigger an expensive
channel scan. The station requests adaptive base power with conservative RSSI
hysteresis; per-attempt escalation and maximum-power recovery remain unchanged.

The sensor selects energy-saving mode only when NVS contains a complete
assignment (`provisioned`, valid station MAC and valid channel). An erased installation,
local factory reset or `provisioned=false` response clears the station identity
and selects discovery mode. Consequently an incomplete or interrupted setup
cannot accidentally use the configured sleep and unicast path.

Protocol V5 includes the sensor's current operating mode in telemetry:
discovery, energy saving or channel recovery. The station exposes this value in
`/api/status` and renders it in both the captive setup wizard and the normal
dashboard. Unknown is reserved for a stored station entry without a current V5
telemetry packet.

BME680 nodes still wake every five minutes to maintain BSEC ULP timing, but
ESP-NOW and ThingSpeak follow the configured reporting interval. The logical
report clock includes time spent awake as well as deep-sleep time, preventing a
few seconds of processing from postponing a due report by another five-minute
BME680 cycle.

A reset command remains pending on the station until the sensor acknowledges
the corresponding configuration revision during a later contact.

## Persistence and failure behaviour

- Wi-Fi and ThingSpeak credentials are stored in station NVS, never in source
  files.
- Sensor interval, I2C sensor type, temperature offset, battery correction
  factor, station MAC address, radio channel and provisioning state are stored
  in sensor NVS. A validated RTC copy avoids reopening NVS after ordinary deep
  sleep and is invalidated by a new firmware SHA or factory reset.
- Boot count, sequence, report-attempt clock and reset acknowledgement
  are retained in RTC RAM across deep sleep.
- BSEC state and continuous algorithm time remain in RTC RAM. A checkpoint is
  written to NVS when accuracy improves and then no more than every six hours.
- A firmware ELF SHA change invalidates the RTC configuration cache while
  retaining pairing and compatible calibration in NVS. Explicit factory reset
  clears assignment; ordinary firmware updates do not.
- The cloud upload queue exists only in RAM. A station restart can therefore
  discard measurements that were queued but not uploaded.

## Security boundaries of this release

- ThingSpeak uses TLS with certificate verification.
- The setup AP uses WPA2 with the initial password `W-Charger-Setup`; the web
  interface allows it to be changed later.
- State-changing web requests require a random CSRF token.
- The HTTP web interface supports an optional password. A per-device salted
  SHA-256 digest is stored in NVS; a successful login creates a random HttpOnly
  session cookie that expires on station restart. Failed logins are rate
  limited.
- At the explicit product-design request, ThingSpeak API keys are returned in
  plain text in the local authenticated interface. With password protection
  enabled, read-only configuration, status, history and ThingSpeak APIs also
  require authentication. Without a password, local access deliberately
  remains open and the dashboard displays a warning.
- CRC32 detects transmission damage but does not authenticate a sensor.
- Automatic discovery of new sensor MAC addresses remains open for setup.

Before deployment to untrusted users or networks, the design still needs a
time-limited, physically confirmed pairing mode, individual ESP-NOW keys,
replay protection across restarts and HTTPS for local administration. The
optional HTTP login is not transport encryption and does not prevent active
interception on the same network.

## Firmware updates

Signed sensor firmware packages are staged in a dedicated station partition
and pulled over ESP-NOW at the next sensor contact. The transfer writes the
inactive application slot, checkpoints a verified prefix for resume and uses
bootloader rollback until startup and station communication have succeeded.
The Arduino default early OTA acceptance is explicitly deferred. Existing V5
telemetry remains compatible; OTA uses an additional message type.

See [OTA.md](OTA.md) for the complete wire/storage behavior, signing-key
bootstrap, USB partition migration, limits and physical acceptance tests.

## Motion and OTA

`lsm6dsox_driver` configures the I2C accelerometer/gyroscope and consumes its
104 Hz FIFO. Only motion telemetry uses the extended V5 suffix. Environmental
nodes retain the original V5 wire length. See [motion acquisition](LSM6DSOX.md)
for interval summaries, scaling and FIFO limits.

The sensor `ota_client` pulls signed firmware from the station `ota_service`.
The inactive slot receives buffered sector writes with durable resume
checkpoints. A pending boot must establish station contact before rollback is
cancelled. OTA state and partitions are independent of measurement history.
See [OTA](OTA.md) for trust, partition setup, compatibility and recovery, and
[power management](POWER_MANAGEMENT.md) for the bounded overhead and sleep paths.

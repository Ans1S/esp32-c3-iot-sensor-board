# Firmware Architecture

## System model

The W-Charger station is the continuously available control point. During
initial setup or network recovery it runs a protected setup access point; after
the first successful home Wi-Fi connection it normally operates as a Wi-Fi
station. ESP-NOW remains active alongside the local web interface and the
optional ThingSpeak uploader. Battery-powered sensor nodes wake briefly,
measure, exchange one telemetry/configuration pair with the station and return
to deep sleep.

```text
PCB V3/V4 sensor
  Measure -> telemetry + applied configuration revision
          -> ESP-NOW -> W-Charger station -> RAM queue -> HTTPS -> ThingSpeak
          <- ESP-NOW <- interval, channel, station MAC, sensor/ADC calibration
  Write NVS only on change -> disable loads -> deep sleep
```

`shared/lil_protocol.h` is the only interface shared by both firmware projects.
Protocol version 4 packets contain a magic value, protocol version, message
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
- `espnow_gateway`: short radio callbacks, packet validation, queueing and
  configuration responses
- `thingspeak_service`: a separate FreeRTOS task for blocking HTTPS requests

New sensor MAC addresses are initially registered as unprovisioned devices.
Their telemetry is visible in the dashboard but is never uploaded as normal
cloud data. Only the sensor wizard sets the provisioning state and optionally
enables uploads. A pending provisioning acknowledgement prevents delayed
pairing packets from moving a newly configured sensor back into the discovery
list.

The station stores up to 16 sensors and six reusable ThingSpeak channel
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

For every sensor, the station keeps 48 fixed 30-minute history buckets in
LittleFS. Each bucket stores the latest valid sample received during that
window. This history survives a normal restart but is intentionally removed by
a full flash erase or factory reset.

### Sensor

- `hardware_profile`: only PCB V3/V4 pins, polarities and battery scaling
- `power_controller`: sensor and ADC power paths
- `environmental_sensor`: shared, extensible I2C sensor facade
- `bme280_driver`: BME280 in energy-efficient forced mode
- `bme680_driver`: Bosch BSEC2 Static IAQ with the 3.3 V ULP profile, a
  continuous five-minute algorithm schedule, deep sleep between measurements
  and persistent learning state; the BSEC operating mode never changes
- `bsec_state_store`: CRC-protected, wear-conscious IAQ state checkpoints
- `adc_reader`: filtered and calibrated millivolt measurement
- `espnow_transport`: saved channel first, followed by same-cycle channel
  recovery when that channel is stale
- `sensor_config_store`: write NVS only when data actually changes
- `sleep_controller`: defined power-down and timer deep sleep

PCB V3 uses its field-validated battery scaling factor of 1.67. PCB V4 adds a
GPIO6-controlled battery divider and an active-low GPIO10 PMOS sensor rail with
a nominal divider factor of 1.667.

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

The sensor selects energy-saving mode only when NVS contains a complete
assignment (`provisioned`, valid station MAC and valid channel). A fresh flash,
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
  in sensor NVS.
- Boot count, sequence, failure count, report clock and reset acknowledgement
  are retained in RTC RAM across deep sleep.
- BSEC state and continuous algorithm time remain in RTC RAM. A checkpoint is
  written to NVS when accuracy improves and then no more than every six hours.
- A firmware ELF SHA change clears the old station assignment and IAQ startup
  state once; ordinary restarts and power cycles with the same image preserve
  stored configuration.
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

This release distributes configuration updates at the next sensor wake-up. It
does not transfer firmware binaries over ESP-NOW. The ESP32-S3 and ESP32-C3
partition layouts already provide two OTA application slots. A future OTA flow
should require a signed manifest, chunked resumable transfer, SHA-256
verification, version and hardware checks, and rollback after a failed
self-test. It must not be presented as end-user OTA without signature
verification.

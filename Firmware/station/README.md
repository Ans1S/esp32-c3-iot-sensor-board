# W-Charger Station Firmware

Firmware **4.4.1 / 40401** uses the existing green light/dark design and a
compact overview of sensor names and current readings. Click or tap anywhere
on a sensor card for details, or anywhere on a measurement tile for that
measurement's chart. **Arrange** saves order, visibility, spacing and column
count on the station. Hidden sensors keep measuring and saving data.

Details show measurement/time selectors first and disclose display/export and
device settings separately. A touch selection stays pinned; previous/next
sample buttons support precise inspection. Automatic orange security/cloud
setup notices no longer appear. Real upload/storage faults and active battery
protection remain visible. Settings and Firmware stay available in details.
See [precision operation](../PRECISION_SENSORS.md) and
[capacity and validation](../RESOURCE_AND_UX_REVIEW.md).

The station runs on the existing Seeed XIAO ESP32-S3. It keeps Wi-Fi and
ESP-NOW active at the same time, so the web interface remains available and
sensors are not locked out while a cloud upload is in progress.

## First start

1. Run `pio run -e station_s3 -t upload`. Normal uploads preserve saved settings,
   sensor names and local archives. For an intentional clean installation only,
   `pio run -e station_s3_factory_reset -t upload` erases the complete flash.
2. Connect to the `W-Charger-XXXXXX` Wi-Fi network. The initial Wi-Fi password
   is `W-Charger-Setup`.
3. The captive portal normally opens automatically. If it does not, open
   `http://192.168.4.1/`. No login is required during the unconfigured first
   start.
4. Follow the setup wizard to configure the measurement interval, optional
   ThingSpeak integration, sensors and home Wi-Fi. An optional but recommended
   website password can be set at the end.
5. Confirm the summary. The station saves the configuration and restarts.

A serial monitor is not required for setup. The setup Wi-Fi remains active
until the first successful connection to the home network and then switches
off automatically. It starts again only after a fresh installation or factory
reset. On the home network, the station is available at
`http://w-charger.local/` when the client supports mDNS. Depending on the
router, `http://w-charger/` may work as well.

After setup, the responsive dashboard separates newly discovered sensors from
configured sensors. A new device appears automatically and must be confirmed
once in the three-step wizard. The wizard assigns its name and interval,
selects the attached I2C sensor (automatic, BME280, BME680, LSM6DSOX or disabled),
optionally adjusts BME680 temperature compensation, selects a saved ThingSpeak
channel profile and maps measurements to fields 1 through 8. Only then does
the sensor appear in the regular dashboard with its last contact, readings,
battery voltage, radio quality and switchable local history. Available
intervals start at one second and include a custom value of up to 24 hours.
Normal history uses bounded persistent pages for environmental, battery,
TMP117 and LSM6DSOX samples. Retention follows the interval and available shared
space rather than a fixed 24-hour limit. Short intervals are batched; a sudden
power loss can lose the latest uncommitted batch (normally up to about 35 s,
longer if storage is failing). The detail view reports saved/buffered points.
Manual SW2 sessions remain independent. MAX30102 remains SW2-only.
Each sensor report also carries its actual operating mode. The captive setup
wizard, pending-sensor list and configured dashboard cards display **Setup
mode**, **Energy-saving mode**, **Channel recovery** or **Mode unknown**. The
badge represents the last mode reported by the sensor rather than an estimate
from the station configuration.
The station setup wizard is available only after a fresh installation or
factory reset. During normal operation, changes are made directly under
**Settings**.

## Modules

- `config_store`: persistent station and sensor configuration in NVS, plus
  local measurement history in LittleFS
- `wifi_service`: one-time setup access point, home Wi-Fi reconnect, DNS and
  mDNS
- `web_portal`: dashboard, sensor wizard, channel profiles, Wi-Fi and reset
- `espnow_gateway`: validated and versioned telemetry/configuration protocol
- `sensor_registry`: sensors identified by MAC address instead of hard-coded
  IDs
- `thingspeak_service`: HTTPS upload, account management and automatic creation
  of private channels

The ESP-NOW gateway builds its configuration response entirely from RAM and
sends it before writing telemetry, history or a newly discovered sensor entry
to flash. A separate low-priority task performs those writes after the send
callback. This keeps filesystem latency out of the battery sensor's response
window; if the persistence queue is unexpectedly full, the station performs a
synchronous fallback only after the response transmission has completed.

The station also selects each sensor's normal transmit-power request from its
received RSSI using hysteresis. PCB V4 starts at 11 dBm when the first link has
adequate margin and can step down to 8.5 dBm after eight strong reports; PCB V3
does not go below 11 dBm. Two weak reports raise power, an RSSI at or below
-82 dBm raises it immediately, and normal requested power is capped at 15 dBm.
The sensor still adds 3 dBm on its second known-channel attempt and uses the
ESP32 API maximum for bounded channel recovery. The current base request is
available as `sensorTxPowerDbm` in `/api/status`.

Up to six ThingSpeak channel profiles can be stored centrally and reused by
multiple sensors. Each profile contains a name, the channel ID and optional
read and write API keys. Selecting a profile gives a sensor its channel ID and
write key without requiring the keys to be entered again. Alternatively, the
sensor wizard can use an existing channel or automatically create a new private
channel with the user API key.

ThingSpeak field mapping is freely configurable for each sensor. Temperature,
humidity, pressure, Static IAQ, gas resistance and battery voltage can each be
assigned to `Field 1` through `Field 8` or set to `Do not upload`. Multiple
sensors can use the same channel and write key while occupying different
fields. The interface prevents duplicate field assignments within one sensor.
An automatically created channel initially maps temperature, humidity,
pressure, IAQ, battery voltage and gas resistance to fields 1 through 6. The
BME680 wizard adopts this mapping by default, but it remains editable before
saving.

For BME680 sensors, the dashboard shows BSEC accuracy from "background
learning" through "high" next to the IAQ value, as well as raw gas resistance
in kOhm. BSEC operates exclusively in its energy-efficient ULP mode from the
first start and measures internally every five minutes. ESP-NOW and ThingSpeak
strictly follow the selected, longer reporting interval. The initial ULP
learning notice remains visible only while Accuracy is 0. It disappears after
Accuracy first reaches 1; the compact accuracy badge then remains visible as
BSEC continues improving to 2 and 3 during normal operation. No fixed-duration
progress bar is shown because the required air-condition changes cannot be
predicted honestly.

New, unconfigured sensors appear automatically in the overview; the additional
add button is now only a shortcut to that section. Configured sensors can be
deleted from the local station. As long as a deleted sensor continues to
transmit, it reappears as a new device on its next contact, as expected.

The editing wizard also provides transparent BME680 commissioning, an explicit
reset of the IAQ learning state and optional battery-voltage calibration using
a multimeter reference. Independently of any open browser, the station stores
an adaptive persistent history for normal measurements. Environmental/battery
rows retain compact precision; TMP117 and IMU rows retain their floats. A
selected sensor loads the complete retained data in bounded API pages. No
browser needs to remain open for collection. The view reports saved/buffered
points, gaps, interval and estimated capacity, with a raw full-history CSV.
Short-interval batches have a bounded uncommitted tail on sudden power loss.

The channel ID and read/write API keys can be entered in the captive portal and
later in the station settings. The read key is used only to read private
channels, while the station uses the write key for uploads. The user API key is
required only when W-Charger should create a new ThingSpeak channel itself. API
keys are shown in plain text in the local web interface only on explicit
request and are restored to the fields after a restart. Cloud upload remains
disabled until it is explicitly enabled in the sensor wizard.

When a user API key is stored, the settings page can load ThingSpeak channels
through the explicit Load channels action. Channels can be created, renamed, edited with their description,
fields, tags, metadata, visibility and location, cleared or deleted. Direct
channel and feed URLs are displayed. A newly detected sensor can select one of
these channels in the wizard; its channel ID and write key are then copied into
a local sensor profile automatically.

The status panel reports **Connected** only after ThingSpeak confirms a write
with a real entry ID. An incorrect write key, HTTP/TLS error or absence of valid
measurements produces a specific error message instead. The local adaptive
history is stored exclusively on the station and therefore does not depend on
ThingSpeak or one browser's storage. A complete firmware upload with flash
erasure and a factory reset intentionally delete this history.

## Security model of this release

- Home Wi-Fi and ThingSpeak credentials are stored only in NVS.
- The setup Wi-Fi uses WPA2 with the initial password `W-Charger-Setup`.
- The local web interface can be protected during initial setup or later in
  Settings with a password of at least eight characters. This protects the
  dashboard, configuration, history, Wi-Fi scan, all write operations and all
  ThingSpeak-related APIs.
- The website password is stored as a device-bound SHA-256 hash rather than in
  plain text. Successful logins receive a random HttpOnly session cookie that
  expires after an absolute 30-minute lifetime or at the next station restart.
  Repeated failed attempts trigger a device-wide temporary login block, and
  sign-out requires the authenticated CSRF-protected action.
- At the explicit request of the product design, ThingSpeak API keys remain
  visible in the authenticated web interface. Without an enabled website
  password, any device on the local network can therefore read these keys and
  change settings; the dashboard displays a clear warning.
- ThingSpeak requests use HTTPS with a verified DigiCert root certificate.
- ESP-NOW packets include a magic value, protocol version, fixed lengths and
  CRC32.

CRC32 protects against transmission errors but does not provide cryptographic
authentication. Individual ESP-NOW keys and a physically confirmed pairing
flow are therefore planned as the next hardening step.

The local web interface is still served over HTTP. Password protection prevents
unintended access by other participants on the home network, but it does not
replace transport encryption against active interception on that network.

The 8 MB OTA station layout reserves two 2.25 MiB application slots, a
1.625 MiB measurement filesystem and a separate 1.75 MiB firmware staging
partition. Install this layout once by USB. The **Firmware updates** page
accepts signed sensor packages, queues one node at a time, and displays actual
reported software/build versions and confirmed transfer progress. See
[the OTA guide](../OTA.md) for signing keys, installation and recovery tests.

## Performance changes and verification

The optional ThingSpeak upload worker starts with the first valid enabled
upload. An empty worker waits for a notification; an installation which has
never enabled uploads does not allocate its 10 KiB task stack. The bounded
cloud scheduler and initialization mutexes still occupy memory. Account
management remains available separately.

The status API reports 64-bit monotonic uptime/contact age and a `memory`
object containing internal free/minimum/largest-block bytes and PSRAM
free/largest-block bytes. These support on-device load and fragmentation
checks. The [resource review](../RESOURCE_AND_UX_REVIEW.md) distinguishes
current retention limits from the proposed adaptive storage policy.

See [power management](../POWER_MANAGEMENT.md) for default operating modes,
energy tradeoffs and validation limits. Reproducible host and browser checks
are in [tests/README.md](../tests/README.md); physical acceptance checks are in
[HARDWARE_TESTPLAN.md](../HARDWARE_TESTPLAN.md). Current package information is
in [releases](../releases/README.md).

## Live precision sensors

TMP117 and LSM6DSOX take fresh normal measurements at the configured interval
with the external rail off between windows. MAX30102 starts through SW2 only.
SW2 / GPIO9 starts/stops a separate fast recording: 1 s TMP117 values, 200 ms
optical waveform packets, or 50 ms IMU means for five minutes and 100 ms afterward.
Pulse estimates update once per second. Saved recordings load as complete,
stable snapshots; full-session CSV retains the original samples. See [operation, accuracy limits and OTA migration](../PRECISION_SENSORS.md).

## Remembered sensor identities

Sensor names and optional motion references are also saved by MAC address,
independently of the 19 active pairing slots. Up to 64 identities are remembered.
Removing a node or resetting it into discovery mode retains its last name for
re-pairing on this station. This also works with existing node firmware because
names are stored on the station. Existing paired names are migrated at startup.
A full station factory reset or full-flash erase clears remembered identities;
names erased by an older station upload cannot be recovered from the node.
Normal USB and OTA firmware updates preserve this data and the recording archive.


## Current release and publication checks

Firmware **4.4.1 / 40401** includes bounded sensor reinitialization, long-uptime
corrections, adaptive normal history and the compact configurable dashboard.
See [the complete change inventory](../CHANGELOG.md),
[release artifacts](../releases/README.md) and
[publication review](../PUBLICATION_REVIEW.md). Build diagnostics use relative
paths; installation keys and device data must stay local. Software tests do not
replace physical accuracy, rail-voltage and sleep-current acceptance.

## Updating an installed station without resetting it

Run `python Firmware/upload.py station --application-only` from the repository
root, or `pio run -e station_s3_update -t upload` from this directory. This target
writes only the application at `0x10000`. Use it on the compatible recording
partition layout already used by releases 4.3.x. It leaves the NVS, Wi-Fi,
sensor identities, partition table, bootloader and filesystem outside the write.
The settings schema is unchanged; dashboard preferences use a separate key.
For a bare/new board use the normal installation target to install the layout.
Only the explicitly named factory-reset target erases all flash.

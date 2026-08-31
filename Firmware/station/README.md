# W-Charger Station Firmware

The station runs on the existing Seeed XIAO ESP32-S3. It keeps Wi-Fi and
ESP-NOW active at the same time, so the web interface remains available and
sensors are not locked out while a cloud upload is in progress.

## First start

1. Run `pio run -e station_s3 -t upload`. The station upload intentionally
   erases the local NVS configuration, so every upload starts as a fresh
   installation.
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
selects the attached I2C sensor (automatic, BME280, BME680 or disabled),
optionally adjusts BME680 temperature compensation, selects a saved ThingSpeak
channel profile and maps measurements to fields 1 through 8. Only then does
the sensor appear in the regular dashboard with its last contact, readings,
battery voltage, radio quality and switchable local history. Available
intervals start at one minute and include a custom value of up to 24 hours.
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
a rolling 24-hour history for every sensor. Its 48 fixed 30-minute windows each
contain the latest measurement received during that window: temperature,
humidity, pressure, IAQ and its accuracy, gas resistance and battery voltage.
A phone or another browser loads the complete data through the local station
API; keeping or reloading a page is not required for collection. The history
survives a normal station restart. A drop-down switches the labelled plot
between every quantity supported by the sensor, with time, value range and unit
shown on the axes. Truly missing 30-minute windows are not bridged by an
invented connecting line.

The channel ID and read/write API keys can be entered in the captive portal and
later in the station settings. The read key is used only to read private
channels, while the station uses the write key for uploads. The user API key is
required only when W-Charger should create a new ThingSpeak channel itself. API
keys are shown in plain text in the local web interface only on explicit
request and are restored to the fields after a restart. Cloud upload remains
disabled until it is explicitly enabled in the sensor wizard.

When a user API key is stored, the settings page loads ThingSpeak channels
automatically. Channels can be created, renamed, edited with their description,
fields, tags, metadata, visibility and location, cleared or deleted. Direct
channel and feed URLs are displayed. A newly detected sensor can select one of
these channels in the wizard; its channel ID and write key are then copied into
a local sensor profile automatically.

The status panel reports **Connected** only after ThingSpeak confirms a write
with a real entry ID. An incorrect write key, HTTP/TLS error or absence of valid
measurements produces a specific error message instead. The local 24-hour
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

The selected 8 MB partition layout has two OTA-capable application slots of
approximately 3.19 MiB each. This provides headroom but does not yet implement
firmware upload. The current release distributes only settings to the sensors;
the secure binary OTA flow is scoped separately in `../ARCHITECTURE.md`.

# Sensor firmware updates through the station

## Implemented scope

The station accepts a signed `.ota` package for a configured PCB V3 or V4
sensor. At its next successful scheduled ESP-NOW contact the sensor polls for
an update and temporarily remains awake to download it. The sensor does not
join a Wi-Fi access point. Sensor drivers and the existing automatic/manual
sensor-type selection remain compiled into the sensor application.

The dashboard links to **Firmware updates** (`/updates`). It shows reported
PCB revision, software version and ELF build identifier. Each sensor card also
links directly to its update selection. Version reports are persisted on the
station and describe the last reported image, not proof that an offline node
is currently reachable. Legacy nodes are displayed without an invented version.

The update page uses the dashboard navigation and saved color theme. Before an
upload it compares the selected package with the sensor's PCB, protocol and
release. During delivery it shows the active stage, confirmed bytes and the
specific station or sensor failure reason, including low battery, interrupted
transport, flash write, digest, image validation and trial-boot rollback errors.

One package and one node update are managed at a time. Finish or cancel the
current job before uploading the next package. The registry supports 19 nodes;
normal telemetry continues while the selected node requests firmware blocks.
New sensor types may still require station changes to add their selectors and
measurements; OTA does not itself make the existing BME-specific UI generic.

## Initial installation

1. Back up station settings/history as needed. The existing station USB upload
   hook deliberately erases flash. The new station partition layout MUST first
   be installed by USB; do not send its application to an old layout.
2. Initialize the installation signing key once, before building either target:

   ```sh
   python -m pip install cryptography
   python Firmware/ota_package.py init
   ```

   This creates `.tools/ota/signing.key` and
   `Firmware/shared/include/ota_public_key.h`. Both are excluded from Git.
   Do not rerun initialization to replace an existing installation identity. Back up the private key securely: losing
   it prevents signing updates trusted by already installed nodes. The key
   never goes to the browser, station, sensor, or release package. Copy the
   public header and retain the original signing key for subsequent builds.
   Without a public header firmware builds fail closed for OTA verification.
3. Build and USB-install `station_s3`, `sensor_pcb_v3` and/or `sensor_pcb_v4`
   with that same public header. Use the existing `Firmware/upload.py` helper
   or PlatformIO. Base images include bootloader, partition table and app.
4. Configure the station and add the sensors. Wait for one successful contact
   after provisioning: the node then reports its version and OTA support.
5. Verify the actual flash size, slot table and supply under load on each PCB
   revision before accepting the physical installation.

Existing sensor configuration and BSEC calibration survive a firmware change.
USB application replacement no longer implies factory reset. Use the explicit
reset workflow when a fresh pairing is intended. A firmware change still
invalidates the RTC configuration cache. The NVS schema is unchanged, allowing
rollback to the preceding OTA-capable release without destructive migration.

## Produce a future update

Increase both `kRelease` (monotonically increasing integer) and
`WCH_FIRMWARE_VERSION` in `shared/include/ota_protocol.h`, then build the sensor
environment. The packager reads the hardware and version from an embedded
identity in the application, rather than trusting the filename or command-line
labels:

```sh
cd Firmware/sensor
pio run -e sensor_pcb_v3 -e sensor_pcb_v4
cd ../..
python Firmware/ota_package.py pack --image Firmware/sensor/.pio/build/sensor_pcb_v3/firmware.bin --output Firmware/releases/sensor-v3.ota
python Firmware/ota_package.py pack --image Firmware/sensor/.pio/build/sensor_pcb_v4/firmware.bin --output Firmware/releases/sensor-v4.ota
python Firmware/ota_package.py verify --image Firmware/releases/sensor-v4.ota
```

Upload the `.ota` file, NOT `firmware.factory.bin`, `bootloader.bin` or a raw
application binary. An update must be newer than the node's reported release;
uploading the same release is deliberately rejected. Current release information
is in [releases](releases/README.md). Binary packages remain local because they
include separately licensed BSEC2 components.

The 160-byte manifest binds PCB revision, protocol, release, version, exact
application length and SHA-256 to an ECDSA P-256 signature. Both station and
sensor verify the signature. The maximum application size is 0x1e0000 bytes
(1.875 MiB). Normal OTA never changes the bootloader or partition table.

## Transfer and recovery

- The station stages one complete package in a dedicated 2 MiB raw partition.
  It invalidates the old job before overwriting storage and publishes the new
  job only after signature and complete-image hash verification. Upload data
  and normal measurement history do not share a partition. A partially uploaded
  package is never offered. The browser can close after upload confirmation.
- ESP-NOW packets remain under 250 bytes, with 192-byte data blocks. Each pull
  request carries a session nonce and sequence; replies must come from the
  configured station and match both. The node retries five times and bounds
  each update session to ten minutes plus the current bounded operation.
- The node writes the inactive OTA slot. Every 16 KiB it checkpoints a CRC-
  protected offset and prefix digest in NVS. On a later contact it verifies the
  saved prefix, erases the unconfirmed tail and resumes at the checkpoint. A
  bad checkpoint causes a clean download; the running image is never erased.
  Up to one checkpoint interval may be retransmitted after interruption.
- Starting with 4.1.0, OTA requires at least 2800 mV on PCB V4 and
  3350 mV on PCB V3. Equality is accepted; only lower voltage postpones the
  update. The same limit is rechecked during transfer. Firmware 4.0.0 still
  enforces its compiled 3700 mV limit until replaced; changing the station or
  offered package cannot change the running client's check. To migrate a
  blocked 4.0.0 node, restore sufficient battery voltage or install via USB.
  Starting with 4.1.1, a ten-minute session budget replaces the previous
  three-minute cutoff; a timeout is reported explicitly and subsequent
  sessions follow the configured contact interval. Flash writes are buffered
  in 4 KiB sectors, while durable checkpoints remain spaced 16 KiB apart.
  These improvements take effect only after the new client is installed.
- After full-image SHA-256 verification and ESP-IDF image validation, the node
  selects the new slot and restarts. The Arduino automatic early acceptance
  hook is overridden. A 60-second trial-boot timer bounds verification; the
  next normal report is forced during a trial boot. Successful configuration
  loading and a valid station configuration response establish the minimum
  local/communication self-test before cancelling rollback. A missing external
  environmental sensor does not by itself fail this test.
- An unconfirmed trial image is not deliberately sent to deep sleep. Failure
  or reset rolls back through the OTA bootloader. The old image reports the
  attempted release as restored on its next contact. Hardware acceptance must
  verify that the installed bootloader actually implements this behavior.
- UI success requires a matching release report after the node accepted its
  boot. 100% transfer is not success. Cancellation prevents subsequent block
  delivery; an already initiated boot selection/restart cannot be undone by
  the cancel button.

Environmental telemetry/configuration retain the original protocol V5 layouts.
LSM6DSOX telemetry adds a motion suffix accepted by the current station. OTA uses an additional
message type; old nodes continue normal operation but cannot update themselves.
The station persists the package, selected MAC, target version, job state and
reported node versions. Transfer progress is reconstructed from the node after
a station restart rather than writing station NVS for every radio block.

## Security and practical limits

Signed packages authenticate firmware; CRC detects radio corruption. ESP-NOW
control and telemetry still use the project's existing unencrypted,
unauthenticated link. Nonces reject stale replies but do not cryptographically
authenticate status reports or prevent a nearby attacker from disrupting radio
traffic. This implementation does not claim secure boot, physical tamper
protection, eFuse anti-rollback or authenticated pairing. Downgrades through the
normal station upload path are rejected; bootloader recovery remains allowed.

The web upload follows the existing optional login, origin and CSRF checks.
Use website protection if local network participants must not schedule updates.
No key is fetched from an uploaded package, and no unsigned fallback exists.

## Verification

Host tests exercise the actual OTA client with flash/RTOS substitutes: full
download, resume, corrupted prefix, bad signature/hardware, low battery, write
failure, image rejection and trial-boot acceptance/rollback. Radio tests cover
stale session replies, a wrong sender and a missing send callback. Python
tests use real SHA-256 and ECDSA, wrong keys, truncation, tampering and duplicate
firmware identities. These are not substitutes for physical power-loss tests.

```sh
python Firmware/tests/test_ota_package.py
python Firmware/tests/run_tests.py --cxx /path/to/zig
```

Required hardware acceptance on BOTH revisions:

1. Read chip/flash identification and the live partition table, and confirm
   GPIO10 has the correct polarity for the installed revision.
2. Install a higher release and observe version/build confirmation, retained
   pairing, settings and BSEC state, followed by normal deep sleep.
3. Remove node power during erase, a block write, a checkpoint, boot selection
   and trial boot. Verify resume or rollback and retained old firmware.
4. Restart the station during upload, transfer and boot confirmation. Verify
   incomplete uploads are rejected and complete jobs recover.
5. Force an early crash, hang and failed station contact in a test release;
   verify rollback with the actual bootloader.
6. Test weak RF, channel changes, cancellation, low battery under sustained
   transmit/flash load, a corrupted package and a V3/V4 mismatch.
7. Test 19 paired nodes while updating one node; verify no unexpected history
   loss and inspect radio/persistence counters and memory headroom.

## Normal operation in 4.1.3

Temporary serial stage logging is removed. The five-second trial-boot reply
retry window, 60-second boot guard, rollback, packet compatibility and buffered
writes remain active. Normal optional sensor logging defaults to disabled.
The reply extension applies only to a pending trial boot. Environmental nodes
return to power-gated deep sleep; see [power management](POWER_MANAGEMENT.md).

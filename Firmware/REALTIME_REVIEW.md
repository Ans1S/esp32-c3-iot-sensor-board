# Real-time and long-uptime review

This review targets the maintained station, sensor and shared firmware. It
examines timing, bounded queues, radio completion, I2C and FIFO failure paths,
recording durability, memory and timer boundaries. It does not establish that
every execution path is free of defects or prove hard real-time deadlines.

## Findings addressed

- A station ESP-NOW send whose completion callback never arrives previously
  left the gateway permanently waiting. A pending send is now monitored even
  without incoming packets; after ten seconds the station restarts rather than
  overlap an uncertain earlier send. A restart can lose uncommitted telemetry.
- When a live acquisition task fails to release its I2C/rail ownership, the
  sensor now restarts after a ten-second cooperative wait, so startup can
  recheck protection. The limit assumes the queued-capture drain itself returns.
- An invalid or low voltage measured during an OTA transfer is now returned to
  the caller. PCB V4 aborts the transfer and enters its existing 24-hour battery
  protection sleep without a further multi-retry OTA status exchange. Normal
  live acquisition remains paused until this decision is made.
- BSEC output counts are checked against their actual array capacity before
  indexing. Extreme future scheduling timestamps are rounded without signed
  integer overflow. Invalid BSEC output falls back to the existing raw path.
- Archive workspace timing uses a 64-bit monotonic clock, preventing the
  temporary reserve from reappearing after a 32-bit millisecond wrap. A
  scheduled station restart is no longer lost when its deadline equals zero.
- MAC address parsing rejects oversized, invalid or unterminated input without
  modifying the destination on failure. Restored OTA node identities must have
  a terminated version string and valid board/protocol fields.
- Recordings stop and retain their committed prefix if their timestamps exceed
  the storage format's 32-bit range; elapsed time saturates instead of wrapping.

The previous dashboard performance changes remain in the working tree: larger
opt-in history/archive pages and a shorter yield between durably acknowledged
recording transfers. Existing 128-row API clients remain compatible.

## Platform constraints and remaining work

ESP-NOW callbacks run in the high-priority Wi-Fi task. Espressif recommends
short callbacks, handing work to another task, and waiting for completion before
the next send. The station uses a bounded receive queue and separate tasks for
radio, telemetry persistence and recording persistence. See the
[ESP-NOW documentation for ESP32-C3](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32c3/api-reference/network/esp_now.html).

ESP32-C3 flash operations can disable cache and defer interrupts. Moving flash
work to a lower-priority task therefore does not prove an uninterrupted sensor
sampling deadline; do not enable flash auto-suspend without confirming the
specific flash device supports it. See the
[ESP32-C3 flash concurrency documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32c3/api-reference/peripherals/spi_flash/spi_flash_concurrency.html).
Watchdogs detect certain blocked interrupt and task conditions, but do not prove
individual measurement deadlines or application progress. See the
[ESP32-S3 watchdog documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/api-reference/system/wdts.html).

ThingSpeak account operations still run synchronously in the web handler and
the HTTP client reads responses into a `String`. Large or slow remote responses
can delay web requests and consume heap; the radio and sensor acquisition use
separate tasks. A bounded, deadline-aware response reader and asynchronous
account requests require separate integration tests against identity and
chunked HTTP responses before changing this behavior.

The sensor waits for its acquisition task to release the rail before battery
protection, OTA or sleep, and restarts if cooperative waiting exceeds ten
seconds. This does not bound a filesystem call that itself never returns or
prove that restart/power gating meets a physical deadline; those cases still
require on-device watchdog and fault-injection tests.

## Validation boundaries

Production host tests exercise timer wraps, acquisition transitions, FIFO gaps,
storage failures, OTA interruption and battery protection with substitutes.
Build and sanitizer results are software evidence only. Before treating the
system as dependable for long deployments, run a physical soak and fault test
on both PCB revisions and the station, including simultaneous flash writes,
radio traffic, browser use, Wi-Fi loss, sensor/I2C faults and low-voltage
events. Record sample intervals and missing samples, task/heap low-water marks,
reset reasons, rail voltage and battery current. No board was flashed or
physically timed as part of this review.

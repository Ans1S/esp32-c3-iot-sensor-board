# Firmware regression checks

Run from the repository root after resolving/building the sensor dependencies:

```sh
python Firmware/tests/run_tests.py --cxx /path/to/clang++
python Firmware/tests/test_ota_package.py
python Firmware/tests/test_library_patches.py
```

`g++`, Clang and `zig c++` are supported; pass the path to `zig.exe` directly.
The runner enables assertions explicitly, builds into a temporary directory and
executes production C++ sources with deterministic hardware/RTOS substitutes.
It does not flash a board or access a cloud account. The patch test downloads
five immutable upstream files and verifies their SHA-256 hashes. `--cache DIR`
uses already downloaded originals for an offline run.

Coverage:

- Actual OTA client transfer, checkpoint resume, corrupted prefix, rejected
  manifest/hardware, low battery, flash failure and trial-boot rollback using
  deterministic flash substitutes. This suite substitutes the hash primitive;
  the package tests independently use real SHA-256 and ECDSA.
- OTA radio response correlation, wrong sender, retry limits and a missing
  send callback; signed-package corruption, wrong keys and truncation.

- Failed first report followed by BSEC maintenance wakes; stale, missing and
  future schedules; rounding; optional discovery backoff.
- Every possible 12-sample ADC sum from 0 to 39600 at seven calibration factors,
  plus every representable float in the accepted calibration range at the
  maximum sum, for both divider ratios. Maximum allowed difference is 1 mV.
- CRC reference vectors, a finalized telemetry packet and corruption of every
  packet byte. The host uses the portable CRC path; inspect the target ELF to
  verify the ESP32 ROM call. Hardware ROM execution is not emulated.
- Late application replies, replies after a failed MAC ACK, replies during
  retry backoff, missing/late send callbacks, and all 13 discovery channels.
- Radio registration injected into every simulated flash write, command
  acknowledgement during persistence, slot deletion/reuse, timestamp buckets,
  interval changes and stale raw-fallback IAQ.
- Per-channel upload limits, fairness, ordered bounded retries, queue overflow,
  expiry and `millis()` wraparound.
- Measurement/fetch budgets, wait clamping and logical time across both normal
  and early-error deep sleep.
- The actual patched Bosch C driver with acknowledged but permanently stuck
  mode status and missing measurement fields.
- All three build patches against pristine pinned upstream sources, repeated
  application, and incompatible input rejection.

The RTOS substitutes detect lock-order violations and deterministic event
interleavings. They do not replace on-device concurrent-load, RF, TLS, GPIO,
BSEC algorithm or current measurements. See `../HARDWARE_TESTPLAN.md`.

## Motion support

The C++ runner also tests the LSM6DSOX register driver against a FIFO/I2C
substitute, signed scaling, interval peaks, overflow and fault handling.
Registry tests cover a 900-slot fast ring without per-report flash writes and
migration from the persisted V3 environmental history layout.

Run `node Firmware/tests/test_motion_dashboard.cjs` with Playwright available.
`PLAYWRIGHT_MODULE` can identify an installed module; `PLAYWRIGHT_CHANNEL` can
select an installed browser channel. Set `QA_SCREENSHOT_DIR` to save desktop
and mobile captures. The test uses the actual embedded HTML with mock APIs;
it never connects to a physical station or cloud account.

## OTA performance and status

The C++ OTA suite asserts sector-sized writes, a final partial sector, resume
with an unflushed RAM tail, completion beyond three minutes and explicit
timeout/resume at ten minutes. It reports flash API call counts without
claiming a hardware timing benchmark. Run
`node Firmware/tests/test_ota_status_page.cjs` with the same Playwright module
to check overlapping refresh suppression, sensor-status caching and recovery
from a network error without overwriting upload results.

The transport suite also covers legacy environmental packet lengths and bounded
trial-boot retries. OTA tests cover both board voltage thresholds at equality
and one millivolt below, as well as boot-confirmation failure handling.
Run `python Firmware/tests/test_web_scripts.py` to syntax-check embedded scripts
with Node.js. Python package tests require `cryptography`.

# Main, radio and OTA audit evidence

This directory is a historical pre-release 4.4.3 audit snapshot. Current
4.4.4 package/build identities are recorded in the [release evidence](../2026-11-07-release/README.md).
Do not overwrite these retained measurements when rerunning tools.

Read [the report](../../FLOW_TRANSFER_REVIEW.md) for conclusions, costs and
physical limits. Before means the preserved previous local MCU audit result,
including its uncommitted changes. It is not clean HEAD. The local archive and
all actual before/after outputs are identified by `artifact-manifest.json`.

| Files | Scope |
| --- | --- |
| `before-functions.csv`, `after-functions.csv` | Own source definitions/locations, including embedded scripts and test helpers |
| `before-modules.csv`, `after-modules.csv` | Source hashes, roles, definition counts and parse errors |
| `module-review.csv` | Changed-block source review versus retained earlier audit scope for unchanged modules |
| Six `*-symbols.csv` files | Emitted own symbols, static instructions/call sites and placement for both architectures |
| Six `*-elf.json` files | Actual ELF hashes and section sizes |
| `function-comparison.csv` | Selected native before/after bodies and local frames, including independently parsed ELF prologues |
| `assembly-excerpts.md` | Actual RISC-V copy/CRC/stack and Xtensa frame excerpts with costs and limits |
| `stack-frames.csv` | All scoped auxiliary GCC frame estimates for V4/S3; excludes caller depth/runtime watermarks |
| `toolchain.json` | Unchanged compiler/architecture/clock/optimization settings |
| `artifact-manifest.json` | Reference archive and retained ELF/BIN/map/partition hashes |
| `measurements.json` | Operation counts, two registration benchmark experiments/ranges, costs and validation evidence |

Complete disassemblies, maps, binaries, compiler probes and raw logs remain in
ignored `.tools/firmware-flow-20261107/`. Public evidence uses relative paths.
It contains no installation key or device data. Test timings and operation
counts come from host substitutes; native sizes/frames come from actual GCC
builds. Neither proves real OTA throughput, RF latency, energy or task watermarks.

## Reproducing current checks

Run from the repository root after resolving/building target dependencies:

```sh
python Firmware/tests/run_tests.py --cxx PATH_TO_COMPILER --sanitize
python Firmware/tests/run_tests.py --cxx PATH_TO_COMPILER --suite registry --benchmark
python Firmware/tests/test_ota_package.py
python Firmware/tests/test_station_upload.py
```

The full runner contains 30 C++ suites. The registry benchmark reports nine
runs of 20,000 registrations per environmental type in addition to the earlier
ring-read benchmark. These are batch averages with fixture locks. No timing
threshold is asserted. Zig executables are accepted as the host compiler.

## Reproducing the preserved production comparison

Set `CXX` to an absolute host compiler path, then execute this Python example
from the repository root. It verifies the retained archive hash, extracts to a
temporary directory, and overlays only the comparison fixtures and test runner.
The before production sources remain unchanged. `FLOW_OPTIMIZED=0` changes
the expected operation counts and omits API checks unavailable before this pass.
Both versions execute their actual production main/transport/drivers/registry/
OTA sources with the same substitutes and compiler options.

```python
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import zipfile

root = Path.cwd()
reference = root / ".tools/firmware-flow-20261107/before-worktree.zip"
assert hashlib.sha256(reference.read_bytes()).hexdigest() == "0dda4582c787e161bf7b447c28d868f6e956256660ac4ead258ab361a89c412f"
fixtures = [
    "run_tests.py", "protocol_counter.h", "counted_protocol.cpp",
    "battery_boot_test.cpp", "boot_stubs/boot_test_platform.h",
    "transport_test.cpp", "stubs/freertos/queue.h", "stubs/Arduino.h",
    "live_stubs/environmental_sensor.h", "live_acquisition_test.cpp",
    "stubs/Wire.h", "lsm6dsox_test.cpp", "registry_test.cpp",
    "ota_stubs/ota_test_platform.h", "ota_client_test.cpp", "ota_station_test.cpp",
]
fixtures += [str(path.relative_to(root / "Firmware/tests"))
             for path in (root / "Firmware/tests/ota_station_stubs").rglob("*.h")]
with tempfile.TemporaryDirectory(prefix="flow-reference-") as temporary:
    baseline = Path(temporary)
    with zipfile.ZipFile(reference) as source:
        source.extractall(baseline)
    for name in fixtures:
        destination = baseline / "Firmware/tests" / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root / "Firmware/tests" / name, destination)
    counter = baseline / "Firmware/tests/protocol_counter.h"
    counter.write_text("#define FLOW_OPTIMIZED 0\n" + counter.read_text())
    suites = [argument for name in ("battery_boot", "transport", "live_acquisition",
                                    "lsm6dsox", "registry", "ota_client", "ota_station")
              for argument in ("--suite", name)]
    for tree in (baseline, root):
        subprocess.run([sys.executable, str(tree / "Firmware/tests/run_tests.py"),
                        "--cxx", os.environ["CXX"], "--benchmark", *suites], check=True)
```

The comparison checks both the 70,000-byte example and maximum 1,310,720-byte
OTA slot, all returned data/CRCs, retry/offset/failure behavior and the complete
sensor-written image. The station fixture substitutes SHA/signature primitives,
JSON, partitions, NVS and RTOS operations; separate package tests use real
SHA-256/ECDSA. It does not claim cryptographic, serializer or SDK concurrency
coverage from the transfer fixture alone. Host benchmark values vary with load;
both recorded experiments, including reversed order, remain in measurements.

## Reproducing machine-code evidence

Use matching installed RISC-V or Xtensa binutils with each retained ELF:

```sh
python Firmware/tools/elf_metrics.py PATH_TO_ELF --tool-prefix PATH_TO_MATCHING_BINUTILS_PREFIX --target sensor_pcb_v4 --phase after --output OUTPUT_DIRECTORY
```

Code-size comparisons require identical build flags/tool versions. GCC frame
probes use the actual compilation database plus `-fstack-usage` with separate
object outputs. Baseline probes prepend archived headers and compile archived
production sources. Selected frames were also verified in linked prologues,
including split RISC-V adjustments/early return paths and hexadecimal Xtensa
`entry` operands. V3 has native prologue evidence; auxiliary GCC probes cover
V4/S3. Static instruction counts are not executed instruction counts or WCET.

The pinned parser versions appear in `Firmware/tools/firmware_inventory.py`.
Current inventory generation uses that tool directly; before generation parses
the archived source bytes with the same normalization, rather than Git HEAD.

No command above enumerates or accesses devices, uploads firmware, replaces a
key, signs an installation package or publishes anything.

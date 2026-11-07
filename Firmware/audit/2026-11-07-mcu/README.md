# Follow-up MCU audit evidence

This directory is a historical pre-release 4.4.3 audit snapshot. Current
4.4.4 package/build identities are recorded in the [release evidence](../2026-11-07-release/README.md).
Do not overwrite these retained measurements when rerunning tools.

Read [the report](../../MCU_OPTIMIZATION_REVIEW.md) for conclusions, costs and
hardware limits. Before means the previous local audit result, including its
uncommitted changes. It is not identical to clean HEAD. The local reference
archive is retained and identified by `artifact-manifest.json`.

| Files | Scope |
| --- | --- |
| `before-functions.csv`, `after-functions.csv` | Own source definitions and locations, including embedded scripts and test helpers |
| `before-modules.csv`, `after-modules.csv` | Module hashes, roles, definition counts and parse errors |
| `module-review.csv` | Changed-block/source review versus retained earlier review of unchanged modules |
| Six `*-symbols.csv` files | Static emitted project symbols, instruction/call-site counts and placement on both architectures |
| Six `*-elf.json` files | Actual target ELF hashes and section sizes |
| `function-comparison.csv` | Selected native before/after costs; GCC local frames and independently parsed ELF prologues |
| `stack-frames.csv` | All scoped GCC auxiliary frame estimates for V4/S3; not runtime stack watermarks |
| `toolchain.json` | Unchanged real compiler/architecture/clock/optimizer settings |
| `artifact-manifest.json` | Before-worktree archive and retained before/after build output hashes |
| `measurements.json` | Host experiment repetitions/ranges, validation and operation counts with domain limits |

Complete disassemblies, maps, binaries, compiler probes and diagnostic logs
remain under ignored `.tools/firmware-mcu-20261107/`. Public artifacts use
relative paths and contain no installation key or device data.

## Reproducing current host results

Run from the repository root with a host compiler; Zig executables are accepted:

```sh
python Firmware/tests/run_tests.py --cxx PATH_TO_COMPILER --sanitize
python Firmware/tests/run_tests.py --cxx PATH_TO_COMPILER --suite registry --benchmark
```

The benchmark reports nine runs x 1,000 reads for each ring type. It measures
the actual production registry on the host, not S3/PSRAM latency. No timing
threshold is asserted. Normal functional assertions still run.

## Reproducing the preserved source comparison

Set `CXX` to an absolute host compiler path, then run this Python example from
the repository root. It uses only the retained, hash-identified reference ZIP,
copies the new comparison fixtures into a temporary reference tree and changes
only their expected operation counts. Production reference sources are untouched.

```python
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import zipfile

root = Path.cwd()
reference = root / ".tools/firmware-mcu-20261107/before-worktree.zip"
with tempfile.TemporaryDirectory(prefix="mcu-reference-") as temporary:
    baseline = Path(temporary)
    with zipfile.ZipFile(reference) as source:
        source.extractall(baseline)
    for name in ("run_tests.py", "bme280_test.cpp", "registry_test.cpp",
                 "imu_peak_test.cpp", "protocol_validation_test.cpp",
                 "optical_fifo_test.cpp", "math_counter.h", "precision_stubs/Wire.h"):
        shutil.copyfile(root / "Firmware/tests" / name, baseline / "Firmware/tests" / name)
    for name, flag in (("imu_peak_test.cpp", "EXPECT_DEFERRED_GYRO_SQRT"),
                       ("optical_fifo_test.cpp", "EXPECT_OPTICAL_BURST")):
        fixture = baseline / "Firmware/tests" / name
        fixture.write_text("#define " + flag + " 0\n" + fixture.read_text())
    suites = [argument for name in ("bme280", "imu_peak", "protocol_validation",
                                    "registry", "optical_fifo")
              for argument in ("--suite", name)]
    for tree in (baseline, root):
        subprocess.run([sys.executable, str(tree / "Firmware/tests/run_tests.py"),
                        "--cxx", os.environ["CXX"], "--benchmark", *suites], check=True)
```

The original source formula remains in the numerical regression; the before
archive separately verifies that it agrees with real reference production code.
Raw benchmark values vary with host load. Both recorded experiments are
included; the second reverses before/after order.

## Reproducing machine-code evidence

Use the matching installed RISC-V or Xtensa binutils with each retained ELF:

```sh
python Firmware/tools/elf_metrics.py PATH_TO_ELF --tool-prefix PATH_TO_MATCHING_BINUTILS_PREFIX --target sensor_pcb_v4 --phase after --output OUTPUT_DIRECTORY
```

Build flags and tool versions must match for a code-size comparison. For GCC
frame estimates, use the actual compilation database with `-fstack-usage` and
separate output objects. Baseline probes prepend reference include paths and
use reference sources. The selected frame values were also checked in actual
linked prologues, including hexadecimal Xtensa frame immediates.

The inventories use the pinned parser versions documented in
`Firmware/tools/firmware_inventory.py`. Current-source generation uses that
tool directly. Before-source generation reads the same archived file names and
bytes instead of Git HEAD, keeping identical parsing and normalization.

No command above enumerates or accesses hardware, uploads, signs an installation
package, changes a key, or publishes anything.

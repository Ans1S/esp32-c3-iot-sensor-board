# Audit evidence - 2026-11-07

This directory is a historical pre-release 4.4.3 audit snapshot. Current
4.4.4 package/build identities are recorded in the [release evidence](../2026-11-07-release/README.md).
Do not overwrite these retained measurements when rerunning tools.

Read [the audit report](../../EFFICIENCY_AUDIT.md) for conclusions and limits.
All paths in these public artifacts are repository-relative. Build outputs,
complete disassemblies, benchmark executables and raw diagnostic logs stay in
the ignored local audit directory; their retained hashes identify the reference.

| Files | Meaning |
| --- | --- |
| `before-functions.csv`, `after-functions.csv` | Every inventoried own definition, including callbacks/templates and test helpers; line ranges are source ranges, not execution coverage |
| `before-modules.csv`, `after-modules.csv` | Source/configuration modules, definition counts and raw byte hashes; Git baseline and worktree line endings can differ |
| `module-review.csv` | Component-level source-review/evidence scope; test helpers have a narrower review status |
| Six `*-symbols.csv` files | Actual emitted project entries for V3, V4 and S3 before/after; counts are static and indirect calls may be unresolved |
| Six `*-elf.json` files | ELF hashes and section sizes/addresses from each architecture's binutils |
| `stack-frames.csv` | Auxiliary GCC local-frame estimates for V4 and S3; not runtime call-chain high-water marks |
| `toolchain.json` | Tool versions, effective language standard, CPU/architecture and relevant compiler settings |
| `artifact-manifest.json` | Immutable baseline commit/archive hash and local before/after ELF/BIN/map/partition file hashes |
| `measurements.json` | Repeated host comparison and validation observations, with their measurement domains |

Recreate inventories in an environment containing the versions documented by
`Firmware/tools/firmware_inventory.py`:

```sh
python Firmware/tools/firmware_inventory.py --revision ff7afa30032221ec961f2bc939c1dbba4c90a0c2 --prefix before --output .tools/audit-reproduction/first
python Firmware/tools/firmware_inventory.py --prefix after --output .tools/audit-reproduction/first
```

After building a target, supply the **matching** installed binutils prefix:

```sh
python Firmware/tools/elf_metrics.py PATH_TO_ELF --tool-prefix PATH_TO_TOOLCHAIN/bin/riscv32-esp-elf- --target sensor_pcb_v4 --phase after --output .tools/audit-reproduction/first
python Firmware/tools/elf_metrics.py PATH_TO_ELF --tool-prefix PATH_TO_TOOLCHAIN/bin/xtensa-esp32s3-elf- --target station_s3 --phase after --output .tools/audit-reproduction/first
```

The before ELF must come from the saved baseline, not from a later rebuild
whose source/SDK/options differ. For auxiliary frame reports, compile each
application source using its PlatformIO compilation-database arguments plus
`-fstack-usage`, with separate object/output paths. Baseline probes use archived
baseline sources/headers; code-generation settings remain the same. Ancillary
dependency-output paths can be omitted. Cross-check relevant frames against
the actual ELF prologue, and account for nested/SDK calls separately.

Host comparisons and sanitizer commands are in
[the test guide](../../tests/README.md#efficiency-audit-regressions-2026-11-07).
The centered-mean reference is included in the numerical test. To repeat the
flush baseline, build the same new flush test and I/O counter against archived
`ff7afa3` production sources with `EXPECT_HISTORY_FLUSH_SEEKS=0`; the unchanged
ABI permits the same fixture. Repeat it against current production code with
the default expected count of two. Use identical compiler flags and row data.

The following Python example performs that isolated comparison without
overwriting the worktree. Run from the repository root with `CXX` set to an
absolute host compiler path (a Zig executable is also accepted). The copied
test changes only its expected seek count; the baseline production sources
come directly from the immutable Git archive. Each runner creates and removes
its own temporary file area. Repeat the two runner calls for repeated counts.

```python
import io
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import zipfile

root = Path.cwd()
reference = "ff7afa30032221ec961f2bc939c1dbba4c90a0c2"
archive = subprocess.check_output(["git", "archive", "--format=zip", reference, "Firmware"])
with tempfile.TemporaryDirectory(prefix="history-flush-reference-") as temporary:
    baseline = Path(temporary)
    with zipfile.ZipFile(io.BytesIO(archive)) as source:
        source.extractall(baseline)
    for name in ("run_tests.py", "history_flush_test.cpp", "file_seek_counter.h"):
        shutil.copyfile(root / "Firmware/tests" / name, baseline / "Firmware/tests" / name)
    fixture = baseline / "Firmware/tests/history_flush_test.cpp"
    fixture.write_text("#define EXPECT_HISTORY_FLUSH_SEEKS 0\n" + fixture.read_text())
    for tree in (baseline, root):
        subprocess.run([sys.executable, str(tree / "Firmware/tests/run_tests.py"),
                        "--cxx", os.environ["CXX"], "--suite", "history_flush"], check=True)
```

Neither inventory generation, ELF analysis nor host tests operate a device.

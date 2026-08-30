#!/usr/bin/env python3
"""Build or upload one of the maintained W-Charger firmware targets."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


FIRMWARE_DIR = Path(__file__).resolve().parent
TARGETS = {
    "station": (FIRMWARE_DIR / "station", "station_s3", "ESP32-S3 station"),
    "sensor-v3": (
        FIRMWARE_DIR / "sensor",
        "sensor_pcb_v3",
        "ESP32-C3 sensor, PCB V3",
    ),
    "sensor-v4": (
        FIRMWARE_DIR / "sensor",
        "sensor_pcb_v4",
        "ESP32-C3 sensor, PCB V4",
    ),
}


def find_platformio() -> Path | None:
    """Find PlatformIO Core from PATH or its standard virtual environment."""
    executable_names = (
        ("pio.exe", "platformio.exe")
        if sys.platform == "win32"
        else ("pio", "platformio")
    )

    for name in executable_names:
        candidate = shutil.which(name)
        if candidate:
            return Path(candidate)

    script_dir = Path(sys.executable).resolve().parent
    candidates = [script_dir / name for name in executable_names]
    if sys.platform == "win32":
        candidates.extend(
            Path.home() / ".platformio" / "penv" / "Scripts" / name
            for name in executable_names
        )
    else:
        candidates.extend(
            Path.home() / ".platformio" / "penv" / "bin" / name
            for name in executable_names
        )

    return next((candidate for candidate in candidates if candidate.is_file()), None)


def choose_target() -> str:
    print("Select the board to flash:\n")
    choices = list(TARGETS)
    for index, key in enumerate(choices, start=1):
        print(f"  {index}. {TARGETS[key][2]} ({key})")

    while True:
        try:
            answer = input("\nChoice [1-3]: ").strip()
        except EOFError as exc:
            raise SystemExit("Pass station, sensor-v3 or sensor-v4 on the command line.") from exc
        if answer in {"1", "2", "3"}:
            return choices[int(answer) - 1]
        print("Please enter 1, 2 or 3.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build or upload the maintained W-Charger firmware."
    )
    parser.add_argument("target", nargs="?", choices=TARGETS)
    parser.add_argument(
        "--build-only",
        action="store_true",
        help="compile without writing to a connected board",
    )
    parser.add_argument(
        "--port",
        metavar="PORT",
        help="serial port if PlatformIO cannot choose one automatically (for example COM5)",
    )
    args = parser.parse_args()

    target = args.target or choose_target()
    project_dir, environment, description = TARGETS[target]
    platformio = find_platformio()
    if platformio is None:
        print(
            "PlatformIO Core was not found. Install the PlatformIO IDE extension "
            "in VS Code or PlatformIO Core, then run this command again.",
            file=sys.stderr,
        )
        return 2

    action = "Building" if args.build_only else "Uploading"
    print(f"\n{action}: {description}", flush=True)
    if target == "station" and not args.build_only:
        print(
            "Note: a station upload erases its saved settings and local history.",
            flush=True,
        )

    command = [
        str(platformio),
        "run",
        "--project-dir",
        str(project_dir),
        "-e",
        environment,
    ]
    if not args.build_only:
        command.extend(("-t", "upload"))
    if args.port:
        command.extend(("--upload-port", args.port))

    try:
        return subprocess.run(command, check=False).returncode
    except KeyboardInterrupt:
        print("\nCancelled.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())

"""Keep the pinned Adafruit BME280 driver asleep after initialization.

Adafruit BME280 2.3.0 starts a high-oversampling normal-mode measurement and
then waits a fixed 100 ms in init(). This project immediately replaces that
configuration with its validated x1 forced-mode profile. Initializing directly
into sleep mode avoids the redundant conversion and delay without weakening the
existing reset/calibration readiness checks or forced-measurement status poll.
"""

from pathlib import Path

Import("env")


def _project_path(value: str, project_dir: Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else project_dir / path


project_dir = Path(env.subst("$PROJECT_DIR")).resolve()
libdeps_dir = _project_path(env.subst("$PROJECT_LIBDEPS_DIR"), project_dir)
pio_env = env.subst("$PIOENV")
library_dir = libdeps_dir / pio_env / "Adafruit BME280 Library"
source_path = library_dir / "Adafruit_BME280.cpp"
properties_path = library_dir / "library.properties"
if not source_path.is_file() or not properties_path.is_file():
    raise RuntimeError(
        "Adafruit BME280 source was not found. Let PlatformIO resolve lib_deps "
        "and run the build again."
    )

properties = properties_path.read_text(encoding="utf-8")
if "version=2.3.0" not in properties:
    raise RuntimeError(
        "Unsupported Adafruit BME280 version; review the low-power patch "
        "before changing the pinned dependency."
    )

source = source_path.read_text(encoding="utf-8")
marker = "WCHARGER_BME280_SLEEP_AFTER_INIT"
if marker not in source:
    needle = "  setSampling(); // use defaults\n\n  delay(100);\n"
    replacement = (
        "  // WCHARGER_BME280_SLEEP_AFTER_INIT\n"
        "  // The application selects its forced-mode sampling profile next.\n"
        "  // Do not start a redundant x16 normal-mode conversion here.\n"
        "  setSampling(MODE_SLEEP, SAMPLING_X1, SAMPLING_X1, SAMPLING_X1,\n"
        "              FILTER_OFF, STANDBY_MS_0_5);\n"
    )
    if needle not in source:
        raise RuntimeError(
            "Adafruit BME280 init layout changed; refusing to apply an "
            "unverified timing patch."
        )
    source_path.write_text(
        source.replace(needle, replacement, 1), encoding="utf-8"
    )
    print("Patched BME280 low-power initialization: %s" % source_path)
else:
    print("BME280 low-power initialization already patched: %s" % source_path)

# Bound the reset readiness loop even when the bus acknowledges but status sticks.
source = source_path.read_text(encoding="utf-8")
if "WCHARGER_BME280_CALIBRATION_DEADLINE" not in source:
    needle = "  while (isReadingCalibration())\n    delay(10);"
    if needle not in source:
        raise RuntimeError("BME280 calibration readiness layout changed")
    source = source.replace(needle,
        "  // WCHARGER_BME280_CALIBRATION_DEADLINE\n"
        "  const uint32_t calibrationStarted = millis();\n"
        "  while (isReadingCalibration()) {\n"
        "    if (millis() - calibrationStarted >= 100U) return false;\n"
        "    delay(10);\n"
        "  }", 1)
    source_path.write_text(source, encoding="utf-8")

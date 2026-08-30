"""Apply the power-cycled BME680 timing fix to the pinned Bosch BSEC2 library.

BSEC2 2.1.5 starts a forced measurement in ``Bsec2::run()`` and immediately
calls ``fetchData()``. That loses the first measurement when the sensor rail
is switched off after every call. Wait for the complete measurement and allow
a bounded 500-ms NO_NEW_DATA retry window before the application-level
five-second acquisition deadline expires.

The Bosch library contains proprietary precompiled BSEC code and remains a
normal PlatformIO dependency.  This small, version-checked source patch keeps
that dependency intact and makes clean builds reproducible without committing
the downloaded library into this repository.
"""

from pathlib import Path

Import("env")


def _project_path(value: str, project_dir: Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else project_dir / path


project_dir = Path(env.subst("$PROJECT_DIR")).resolve()
libdeps_dir = _project_path(env.subst("$PROJECT_LIBDEPS_DIR"), project_dir)
pio_env = env.subst("$PIOENV")

candidates = [libdeps_dir / pio_env / "bsec2" / "src" / "bsec2.cpp"]
candidates.extend(libdeps_dir.glob("*/bsec2/src/bsec2.cpp"))
source_path = next((path for path in candidates if path.is_file()), None)
if source_path is None:
    raise RuntimeError(
        "BSEC2 source was not found. Let PlatformIO resolve lib_deps and "
        "run the build again."
    )

source = source_path.read_text(encoding="utf-8")
header_path = source_path.with_name("bsec2.h")
if not header_path.is_file():
    raise RuntimeError("BSEC2 header was not found next to bsec2.cpp.")
header = header_path.read_text(encoding="utf-8")
if "2.1.5" not in source:
    raise RuntimeError(
        "Unsupported Bosch BSEC2 source version; update the timing patch "
        "before changing the pinned dependency."
    )

marker = "WCHARGER_BSEC2_MEASUREMENT_WAIT"
changed = False
header_changed = False
if marker not in source:
    needle = (
        "        if (sensor.checkStatus() == BME68X_ERROR)\n"
        "            return false;\n"
        "\n"
        "        if (bmeConf.trigger_measurement && bmeConf.op_mode != "
        "BME68X_SLEEP_MODE)\n"
    )
    replacement = (
        "        if (sensor.checkStatus() == BME68X_ERROR)\n"
        "            return false;\n"
        "\n"
        "        /* WCHARGER_BSEC2_MEASUREMENT_WAIT\n"
        "         *\n"
        "         * BSEC2 triggers forced mode but its generic adapter does not "
        "wait for\n"
        "         * the TPH/heater cycle before fetchData().  A power-cycled "
        "BME680 has\n"
        "         * no previous field to read, so this would return "
        "NO_NEW_DATA on every\n"
        "         * wake-up.  Wait exactly for the configured sensor duration "
        "plus a\n"
        "         * conservative bus/clock margin before fetching the field.\n"
        "         */\n"
        "#if defined(ARDUINO)\n"
        "        if (bmeConf.trigger_measurement &&\n"
        "            bmeConf.op_mode == BME68X_FORCED_MODE) {\n"
        "            const uint32_t measurementUs =\n"
        "                sensor.getMeasDur(BME68X_FORCED_MODE) +\n"
        "                static_cast<uint32_t>(\n"
        "                    sensor.getHeaterConfiguration().heatr_dur) *\n"
        "                    1000UL;\n"
        "            if (measurementUs > 0U) {\n"
        "                /* WCHARGER_BSEC2_COOPERATIVE_WAIT */\n"
        "                const uint32_t waitUs = measurementUs + 10000UL;\n"
        "                delay(waitUs / 1000UL);\n"
        "                delayMicroseconds(waitUs % 1000UL);\n"
        "            }\n"
        "        }\n"
        "#endif\n"
        "\n"
        "        if (bmeConf.trigger_measurement && bmeConf.op_mode != "
        "BME68X_SLEEP_MODE)\n"
    )
    if needle not in source:
        raise RuntimeError(
            "BSEC2 run() layout changed; refusing to apply an unverified patch."
        )

    source = source.replace(needle, replacement, 1)
    changed = True
elif "measurementUs + 2000UL" in source:
    source = source.replace(
        "measurementUs + 2000UL", "measurementUs + 10000UL", 1
    )
    changed = True
elif "measurementUs + 10000UL" not in source:
    raise RuntimeError(
        "Existing BSEC2 timing patch has an unknown margin; refusing to alter it."
    )

cooperative_marker = "WCHARGER_BSEC2_COOPERATIVE_WAIT"
if cooperative_marker not in source:
    blocking_wait = (
        "                bme68xDelayUs(measurementUs + 10000UL, nullptr);\n"
    )
    cooperative_wait = (
        "                /* WCHARGER_BSEC2_COOPERATIVE_WAIT */\n"
        "                const uint32_t waitUs = measurementUs + 10000UL;\n"
        "                delay(waitUs / 1000UL);\n"
        "                delayMicroseconds(waitUs % 1000UL);\n"
    )
    if blocking_wait not in source:
        raise RuntimeError(
            "BSEC2 measurement wait layout changed; refusing to alter it."
        )
    source = source.replace(blocking_wait, cooperative_wait, 1)
    changed = True

retry_marker = "WCHARGER_BSEC2_FETCH_RETRY"
if retry_marker not in source:
    fetch_needle = (
        "            if (sensor.fetchData())\n"
        "            {\n"
    )
    fetch_replacement = (
        "            uint8_t fetchedFields = sensor.fetchData();\n"
        "#if defined(ARDUINO)\n"
        "            /* WCHARGER_BSEC2_FETCH_RETRY: a just-completed forced "
        "cycle may\n"
        "             * briefly report NO_NEW_DATA on a power-cycled rail. */\n"
        "            for (uint8_t retry = 0;\n"
        "                 fetchedFields == 0 &&\n"
        "                 bmeConf.op_mode == BME68X_FORCED_MODE && retry < 50;\n"
        "                 ++retry) {\n"
        "                delay(10);\n"
        "                fetchedFields = sensor.fetchData();\n"
        "            }\n"
        "#endif\n"
        "            if (fetchedFields)\n"
        "            {\n"
    )
    if fetch_needle not in source:
        raise RuntimeError(
            "BSEC2 fetchData() layout changed; refusing to apply an "
            "unverified retry patch."
        )
    source = source.replace(fetch_needle, fetch_replacement, 1)
    changed = True
else:
    if "retry < 3" in source:
        source = source.replace("retry < 3", "retry < 50", 1)
        changed = True
    if "                delay(5);\n                fetchedFields" in source:
        source = source.replace(
            "                delay(5);\n                fetchedFields",
            "                delay(10);\n                fetchedFields",
            1,
        )
        changed = True
    elif "bme68xDelayUs(5000UL, nullptr);" in source:
        source = source.replace(
            "bme68xDelayUs(5000UL, nullptr);", "delay(10);", 1
        )
        changed = True

next_call_marker = "WCHARGER_BSEC2_NEXT_CALL_ACCESSOR"
if next_call_marker not in header:
    header_needle = "    int64_t getTimeMs(void);\n"
    header_replacement = (
        "    int64_t getTimeMs(void);\n\n"
        "    /* WCHARGER_BSEC2_NEXT_CALL_ACCESSOR: expose the schedule selected\n"
        "     * by bsec_sensor_control so a power-cycled node can deep-sleep\n"
        "     * until the precise next slot instead of waiting with the rail on. */\n"
        "    int64_t getNextCallNs(void) const\n"
        "    {\n"
        "        return bmeConf.next_call;\n"
        "    }\n"
    )
    if header_needle not in header:
        raise RuntimeError(
            "BSEC2 header layout changed; refusing to add next_call accessor."
        )
    header = header.replace(header_needle, header_replacement, 1)
    header_changed = True

if changed:
    source_path.write_text(source, encoding="utf-8")
if header_changed:
    header_path.write_text(header, encoding="utf-8")
if changed or header_changed:
    print("Patched BSEC2 forced-measurement timing and schedule: %s" % source_path)
else:
    print(
        "BSEC2 forced-measurement timing and schedule already patched: %s"
        % source_path
    )

"""Run actual production C++ logic with deterministic hardware/RTOS substitutes.

Requires a C++17 host compiler. --cxx also accepts a portable zig executable.
The Bosch fault test uses the dependency resolved by a sensor-v3 build.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default=os.environ.get("CXX"))
    args = parser.parse_args()
    compiler = args.cxx or shutil.which("clang++") or shutil.which("g++")
    if not compiler:
        parser.error("Pass --cxx with a host C++ compiler or zig executable")
    command = [compiler]
    if Path(compiler).stem == "zig":
        command += ["c++"]
    driver = ROOT / "sensor/.pio/libdeps/sensor_pcb_v3/BME68x Sensor library/src/bme68x"
    suites = {
        "power_pins": ["tests/power_pins_test.cpp", "sensor/src/power_controller.cpp"],
        "battery_boot": ["tests/battery_boot_test.cpp", "shared/src/lil_protocol.cpp"],
        "sensor_config": ["tests/sensor_config_test.cpp"],
        "config_store": ["tests/config_store_test.cpp", "station/src/config_store.cpp"],
        "web_input": ["tests/web_input_test.cpp"],
        "bme280": ["tests/bme280_test.cpp", "sensor/src/bme280_driver.cpp"],
        "bme680": ["tests/bme680_test.cpp", "sensor/src/bme680_driver.cpp",
                   "sensor/src/bsec_state_store.cpp", "shared/src/lil_protocol.cpp"],
        "environmental_sensor": ["tests/environmental_sensor_test.cpp", "sensor/src/environmental_sensor.cpp"],
        "motion_feedback": ["tests/motion_feedback_test.cpp"],
        "recording": ["tests/recording_test.cpp", "shared/src/recording_journal.cpp", "shared/src/lil_protocol.cpp"],
        "recording_archive": ["tests/recording_archive_test.cpp", "station/src/recording_archive.cpp", "shared/src/lil_protocol.cpp"],
        "recording_json": ["tests/recording_json_test.cpp", "shared/src/lil_protocol.cpp"],
        "recording_store": ["tests/recording_store_test.cpp", "sensor/src/recording_store.cpp", "shared/src/recording_journal.cpp", "shared/src/lil_protocol.cpp"],
        "precision_sensors": ["tests/precision_sensors_test.cpp", "sensor/src/precision_sensors.cpp"],
        "lsm6dsox": ["tests/lsm6dsox_test.cpp", "sensor/src/lsm6dsox_driver.cpp"],
        "live_acquisition": ["tests/live_acquisition_test.cpp", "sensor/src/live_acquisition.cpp"],
        "ota_client": ["tests/ota_client_test.cpp", "sensor/src/ota_client.cpp", "shared/src/lil_protocol.cpp"],
        "power_and_upload": ["tests/power_and_upload_test.cpp", "shared/src/lil_protocol.cpp"],
        "transport": ["tests/transport_test.cpp", "sensor/src/espnow_transport.cpp",
                      "shared/src/lil_protocol.cpp"],
        "registry": ["tests/registry_test.cpp", "station/src/sensor_registry.cpp"],
        "clock_budget": ["tests/clock_budget_test.cpp", "sensor/src/logical_clock.cpp",
                         "sensor/src/measurement_budget.cpp"],
        "bme68x_fault": ["tests/bme68x_fault_test.cpp", str(driver / "bme68x.c")],
    }
    with tempfile.TemporaryDirectory(prefix="wcharger-tests-") as output:
        for name, sources in suites.items():
            executable = Path(output) / (name + (".exe" if os.name == "nt" else ""))
            includes = [ROOT / directory for directory in
                        ["tests/stubs", "shared/include", "station/include", "sensor/include"]]
            includes.append(driver)
            if name == "recording_json":
                includes.append(ROOT / "station/.pio/libdeps/station_s3/ArduinoJson/src")
            if name == "sensor_config":
                includes.insert(0, ROOT / "tests/sensor_config_stubs")
            if name == "config_store":
                includes.insert(0, ROOT / "tests/station_stubs")
            if name == "bme280":
                includes.insert(0, ROOT / "tests/bme280_stubs")
            if name == "bme680":
                includes.insert(0, ROOT / "tests/bme680_stubs")
                includes.append(ROOT / "sensor/.pio/libdeps/sensor_pcb_v3/bsec2/src")
            if name == "environmental_sensor":
                includes.insert(0, ROOT / "tests/bme680_stubs")
                includes.insert(0, ROOT / "tests/environmental_stubs")
                includes.append(ROOT / "sensor/.pio/libdeps/sensor_pcb_v3/bsec2/src")
            if name == "precision_sensors":
                includes.insert(0, ROOT / "tests/precision_stubs")
            if name == "live_acquisition":
                includes.insert(0, ROOT / "tests/live_stubs")
            if name == "recording_store":
                includes.insert(0, ROOT / "tests/recording_stubs")
            if name == "ota_client":
                includes.insert(0, ROOT / "tests/ota_stubs")
            compile_command = command + ["-std=c++17", "-O2", "-UNDEBUG", "-Wall", "-Wextra"]
            if name == "environmental_sensor":
                compile_command += ["-DPCB_VERSION=4"]
            if name == "recording_archive":
                includes.insert(0, ROOT / "tests/archive_stubs")
                archive_path = Path(output) / "archive"
                archive_path.mkdir()
                compile_command += [f'-DRECORDING_MOUNT_PATH="{archive_path.as_posix()}"']
                if os.name == "nt":
                    compile_command += ["-Dfsync=_commit", "-include", "io.h"]
            if name == "battery_boot":
                includes.insert(0, ROOT / "tests/boot_stubs")
                compile_command += ["-DPCB_VERSION=4"]
            if name == "power_pins":
                includes.insert(0, ROOT / "tests/power_stubs")
                compile_command += ["-DPCB_VERSION=4"]
            for include in includes:
                compile_command += ["-I", str(include)]
            base_command = list(compile_command)
            for source in sources:
                path = ROOT / source
                if path.suffix == ".c":
                    obj = Path(output) / (path.stem + ".o")
                    c_command = [flag for flag in base_command if flag != "-std=c++17"]
                    c_command += ["-x", "c", "-std=c11", "-c", str(path), "-o", str(obj)]
                    subprocess.run(c_command, check=True)
                    compile_command.append(str(obj))
                else:
                    compile_command.append(str(path))
            compile_command += ["-o", str(executable)]
            subprocess.run(compile_command, check=True)
            subprocess.run([str(executable)], check=True, timeout=60)


if __name__ == "__main__":
    main()

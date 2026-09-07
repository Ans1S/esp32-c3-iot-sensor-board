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
        "lsm6dsox": ["tests/lsm6dsox_test.cpp", "sensor/src/lsm6dsox_driver.cpp"],
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
            if name == "ota_client":
                includes.insert(0, ROOT / "tests/ota_stubs")
            compile_command = command + ["-std=c++17", "-O2", "-UNDEBUG", "-Wall", "-Wextra"]
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

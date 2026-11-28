"""Generate deterministic flash-resident page representations for the station."""
import gzip
from pathlib import Path
import re


def page_bytes(source, page):
    # C++ translation normalizes source line endings before parsing raw strings.
    text = source.read_text(encoding="utf-8")
    matches = re.findall(rf'const char k{page}Page\[\].*?R"WCH\((.*?)\)WCH";', text, re.S)
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one {page} raw string")
    return matches[0].encode("utf-8")


def generate(source, destination, page="Dashboard"):
    compressed = gzip.compress(page_bytes(source, page), compresslevel=9, mtime=0)
    rows = [", ".join(f"0x{b:02x}" for b in compressed[i:i + 16])
            for i in range(0, len(compressed), 16)]
    output = ("#pragma once\n#include <Arduino.h>\n"
              f"static const unsigned char k{page}Gzip[] PROGMEM = {{\n  "
              + ",\n  ".join(rows) + "\n};\n")
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not destination.exists() or destination.read_text(encoding="ascii") != output:
        destination.write_text(output, encoding="ascii", newline="\n")


if "Import" in globals():
    Import("env")  # type: ignore[name-defined]  # PlatformIO provides this.
    project = Path(env.subst("$PROJECT_DIR"))  # type: ignore[name-defined]
    build = Path(env.subst("$BUILD_DIR"))  # type: ignore[name-defined]
    generate(project / "src/web_pages.cpp", build / "dashboard_gzip.h")
    generate(project / "src/web_pages.cpp", build / "setup_gzip.h", "SetupWizard")
    env.Append(CPPPATH=[str(build)])  # type: ignore[name-defined]

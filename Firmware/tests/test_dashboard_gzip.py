"""Check the generated dashboard payload and HTTP content negotiation on the host."""
import gzip
import importlib.util
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("dashboard_gzip", root / "station/dashboard_gzip.py")
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)
source = root / "station/src/web_pages.cpp"

with tempfile.TemporaryDirectory(prefix="dashboard-gzip-") as tmp:
    folder = Path(tmp)
    header = folder / "dashboard_gzip.h"
    generator.generate(source, header)
    generated = header.read_bytes()
    payload = bytes(int(value, 16) for value in re.findall(rb"0x([0-9a-f]{2})", generated))
    original = generator.dashboard_bytes(source)
    assert gzip.decompress(payload) == original
    generator.generate(source, header)
    assert header.read_bytes() == generated
    modified = folder / "web_pages.cpp"
    modified.write_text(source.read_text(encoding="utf-8").replace("W-Charger Dashboard", "Dashboard changed"), encoding="utf-8")
    generator.generate(modified, header)
    assert gzip.decompress(bytes(int(v, 16) for v in re.findall(rb"0x([0-9a-f]{2})", header.read_bytes()))) == generator.dashboard_bytes(modified)
    assert header.read_bytes() != generated
    test = folder / "negotiation.cpp"
    test.write_text(r'''
#include "accept_encoding.h"
#include <cassert>
int main() {
  using station::acceptsDashboardGzip;
  assert(!acceptsDashboardGzip(""));
  assert(!acceptsDashboardGzip("br, deflate"));
  assert(!acceptsDashboardGzip("xgzip, gzip;q=0"));
  assert(!acceptsDashboardGzip("gzip;q=0.000"));
  assert(!acceptsDashboardGzip("gzip;q=1.001"));
  assert(!acceptsDashboardGzip("gzip;q=0.0000"));
  assert(!acceptsDashboardGzip("gzip;q=bogus"));
  assert(!acceptsDashboardGzip("gzip;q=0; q=1"));
  assert(acceptsDashboardGzip("br, gzip"));
  assert(acceptsDashboardGzip("GZIP; q=0.001, br"));
  assert(acceptsDashboardGzip("gzip;q=1.000"));
  assert(acceptsDashboardGzip("gzip \t"));
  assert(acceptsDashboardGzip("gzip;q=0.5 \t"));
}
''', encoding="utf-8")
    executable = folder / "negotiation"
    subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", str(root / "station/include"), str(test), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
    print(f"Dashboard: {len(original)} bytes; gzip: {len(payload)} bytes; round-trip and negotiation passed")

"""Parse the JavaScript actually embedded in the station pages using Node.js."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
node = shutil.which("node")
if not node:
    raise SystemExit("Node.js is required")
count = 0
with tempfile.TemporaryDirectory(prefix="wcharger-js-") as folder:
    for source in (root / "station/src/web_pages.cpp", root / "station/include/ota_page.h"):
        for script in re.findall(r"<script>(.*?)</script>", source.read_text(encoding="utf-8"), re.S):
            path = Path(folder) / f"page-{count}.js"
            path.write_text(script, encoding="utf-8")
            subprocess.run([node, "--check", str(path)], check=True)
            count += 1
print(f"Parsed {count} embedded station scripts successfully")

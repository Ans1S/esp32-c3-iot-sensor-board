"""Verify clean upstream inputs, patch idempotence and fail-closed layout checks.

Downloads only immutable pinned source files into a temporary directory unless
--cache points to a directory containing the verified original files.
"""
import argparse
import hashlib
from pathlib import Path
import runpy
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
BSEC = "https://raw.githubusercontent.com/boschsensortec/Bosch-BSEC2-Library/4f559a6aa450f0ce436e602b42dc384f52128c66/src/"
BME = "https://raw.githubusercontent.com/adafruit/Adafruit_BME280_Library/2.3.0/"
BOSCH = "https://raw.githubusercontent.com/boschsensortec/Bosch-BME68x-Library/4f37df69dde205e4f58d2bb9a45568c61e128e86/src/bme68x/"
FILES = [
    ("bsec2.cpp", BSEC, "bsec2/src/bsec2.cpp", "6e5099ba63aa8ef9d8524b2896516f6a6bb81f8c15f4c16ae938813aab98207f"),
    ("bsec2.h", BSEC, "bsec2/src/bsec2.h", "682a22c6b93cf33323565bc1f75c7b69188a73e5ede32171d80427867e723921"),
    ("Adafruit_BME280.cpp", BME, "Adafruit BME280 Library/Adafruit_BME280.cpp", "7298a9c56b33f4a47f3575c5fc5f69813fc6ebfd353c225a6e0ef74bb3ad9ea4"),
    ("library.properties", BME, "Adafruit BME280 Library/library.properties", "f96c6043f3693b9ad57c14d8e49ec7e922b93f12e16445d0e2e7d3fb87450210"),
    ("bme68x.c", BOSCH, "BME68x Sensor library/src/bme68x/bme68x.c", "bad525ea57fe57a5d7dda19d287781bcbab50fdbf2dedea737c16b206cbe0157"),
]


class Environment:
    def __init__(self, root):
        self.root = root

    def subst(self, value):
        return {"$PROJECT_DIR": str(self.root), "$PROJECT_LIBDEPS_DIR": "deps",
                "$PIOENV": "fixture"}[value]


def apply(root, script):
    runpy.run_path(str(ROOT / "sensor/scripts" / script),
                  init_globals={"env": Environment(root), "Import": lambda _: None})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="wcharger-patches-") as temporary:
        root = Path(temporary)
        originals = {}
        for name, base, relative, digest in FILES:
            data = ((args.cache / name).read_bytes() if args.cache else
                    urllib.request.urlopen(base + name, timeout=30).read())
            if hashlib.sha256(data).hexdigest() != digest:
                raise RuntimeError("Unexpected upstream input: " + name)
            target = root / "deps/fixture" / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
            originals[target] = data
        scripts = ["patch_bsec2.py", "patch_bme280.py", "patch_bme68x.py"]
        for script in scripts:
            apply(root, script)
        first = {path: path.read_bytes() for path in originals}
        for script in scripts:
            apply(root, script)
        assert first == {path: path.read_bytes() for path in originals}
        incompatible = [
            ("bsec2/src/bsec2.cpp", b"2.1.5", b"9.9.9", scripts[0]),
            ("Adafruit BME280 Library/library.properties", b"version=2.3.0", b"version=9.9.9", scripts[1]),
            ("BME68x Sensor library/src/bme68x/bme68x.c", b"Call until in sleep", b"Unsupported layout", scripts[2]),
        ]
        for relative, before, after, script in incompatible:
            path = root / "deps/fixture" / relative
            path.write_bytes(originals[path].replace(before, after))
            try:
                apply(root, script)
            except RuntimeError:
                pass
            else:
                raise AssertionError("Unknown input was accepted: " + script)
        print("All three library patches passed pristine-input, repeat and rejection tests")


if __name__ == "__main__":
    main()

"""Check that a normal station firmware update cannot request full flash erasure."""
import runpy
from pathlib import Path
import unittest


class UploadEnvironment(dict):
    def __init__(self, erase="false"):
        super().__init__(UPLOADERFLAGS=["--chip", "esp32s3", "write-flash", "-z"])
        self.erase = erase

    def GetProjectOption(self, name, default):
        return self.erase if name == "custom_erase_all" else default

    def Replace(self, **values):
        self.update(values)


class StationUploadTests(unittest.TestCase):
    def apply(self, env):
        runpy.run_path(str(Path(__file__).resolve().parents[1] / "station/fresh_upload.py"),
                       init_globals={"env": env, "Import": lambda _: None})

    def test_normal_update_preserves_flash(self):
        env = UploadEnvironment()
        self.apply(env)
        self.assertNotIn("--erase-all", env["UPLOADERFLAGS"])

    def test_explicit_factory_reset(self):
        env = UploadEnvironment("true")
        self.apply(env)
        flags = env["UPLOADERFLAGS"]
        self.assertEqual(flags[flags.index("write-flash") + 1], "--erase-all")
        self.apply(env)
        self.assertEqual(flags.count("--erase-all"), 1)


if __name__ == "__main__":
    unittest.main()

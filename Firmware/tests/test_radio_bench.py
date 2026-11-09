"""Exercise the sender benchmark summary with mixed modes and serial noise."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


TOOL = Path(__file__).resolve().parents[1] / "tools" / "summarize_radio_bench.py"


class RadioBenchTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        root = Path(self.temporary.name)
        self.trace = root / "serial.log"
        self.output = root / "summary.json"
        self.image = root / "firmware.bin"
        self.image.write_bytes(b"fixture sensor image")

    def run_capture(self, content):
        self.trace.write_text(content, encoding="utf-8")
        return subprocess.run([sys.executable, str(TOOL), str(self.trace), "--pcb", "4",
                               "--firmware-bin", str(self.image), "--output", str(self.output)],
                              capture_output=True, text=True, timeout=10)

    def test_ack_attempts_modes_and_private_lines(self):
        result = self.run_capture("boot log with private fixture data\n"
                                  "WCH_RADIO,1,4,10,5,2,1,1,2000\n"
                                  "WCH_RADIO,1,4,11,6,2,3,0,42000\n"
                                  "WCH_RADIO,1,4,1,1,1,0,0,200\n"
                                  "WCH_RADIO,1,4,1,1,1,2,1,5000\n"
                                  "WCH_RADIO,1,4,1\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        raw = self.output.read_text(encoding="utf-8")
        self.assertNotIn("private fixture", raw)
        capture = json.loads(raw)
        self.assertEqual(capture["sensorImageSha256"], hashlib.sha256(self.image.read_bytes()).hexdigest())
        self.assertEqual(capture["malformedLines"], 1)
        self.assertEqual(capture["summary"]["applicationAckPercent"], 50)
        self.assertEqual(capture["summary"]["additionalFrames"], 3)
        self.assertEqual(capture["summary"]["contactsWithNoFrame"], 1)
        self.assertEqual(capture["summary"]["elapsedUs"], {"p50": 2000, "p95": 42000, "p99": 42000})
        self.assertEqual(capture["modes"]["2"]["acknowledgedElapsedUs"]["p95"], 2000)
        self.assertEqual(capture["modes"]["1"]["contacts"], 2)

    def test_empty_or_mixed_capture_is_rejected(self):
        for content in ("ordinary boot log\n", "WCH_RADIO,1,3,10,5,2,1,1,100\n"):
            result = self.run_capture(content)
            self.assertEqual(result.returncode, 2)
            self.assertFalse(self.output.exists())

    def test_existing_output_is_preserved(self):
        self.output.write_text("prior capture", encoding="utf-8")
        result = self.run_capture("WCH_RADIO,1,4,10,5,2,1,1,100\n")
        self.assertEqual(result.returncode, 2)
        self.assertEqual(self.output.read_text(encoding="utf-8"), "prior capture")


if __name__ == "__main__":
    unittest.main()

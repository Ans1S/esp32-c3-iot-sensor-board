"""Run the hardware capture client against a local, synthetic station."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest


TOOL = Path(__file__).resolve().parents[1] / "tools" / "capture_station_bench.py"
MAC = "02:00:00:00:00:11"
COOKIE = "session=synthetic-session-secret"


class StationFixture(BaseHTTPRequestHandler):
    def do_GET(self):
        self.server.requests.append((self.path, self.headers.get("Cookie")))
        if self.headers.get("Cookie") != COOKIE:
            self.send_response(302)
            self.send_header("Location", "/login")
            self.end_headers()
            return
        if self.path == "/api/status":
            value = self.server.statuses.pop(0)
            value = {**value, "wifi": {"ssid": "test-private-fixture"}, "sensors": [{"mac": MAC}],
                     "memory": {"internalMinimumFreeBytes": 12345}, "credential": COOKIE}
        else:
            value = {"points": [{"temperature": 22.5, "mac": MAC}]}
        body = json.dumps(value).encode()
        self.send_response(200)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_):
        pass


class CaptureTest(unittest.TestCase):
    def setUp(self):
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), StationFixture)
        self.server.statuses = []
        self.server.requests = []
        self.worker = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.worker.start()
        self.origin = f"http://127.0.0.1:{self.server.server_port}"
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.image = root / "firmware.bin"
        self.image.write_bytes(b"synthetic image")
        self.output = root / "capture.json"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.worker.join()
        self.temporary.cleanup()

    def capture(self, *, cookie=COOKIE, samples=2, sensor=None, origin=None):
        command = [sys.executable, str(TOOL), origin or self.origin, "--firmware-bin", str(self.image),
                   "--output", str(self.output), "--samples", str(samples), "--interval", "0"]
        if sensor:
            command.extend(["--sensor", sensor])
        return subprocess.run(command, capture_output=True, text=True, timeout=20,
                              env={**os.environ, "WCHARGER_BENCH_COOKIE": cookie})

    def test_status_history_and_sensitive_field_exclusion(self):
        self.server.statuses = [{"uptimeMs": 1000, "espnow": {"received": 5, "dropped": 1}},
                                {"uptimeMs": 2000, "espnow": {"received": 8, "dropped": 1}}]
        result = self.capture(sensor=MAC)
        self.assertEqual(result.returncode, 0, result.stderr)
        raw = self.output.read_text()
        for private in (MAC, COOKIE, "test-private-fixture", self.origin):
            self.assertNotIn(private, raw)
        document = json.loads(raw)
        self.assertEqual(document["stationImageSha256"], hashlib.sha256(self.image.read_bytes()).hexdigest())
        summary = document["summary"]
        self.assertEqual(summary["counterDeltas"]["espnow"]["received"], 3)
        self.assertEqual(summary["counterDeltas"]["espnow"]["dropped"], 0)
        self.assertIsNone(summary["counterDeltas"]["espnow"]["replyBusyDrops"])
        self.assertEqual(summary["history"]["requests"], 2)
        self.assertEqual(document["samples"][0]["history"]["points"], 1)
        self.assertEqual(document["samples"][0]["status"]["memory"]["internalMinimumFreeBytes"], 12345)
        self.assertTrue(all(cookie == COOKIE for _, cookie in self.server.requests))
        self.assertTrue(self.server.requests[1][0].startswith("/api/history?mac="))
        self.assertEqual(summary["status"]["errors"], 0)

    def test_restart_invalidates_counter_deltas(self):
        self.server.statuses = [{"uptimeMs": 9000, "espnow": {"received": 10}},
                                {"uptimeMs": 2000, "espnow": {"received": 30}}]
        result = self.capture()
        self.assertEqual(result.returncode, 0, result.stderr)
        summary = json.loads(self.output.read_text())["summary"]
        self.assertTrue(summary["stationRestartObserved"])
        self.assertIsNone(summary["counterDeltas"]["espnow"]["received"])

    def test_redirect_is_recorded_without_following_it(self):
        result = self.capture(cookie="", samples=1)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(self.server.requests, [("/api/status", None)])
        document = json.loads(self.output.read_text())
        self.assertEqual(document["samples"][0]["status"]["error"], "http_302")

    def test_existing_capture_and_credentials_in_url_are_rejected(self):
        result = self.capture(origin=self.origin.replace("://", "://test:fixture@"))
        self.assertEqual(result.returncode, 2)
        self.assertFalse(self.output.exists())
        self.output.write_text("existing capture")
        result = self.capture()
        self.assertEqual(result.returncode, 2)
        self.assertEqual(self.output.read_text(), "existing capture")
        self.assertFalse(self.server.requests)


if __name__ == "__main__":
    unittest.main()

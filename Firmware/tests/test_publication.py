"""Exercise publication guards with generated credentials and Git snapshots."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
import zipfile

SPEC = importlib.util.spec_from_file_location(
    "publication", Path(__file__).resolve().parents[1] / "tools/check_publication.py")
guard = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(guard)


class PublicationTests(unittest.TestCase):
    def scan(self, name, data):
        findings = []
        stats = {"files_scanned": 0, "archive_members_scanned": 0}
        guard.scan_bytes(name, data, findings, stats)
        return findings

    def test_tokens_in_binary_and_archive(self):
        token = b"gh" + b"p_" + b"A" * 36
        for name, data in [("app.bin", b"\0" + token + b"\0"),
                           ("source.cpp", b'const char* apiKey="' + b"Q" * 24 + b'";')]:
            self.assertTrue(self.scan(name, data))
        archive = io.BytesIO()
        with zipfile.ZipFile(archive, "w") as z:
            z.writestr("data.txt", token)
        findings = self.scan("production.zip", archive.getvalue())
        self.assertEqual(findings[0]["file"], "production.zip!data.txt")
        self.assertNotIn(token.decode(), json.dumps(findings))

    def test_private_material_and_parser_markers(self):
        marker = b"-----BEGIN " + b"PRIVATE KEY-----"
        self.assertFalse(self.scan("tls.bin", marker + b"\0"))
        key = marker + b"\n" + b"A" * 100 + b"\n-----END " + b"PRIVATE KEY-----"
        self.assertEqual(self.scan("app.bin", key)[0]["category"], "private-key-material")

    def test_private_paths_and_approved_examples(self):
        home = b"C:" + b"/Users/" + b"private-user/project/main.cpp"
        self.assertEqual(self.scan("app.bin", home)[0]["category"], "personal-build-path")
        for name in (".tools/ota/signing.key", ".env", "out.zip!secrets.json"):
            self.assertTrue(self.scan(name, b"empty"))
        examples = b'apiKey="YOUR_API_KEY"; wifiSsid="test"; password="W-Charger-Setup"; data-ssid="${esc(n.ssid)}";'
        self.assertFalse(self.scan("fixture.cpp", examples))

    def test_device_exports_and_auth_files_are_rejected(self):
        names = ("device-data/readings.json", "device-backups/station.zip",
                 "flash-dumps/station.bin", "exports/history.csv", "nvs.csv",
                 "Readme/raw/dashboard.png", "Readme/private/screen.webp",
                 ".ssh/id_ed25519", ".aws/config", ".azure/profile.json",
                 ".config/gcloud/configurations/config_default", ".npmrc",
                 ".pypirc", "_netrc", "client.ovpn", "signing.keystore")
        for name in names:
            with self.subTest(name=name):
                self.assertEqual(self.scan(name, b"fixture")[0]["category"],
                                 "private-local-file")
        self.assertFalse(self.scan("Firmware/tests/fixtures/normal-history.csv", b"fixture"))
        self.assertFalse(self.scan("Firmware/releases/station-4.4.1.bin", b"fixture"))

    def test_gitignore_preserves_public_build_inputs_and_release_files(self):
        ignored = [".tools/ota/signing.key", "device-data/readings.json", "nvs.csv",
                   ".npmrc", "Readme/raw/dashboard.png", "Firmware/releases/flash-dump.bin",
                   "Firmware/releases/station-unreviewed.bin"]
        public = ["Firmware/releases/station-4.4.1.bin",
                  "Firmware/releases/sensor-v4-4.4.1.ota", "Firmware/ota_package.py",
                  "Firmware/sensor/platformio.ini", "Readme/station-history-4.4.1.webp"]
        result = subprocess.run(
            ["git", "check-ignore", "--no-index", "-z", "--stdin"], cwd=guard.ROOT,
            input=("\0".join(ignored + public) + "\0").encode(),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        self.assertIn(result.returncode, (0, 1))
        self.assertEqual({name.decode() for name in result.stdout.split(b"\0") if name},
                         set(ignored))

    def test_staged_blob_cannot_be_hidden_by_worktree_edit(self):
        previous_root = guard.ROOT
        with tempfile.TemporaryDirectory(prefix="publication-test-") as temp:
            root = Path(temp)
            subprocess.run(["git", "init", "-q", temp], check=True)
            fixture_token = "gh" + "p_" + "B" * 36
            file = root / "app.bin"
            file.write_bytes(fixture_token.encode())
            subprocess.run(["git", "-C", temp, "add", "app.bin"], check=True)
            file.write_bytes(b"clean working tree")
            guard.ROOT = root
            try:
                output = io.StringIO()
                with contextlib.redirect_stdout(output):
                    self.assertEqual(guard.main(["--staged"]), 1)
                self.assertNotIn(fixture_token, output.getvalue())
                self.assertEqual(json.loads(output.getvalue())["findings"][0]["category"], "provider-token")
                subprocess.run(["git", "-C", temp, "add", "app.bin"], check=True)
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(guard.main(["--staged"]), 0)
            finally:
                guard.ROOT = previous_root

    def test_new_history_retains_removed_secret(self):
        previous_root = guard.ROOT
        with tempfile.TemporaryDirectory(prefix="publication-history-test-") as temp:
            root = Path(temp)
            def run(*args):
                return subprocess.run(["git", "-C", temp, *args], check=True,
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            def commit(message):
                run("-c", "user.name=Test", "-c", "user.email=test@example.invalid",
                    "commit", "--allow-empty", "-qm", message)
            run("init", "-q")
            commit("Baseline")
            fixture_token = "gh" + "p_" + "C" * 36
            file = root / "app.bin"
            file.write_bytes(fixture_token.encode())
            run("add", "app.bin"); commit("Introduce fixture")
            file.write_bytes(b"clean final image")
            run("add", "app.bin"); commit("Clean final image")
            guard.ROOT = root
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(guard.main(["--revision", "HEAD"]), 0)
                output = io.StringIO()
                with contextlib.redirect_stdout(output):
                    self.assertEqual(guard.main(["--range", "HEAD~2..HEAD"]), 1)
                self.assertNotIn(fixture_token, output.getvalue())
            finally:
                guard.ROOT = previous_root


if __name__ == "__main__":
    unittest.main()

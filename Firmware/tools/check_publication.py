"""Check publication candidates without printing matched sensitive values.

Scans text, firmware bytes and ZIP members. Use --staged before committing,
--revision HEAD before pushing, and --history for a historical exposure review.
This is a heuristic guard, not proof that every possible secret is absent.
"""
import argparse
import io
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[2]
RULES = {
    "private-key-material": re.compile(
        rb"-----BEGIN (?:RSA |EC |OPENSSH |DSA |ENCRYPTED )?PRIVATE KEY-----"
        rb"[\r\n]+[A-Za-z0-9+/=\r\n]{80,}-----END "
        rb"(?:RSA |EC |OPENSSH |DSA |ENCRYPTED )?PRIVATE KEY-----"),
    "provider-token": re.compile(
        rb"(?<![A-Za-z0-9])(?:gh[pousr]_[A-Za-z0-9_]{30,}|"
        rb"github_pat_[A-Za-z0-9_]{20,}|sk-(?:proj-)?[A-Za-z0-9_-]{20,}|"
        rb"xox[baprs]-[A-Za-z0-9-]{10,}|AKIA[0-9A-Z]{16}|ASIA[0-9A-Z]{16}|"
        rb"AIza[0-9A-Za-z_-]{35})(?![A-Za-z0-9])"),
    "signed-access-url": re.compile(
        rb"(?:x-oss-(?:security-token|credential|signature)|"
        rb"X-Amz-(?:Credential|Signature|Security-Token))=[^&\s\"\x27<>]{8,}", re.I),
    "personal-build-path": re.compile(
        rb"(?:[A-Za-z]:[/\\]+Users[/\\]+[A-Za-z0-9_.-]+|"
        rb"/(?:Users|home)/[A-Za-z0-9_.-]+/)"),
}
LITERAL = re.compile(
    rb"\b(?:user[_-]?api[_-]?key|read[_-]?api[_-]?key|write[_-]?api[_-]?key|"
    rb"api[_-]?key|wifi[_-]?(?:password|ssid)|admin[_-]?password|"
    rb"access[_-]?point[_-]?password|password|ssid|secret|token)"
    rb"[\"\x27]?\s*[:=]\s*([\"\x27])([^\r\n\"\x27]{1,128})\1", re.I)
PLACEHOLDERS = {b"test", b"test-key", b"test-password", b"demo", b"test-network",
                b"demo home network", b"w-charger-setup", b"w-charger-demo"}
DENIED_NAME = re.compile(r"(?:^|/)(?:\.env(?:\..*)?|(?:secrets?|credentials?)"
                         r"(?:\..*)?|config\.local\..*|.*\.(?:pem|key|p12|pfx))$", re.I)
LOCAL_COMPONENTS = {".tools", ".pio", ".agents", ".codex", "captures"}
MAX_ARCHIVE_BYTES = 64 * 1024 * 1024


def git(*arguments, data=None):
    return subprocess.run(["git", *arguments], cwd=ROOT, input=data,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          check=True).stdout


def path_is_private(name):
    path = PurePosixPath(name.replace("\\", "/"))
    return (bool(DENIED_NAME.search(path.as_posix())) or
            any(part.lower() in LOCAL_COMPONENTS for part in path.parts) or
            path.name in {"AGENTS.md", "AGENTS.override.md", "ota_public_key.h"})


def placeholder(value):
    low = value.strip().lower()
    return (low in PLACEHOLDERS or low.startswith((b"your_", b"your-", b"example-", b"<"))
            or low.startswith(b"test-") or low.startswith(b"demo-")
            or (low.startswith(b"${") and low.endswith(b"}")))


def scan_bytes(name, data, findings, stats, depth=0):
    stats["files_scanned" if depth == 0 else "archive_members_scanned"] += 1
    if path_is_private(name.split("!", 1)[-1]):
        findings.append({"file": name, "category": "private-local-file"})
    for category, pattern in RULES.items():
        for match in pattern.finditer(data):
            # A bare PEM marker in the TLS parser is not key material.
            location = {"byte": match.start()} if b"\0" in data else {
                "line": data.count(b"\n", 0, match.start()) + 1}
            findings.append({"file": name, "category": category, **location})
    for match in LITERAL.finditer(data):
        if not placeholder(match.group(2)):
            findings.append({"file": name, "category": "credential-literal",
                             "byte": match.start()})
    if name.lower().endswith(".zip"):
        try:
            with zipfile.ZipFile(io.BytesIO(data)) as archive:
                members = archive.infolist()
                if depth >= 3 or sum(m.file_size for m in members) > MAX_ARCHIVE_BYTES:
                    raise ValueError("Archive inspection limit")
                for member in members:
                    if not member.is_dir():
                        scan_bytes(name + "!" + member.filename, archive.read(member),
                                   findings, stats, depth + 1)
        except (ValueError, OSError, RuntimeError, zipfile.BadZipFile):
            findings.append({"file": name, "category": "unreadable-or-oversized-archive"})


def blobs(entries):
    """Read Git objects in one batch, including binary staged blobs."""
    if not entries:
        return
    output = io.BytesIO(git("cat-file", "--batch",
                           data=b"".join(oid.encode() + b"\n" for oid, _ in entries)))
    for oid, name in entries:
        header = output.readline().split()
        if len(header) != 3 or header[1] != b"blob":
            raise ValueError("Expected readable Git blob")
        size = int(header[2])
        yield name, output.read(size)
        if output.read(1) != b"\n":
            raise ValueError("Invalid Git batch framing")


def index_entries():
    for row in git("ls-files", "--stage", "-z").split(b"\0"):
        if not row:
            continue
        metadata, name = row.split(b"\t", 1)
        mode, oid, stage = metadata.split()
        if stage != b"0":
            raise ValueError("Unresolved index entries")
        if mode == b"160000":
            raise ValueError("Submodules need a separate publication review")
        yield oid.decode(), name.decode("utf-8")


def revision_entries(revision):
    for row in git("ls-tree", "-r", "-z", revision).split(b"\0"):
        if not row:
            continue
        metadata, name = row.split(b"\t", 1)
        mode, kind, oid = metadata.split()
        if kind != b"blob":
            raise ValueError("Submodules need a separate publication review")
        yield oid.decode(), name.decode("utf-8")


def historical_entries(revision_range=None, exclude_remotes=False):
    args = ["rev-list", "--objects"]
    args += [revision_range] if revision_range else ["--all"]
    if exclude_remotes:
        args += ["--not", "--remotes"]
    rows = [row.split(b" ", 1) for row in git(*args).splitlines()]
    oids = [row[0] for row in rows]
    checked = git("cat-file", "--batch-check", data=b"\n".join(oids) + b"\n").splitlines() if oids else []
    for row, metadata in zip(rows, checked):
        if metadata.split()[1:2] == [b"blob"]:
            yield row[0].decode(), row[1].decode("utf-8") if len(row) > 1 else row[0].decode()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--staged", action="store_true")
    modes.add_argument("--revision")
    modes.add_argument("--history", action="store_true")
    modes.add_argument("--range", dest="revision_range")
    parser.add_argument("--exclude-remotes", action="store_true")
    args = parser.parse_args(argv)
    if args.staged:
        candidates = blobs(list(index_entries()))
    elif args.revision:
        candidates = blobs(list(revision_entries(args.revision)))
    elif args.history or args.revision_range:
        candidates = blobs(list(historical_entries(args.revision_range, args.exclude_remotes)))
    else:
        paths = sorted(set(git("ls-files", "--cached", "--others", "--exclude-standard", "-z").split(b"\0")) - {b""})
        candidates = ((name.decode("utf-8"), (ROOT / name.decode("utf-8")).read_bytes())
                      for name in paths if (ROOT / name.decode("utf-8")).is_file())
    findings = []
    stats = {"files_scanned": 0, "archive_members_scanned": 0}
    for name, data in candidates:
        scan_bytes(name, data, findings, stats)
    print(json.dumps({**stats, "findings": findings}, indent=2))
    return 1 if findings else 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Capture read-only HTTP timings and selected station counters as JSON."""

import argparse
import hashlib
import http.client
import json
import math
import os
from pathlib import Path
import re
import time
from urllib.parse import urlencode, urlsplit


COUNTERS = {
    "espnow": ("received", "invalid", "dropped", "persistenceDrops", "peerFailures",
               "replyBusyDrops", "replyMacFailures"),
    "cloud": ("successfulUploads", "failedUploads", "droppedJobs"),
}
MEMORY = ("internalFreeBytes", "internalMinimumFreeBytes", "internalLargestBlockBytes",
          "psramFreeBytes", "psramLargestBlockBytes")
MAX_BODY = 2 * 1024 * 1024


def parse_origin(url):
    origin = urlsplit(url)
    if (origin.scheme not in {"http", "https"} or not origin.hostname or origin.username is not None
            or origin.path not in {"", "/"} or origin.query or origin.fragment):
        raise ValueError("Enter a station origin without credentials, path or query")
    _ = origin.port  # Validate an explicit port before capturing.
    return origin


def numeric_fields(document, names):
    if not isinstance(document, dict):
        return {}
    return {name: document[name] for name in names
            if type(document.get(name)) is int and document[name] >= 0}


def status_fields(document):
    result = numeric_fields(document, ("uptimeMs",))
    if "uptimeMs" not in result or not isinstance(document.get("espnow"), dict):
        raise ValueError("Unexpected status response")
    for group, names in COUNTERS.items():
        result[group] = numeric_fields(document.get(group), names)
    result["memory"] = numeric_fields(document.get("memory"), MEMORY)
    return result


def probe(origin, route, timeout, cookie, history=False):
    connection_type = (http.client.HTTPSConnection if origin.scheme == "https"
                       else http.client.HTTPConnection)
    connection = connection_type(origin.hostname, origin.port, timeout=timeout)
    headers = {"Accept": "application/json"}
    if cookie:
        headers["Cookie"] = cookie
    started = time.perf_counter()
    try:
        connection.request("GET", route, headers=headers)
        response = connection.getresponse()
        header_ms = (time.perf_counter() - started) * 1000
        if response.status != 200:
            return {"error": f"http_{response.status}"}
        body = response.read(MAX_BODY + 1)
        elapsed_ms = (time.perf_counter() - started) * 1000
        if len(body) > MAX_BODY:
            return {"error": "response_too_large"}
        document = json.loads(body)
        if not isinstance(document, dict) or document.get("success") is False:
            return {"error": "unexpected_response"}
        result = {"headerMs": round(header_ms, 3), "completeMs": round(elapsed_ms, 3),
                  "responseBytes": len(body)}
        if history:
            if not isinstance(document.get("points"), list):
                return {"error": "unexpected_response"}
            result["points"] = len(document["points"])
        else:
            result.update(status_fields(document))
        return result
    except (OSError, http.client.HTTPException):
        return {"error": "connection_or_timeout"}
    except ValueError:
        return {"error": "unexpected_response"}
    finally:
        connection.close()


def percentiles(values):
    ordered = sorted(values)
    if not ordered:
        return {}
    return {f"p{p}": ordered[math.ceil(len(ordered) * p / 100) - 1] for p in (50, 95, 99)}


def summarize(samples):
    result = {}
    for endpoint in ("status", "history"):
        entries = [sample[endpoint] for sample in samples if endpoint in sample]
        if not entries:
            continue
        successful = [entry for entry in entries if "error" not in entry]
        result[endpoint] = {"requests": len(entries), "errors": len(entries) - len(successful),
                            "headerMs": percentiles([entry["headerMs"] for entry in successful]),
                            "completeMs": percentiles([entry["completeMs"] for entry in successful])}
    statuses = [sample["status"] for sample in samples if "error" not in sample["status"]]
    restarted = any(new["uptimeMs"] < old["uptimeMs"] for old, new in zip(statuses, statuses[1:]))
    result["stationRestartObserved"] = restarted
    result["counterDeltas"] = {}
    for group, names in COUNTERS.items():
        deltas = result["counterDeltas"][group] = {}
        for name in names:
            values = [status[group].get(name) for status in statuses]
            valid = (len(values) >= 2 and not restarted and all(value is not None for value in values)
                     and all(new >= old for old, new in zip(values, values[1:])))
            deltas[name] = values[-1] - values[0] if valid else None
    return result


def capture(origin, samples, interval, timeout, cookie, sensor):
    history = "/api/history?" + urlencode({"mac": sensor, "limit": 512}) if sensor else None
    started = time.perf_counter()
    records = []
    for index in range(samples):
        round_started = time.perf_counter()
        record = {"elapsedMs": round((round_started - started) * 1000, 3),
                  "status": probe(origin, "/api/status", timeout, cookie)}
        if history:
            record["history"] = probe(origin, history, timeout, cookie, history=True)
        records.append(record)
        remaining = round_started + interval - time.perf_counter()
        if index + 1 < samples and remaining > 0:
            time.sleep(remaining)
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("origin", help="Station HTTP origin")
    parser.add_argument("--firmware-bin", required=True, type=Path, help="Application image installed on the test station")
    parser.add_argument("--output", required=True, type=Path, help="New JSON capture file")
    parser.add_argument("--samples", type=int, default=60)
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between sample starts")
    parser.add_argument("--timeout", type=float, default=4.0, help="Per-request socket timeout in seconds")
    parser.add_argument("--sensor", help="Optionally request a normal history page for this MAC")
    args = parser.parse_args()
    try:
        origin = parse_origin(args.origin)
        if (args.samples < 1 or not math.isfinite(args.interval) or args.interval < 0
                or not math.isfinite(args.timeout) or args.timeout <= 0):
            raise ValueError("Use positive samples and timeout, and a nonnegative interval")
        if args.sensor and not re.fullmatch(r"(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}", args.sensor):
            raise ValueError("Use a colon-separated sensor MAC")
        digest = hashlib.sha256(args.firmware_bin.read_bytes()).hexdigest()
        with args.output.open("x", encoding="utf-8") as output:
            samples = capture(origin, args.samples, args.interval, args.timeout,
                              os.environ.get("WCHARGER_BENCH_COOKIE"), args.sensor)
            summary = summarize(samples)
            json.dump({"stationImageSha256": digest, "intervalSeconds": args.interval,
                       "timeoutSeconds": args.timeout, "summary": summary,
                       "samples": samples}, output, indent=2)
            output.write("\n")
        if summary["status"]["errors"] == args.samples:
            raise SystemExit(1)
    except (ValueError, OSError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()

"""Summarize application ACKs and sender timings from one sensor's bench log."""

import argparse
import hashlib
import json
from pathlib import Path
import re

from capture_station_bench import percentiles


MARKER = "WCH_RADIO,"
ROW = re.compile(r"WCH_RADIO,1,([34]),([0-9]{1,10}),([0-9]{1,10}),([0-9]{1,3}),"
                 r"([0-9]{1,10}),([01]),([0-9]{1,19})")
UINT32_MAX = (1 << 32) - 1


def read_samples(lines, pcb):
    samples = []
    malformed = 0
    for raw in lines:
        line = raw.strip()
        if not line.startswith(MARKER):
            continue
        match = ROW.fullmatch(line)
        if not match:
            malformed += 1
            continue
        board, boot, sequence, mode, frames, ack, elapsed_us = map(int, match.groups())
        if board != pcb:
            raise ValueError("The log contains a radio sample for a different PCB revision")
        if max(boot, sequence, frames) > UINT32_MAX or mode > 255:
            malformed += 1
            continue
        samples.append({"boot": boot, "sequence": sequence, "mode": mode, "acceptedFrames": frames,
                        "applicationAck": bool(ack), "elapsedUs": elapsed_us})
    if not samples:
        raise ValueError("The log contains no complete radio benchmark samples")
    return samples, malformed


def summarize(samples):
    accepted = [sample for sample in samples if sample["applicationAck"]]
    frames = [sample["acceptedFrames"] for sample in samples]
    return {
        "contacts": len(samples),
        "applicationAcks": len(accepted),
        "applicationAckPercent": round(100 * len(accepted) / len(samples), 3),
        "acceptedFrames": sum(frames),
        "additionalFrames": sum(max(0, count - 1) for count in frames),
        "contactsWithMultipleFrames": sum(count > 1 for count in frames),
        "contactsWithNoFrame": frames.count(0),
        "acksWithNoFrame": sum(not sample["acceptedFrames"] for sample in accepted),
        "elapsedUs": percentiles([sample["elapsedUs"] for sample in samples]),
        "acknowledgedElapsedUs": percentiles([sample["elapsedUs"] for sample in accepted]),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path, help="Serial capture from a single sensor")
    parser.add_argument("--pcb", type=int, choices=(3, 4), required=True)
    parser.add_argument("--firmware-bin", type=Path, required=True, help="Application image used for the capture")
    parser.add_argument("--output", type=Path, required=True, help="New JSON summary file")
    args = parser.parse_args()
    try:
        with args.trace.open(encoding="utf-8", errors="replace") as trace:
            samples, malformed = read_samples(trace, args.pcb)
        digest = hashlib.sha256(args.firmware_bin.read_bytes()).hexdigest()
        modes = {}
        for sample in samples:
            modes.setdefault(sample["mode"], []).append(sample)
        with args.output.open("x", encoding="utf-8") as output:
            json.dump({"pcb": args.pcb, "sensorImageSha256": digest, "malformedLines": malformed,
                       "summary": summarize(samples), "modes": {mode: summarize(rows) for mode, rows in modes.items()},
                       "samples": samples}, output, indent=2)
            output.write("\n")
    except (ValueError, OSError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()

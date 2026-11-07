"""Summarize emitted project functions using the ELF's matching GNU binutils.

Static instruction/call-site counts are not execution counts or MCU timings.
Debug locations identify emitted definitions; inlined, merged and discarded
source functions do not necessarily have separate symbols. No device access.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import subprocess


def output(tool, *arguments):
    return subprocess.check_output([str(tool), *map(str, arguments)], text=True, encoding="utf-8")


def project_path(location):
    normalized = location.replace("\\", "/")
    if "/Firmware/" not in normalized:
        return ""
    path = "Firmware/" + normalized.rsplit("/Firmware/", 1)[1]
    # Remapped external paths can be relative to a project compilation folder.
    # Accept only the maintained application source/header directories.
    return path if re.fullmatch(r"Firmware/(?:sensor|station|shared)/(?:src|include)/[^/]+\.(?:cpp|h)", path) else ""


def instructions(disassembly):
    functions, current = {}, None
    for line in disassembly.splitlines():
        header = re.match(r"^([0-9a-f]+) <(.+)>:$", line)
        if header:
            current = int(header[1], 16)
            functions.setdefault(current, [])
        elif current is not None:
            fields = line.strip().split("\t")
            if len(fields) >= 3 and re.fullmatch(r"[0-9a-f]+:", fields[0]):
                functions[current].append(" ".join(fields[2:]))
    return functions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path)
    parser.add_argument("--tool-prefix", required=True,
                        help="path/name before nm and objdump, including the trailing hyphen")
    parser.add_argument("--target", required=True)
    parser.add_argument("--phase", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    suffix = ".exe" if Path(args.tool_prefix + "nm.exe").exists() else ""
    nm = args.tool_prefix + "nm" + suffix
    objdump = args.tool_prefix + "objdump" + suffix
    symbols = output(nm, "-S", "-C", "-l", "--size-sort", args.elf)
    disassembly = output(objdump, "-d", "-C", args.elf)
    headers = output(objdump, "-h", args.elf)
    code = instructions(disassembly)
    sections = []
    for line in headers.splitlines():
        match = re.match(r"\s*\d+\s+(\S+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s", line)
        if match:
            sections.append({"name": match[1], "size": int(match[2], 16), "address": int(match[3], 16)})
    rows = []
    for line in symbols.splitlines():
        match = re.match(r"^([0-9a-f]+) ([0-9a-f]+) ([tTwW]) (.+)\t(.+):(\d+)$", line)
        if not match:
            continue
        source = project_path(match[5])
        if not source:
            continue
        address = int(match[1], 16)
        insns = code.get(address, [])
        calls = []
        for instruction in insns:
            callee = re.search(r"<(.+)>", instruction)
            if callee and "+0x" not in callee[1] and instruction.split()[0] in {
                    "jal", "jalr", "jr", "j", "call0", "call4", "call8", "call12"}:
                calls.append(callee[1])
        section = next((s["name"] for s in sections if s["address"] <= address < s["address"] + s["size"]), "unknown")
        rows.append({"phase": args.phase, "target": args.target, "source": source,
                     "line": int(match[6]), "name": match[4], "address": "0x" + match[1],
                     "bytes": int(match[2], 16), "section": section,
                     "instructions": len(insns),
                     "division_remainder_sites": sum(i.split()[0] in {"div", "divu", "rem", "remu", "quou", "quos", "rems"} for i in insns),
                     "fence_sites": sum(i.startswith(("fence", "memw")) for i in insns),
                     "copy_call_sites": sum(c.startswith(("memcpy", "memmove", "memset")) for c in calls),
                     "software_arithmetic_sites": sum(bool(re.match(r"^__(?:add|sub|mul|div|udiv|mod|umod|neg|extend|trunc|fix|float|eq|ne|lt|le|gt|ge|unord)[a-z0-9]*$", c)) for c in calls),
                     "calls": ";".join(calls)})
    args.output.mkdir(parents=True, exist_ok=True)
    stem = args.phase + "-" + args.target
    fields = ["phase", "target", "source", "line", "name", "address", "bytes", "section", "instructions",
              "division_remainder_sites", "fence_sites", "copy_call_sites", "software_arithmetic_sites", "calls"]
    with (args.output / (stem + "-symbols.csv")).open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    summary = {"phase": args.phase, "target": args.target,
               "elf_sha256": hashlib.sha256(args.elf.read_bytes()).hexdigest(),
               "project_symbols": len(rows), "sections": sections}
    (args.output / (stem + "-elf.json")).write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in summary.items() if key != "sections"}))


if __name__ == "__main__":
    main()

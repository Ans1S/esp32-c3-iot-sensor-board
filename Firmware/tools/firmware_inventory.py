"""Inventory maintained definitions, including templates and embedded scripts.

Install tree-sitter==0.25.2, tree-sitter-cpp==0.23.4 and
tree-sitter-javascript==0.25.0 in a local environment. No build or device access
occurs. Definitions are source locations, not distinct machine-code entries
or a claim of executed branch coverage. --revision reads an immutable Git tree.
"""
import argparse
import ast
import csv
import hashlib
import json
from pathlib import Path
import re
import subprocess

import tree_sitter
import tree_sitter_cpp
import tree_sitter_javascript

ROOT = Path(__file__).resolve().parents[2]
EXTENSIONS = {".cpp", ".h", ".py", ".cjs", ".ini", ".csv", ".json"}
CPP_DEFINITIONS = {"function_definition", "lambda_expression"}
JS_DEFINITIONS = {"function_declaration", "function_expression", "arrow_function",
                  "method_definition", "generator_function_declaration"}


def git(*arguments):
    return subprocess.check_output(["git", *arguments], cwd=ROOT)


def definitions(parser, data, path, language, line_offset=0):
    # Ignore storage attributes and include directives while retaining line
    # numbers. The C++ grammar cannot expand ESP-IDF macros or initializer includes.
    if language == "C++":
        data = b"\n".join(line if line.lstrip().startswith(b"#define") else
                           re.sub(rb"\b(?:PROGMEM|RTC_DATA_ATTR|IRAM_ATTR)\b",
                                  lambda match: b" " * len(match[0]), line)
                           for line in data.split(b"\n"))
        data = re.sub(rb"(?m)^[ \t]*#include[^\r\n]*",
                      lambda match: b" " * len(match[0]), data)
        data = re.sub(rb'__has_include\([^\r\n]*?\)', b"1", data)
    tree = parser.parse(data)
    stack = [tree.root_node]
    rows, errors = [], []
    kinds = CPP_DEFINITIONS if language == "C++" else JS_DEFINITIONS
    while stack:
        node = stack.pop()
        if node.type == "ERROR" or node.is_missing:
            errors.append(node.start_point.row + 1 + line_offset)
        if node.type in kinds:
            named = node.child_by_field_name("declarator" if language == "C++" else "name")
            if named is None and node.parent and node.parent.type in {"variable_declarator", "pair"}:
                named = node.parent.child_by_field_name("name") or node.parent.child_by_field_name("key")
            name = named.text.decode("utf-8") if named else "<anonymous>"
            rows.append({"path": path, "line": node.start_point.row + 1 + line_offset,
                         "end": node.end_point.row + 1 + line_offset, "language": language,
                         "kind": node.type, "name": re.sub(r"\s+", " ", name)})
        stack.extend(reversed(node.children))
    return rows, sorted(set(errors))


def write_csv(path, rows, fields):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--revision")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--prefix", default="current")
    args = parser.parse_args()
    cpp = tree_sitter.Parser(tree_sitter.Language(tree_sitter_cpp.language()))
    js = tree_sitter.Parser(tree_sitter.Language(tree_sitter_javascript.language()))
    if args.revision:
        paths = git("ls-tree", "-r", "--name-only", args.revision, "--", "Firmware").decode().splitlines()
    else:
        paths = git("ls-files", "--cached", "--others", "--exclude-standard", "--", "Firmware").decode().splitlines()
    files, functions = [], []
    for path in sorted(set(paths)):
        if Path(path).suffix not in EXTENSIONS or "/releases/" in path or "/audit/" in path:
            continue
        raw = git("show", args.revision + ":" + path) if args.revision else (ROOT / path).read_bytes()
        source = raw.decode("utf-8")
        rows, errors = [], []
        if Path(path).suffix in {".cpp", ".h"}:
            rows, errors = definitions(cpp, raw, path, "C++")
            for script in re.finditer(r"<script[^>]*>(.*?)</script>", source, re.S):
                found, bad = definitions(js, script[1].encode(), path, "JavaScript",
                                         source[:script.start(1)].count("\n"))
                rows.extend(found)
                errors.extend(bad)
        elif path.endswith(".cjs"):
            rows, errors = definitions(js, raw, path, "JavaScript")
        elif path.endswith(".py"):
            for node in ast.walk(ast.parse(source)):
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
                    rows.append({"path": path, "line": node.lineno, "end": node.end_lineno,
                                 "language": "Python", "kind": type(node).__name__,
                                 "name": getattr(node, "name", "<lambda>")})
        role = "test" if "/tests/" in path else "production"
        for row in rows:
            row["role"] = role
        files.append({"path": path, "role": role, "lines": len(source.splitlines()),
                      "definitions": len(rows), "sha256": hashlib.sha256(raw).hexdigest(),
                      "parse_errors": ";".join(map(str, sorted(set(errors))))})
        functions.extend(rows)
    functions.sort(key=lambda row: (row["path"], row["line"], row["end"]))
    args.output.mkdir(parents=True, exist_ok=True)
    write_csv(args.output / (args.prefix + "-modules.csv"), files,
              ["path", "role", "lines", "definitions", "sha256", "parse_errors"])
    write_csv(args.output / (args.prefix + "-functions.csv"), functions,
              ["path", "line", "end", "language", "kind", "name", "role"])
    print(json.dumps({"modules": len(files), "definitions": len(functions),
                      "production_modules": sum(row["role"] == "production" for row in files),
                      "production_definitions": sum(row["role"] == "production" for row in functions),
                      "parse_errors": {row["path"]: row["parse_errors"] for row in files if row["parse_errors"]}}, indent=2))


if __name__ == "__main__":
    main()

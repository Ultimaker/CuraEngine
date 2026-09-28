#!/usr/bin/env python3
"""Check new C++ lines with CuraEngine's structural ast-grep rules."""

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CPP_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hpp"}
HUNK = re.compile(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,(\d+))? @@")


def added_lines(path: str, staged: bool) -> set[int] | None:
    command = ["git", "diff", "--no-ext-diff", "--unified=0"]
    if staged:
        command.append("--cached")
    else:
        command.append("HEAD")
    result = subprocess.run([*command, "--", path], cwd=ROOT, capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or "git diff failed")

    lines = set()
    for line in result.stdout.splitlines():
        match = HUNK.match(line)
        if match:
            start = int(match.group(1))
            lines.update(range(start, start + int(match.group(2) or 1)))
    if lines:
        return lines
    if result.stdout:
        return set()
    # An untracked file has no diff against HEAD; all its lines are new.
    tracked = subprocess.run(
        ["git", "ls-files", "--error-unmatch", "--", path],
        cwd=ROOT,
        capture_output=True,
    )
    return None if not staged and tracked.returncode else set()


def scan(binary: str, paths: list[str]) -> list[dict]:
    result = subprocess.run(
        [binary, "scan", "--json=compact", *paths],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    if result.returncode not in (0, 1):
        raise RuntimeError(result.stderr.strip() or f"ast-grep exited {result.returncode}")
    try:
        findings = json.loads(result.stdout) if result.stdout.strip() else []
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"ast-grep returned invalid JSON: {exc}") from exc
    if not isinstance(findings, list):
        raise RuntimeError("ast-grep returned an unexpected JSON result")
    if result.returncode == 1 and not findings:
        raise RuntimeError(result.stderr.strip() or "ast-grep failed without diagnostics")
    return findings


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--working", action="store_true", help="check edits relative to HEAD, including untracked files")
    parser.add_argument("paths", nargs="+")
    args = parser.parse_args(argv)

    selected: dict[str, set[int] | None] = {}
    for name in args.paths:
        path = (ROOT / name).resolve()
        if not path.is_relative_to(ROOT) or not path.is_file() or path.suffix.lower() not in CPP_SUFFIXES:
            continue
        relative = path.relative_to(ROOT).as_posix()
        try:
            lines = added_lines(relative, not args.working)
        except RuntimeError as exc:
            print(f"check_ast_grep: {exc}", file=sys.stderr)
            return 1
        if lines != set():
            selected[relative] = lines
    if not selected:
        return 0

    binary = shutil.which("ast-grep")
    if binary is None:
        print("check_ast_grep: ast-grep is missing; install ast-grep-cli==0.45.1.", file=sys.stderr)
        return 0 if args.working else 1

    try:
        findings = scan(binary, sorted(selected))
    except (OSError, RuntimeError) as exc:
        print(f"check_ast_grep: scan failed: {exc}", file=sys.stderr)
        return 1
    violations = []
    for finding in findings:
        path = Path(finding["file"]).as_posix()
        if path.startswith("./"):
            path = path[2:]
        if path not in selected:
            continue
        start = finding["range"]["start"]["line"] + 1
        end = finding["range"]["end"]["line"] + 1
        lines = selected[path]
        if lines is None or any(line in lines for line in range(start, end + 1)):
            violations.append(f"{path}:{start}: [{finding['ruleId']}] {finding['message']}")
    if violations:
        print("CuraEngine structural rule violations:\n" + "\n".join(violations), file=sys.stderr)
    return 1 if violations else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

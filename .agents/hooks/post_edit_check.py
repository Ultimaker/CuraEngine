#!/usr/bin/env python3
"""Check the edited C++ file without requiring Bash on Windows."""

import json
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PATH_KEYS = {"file_path", "filePath", "path", "target_file", "TargetFile", "absolute_path"}
CPP_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hpp"}


def paths(value):
    if isinstance(value, dict):
        for key, item in value.items():
            if key in PATH_KEYS and isinstance(item, str):
                yield item
            else:
                yield from paths(item)
    elif isinstance(value, list):
        for item in value:
            yield from paths(item)


def main():
    try:
        payload = json.load(sys.stdin)
    except (ValueError, OSError):
        return 0

    changed = set()
    tool_input = payload.get("tool_input") or payload.get("toolArgs") or payload
    if isinstance(tool_input, str):
        try:
            tool_input = json.loads(tool_input)
        except ValueError:
            return 0
    for value in paths(tool_input):
        path = (ROOT / value).resolve()
        if path.is_file() and path.is_relative_to(ROOT):
            changed.add(path)
    if not changed:
        return 0

    formatter = shutil.which("clang-format")
    failures = []
    for path in sorted(changed):
        if path.suffix.lower() in CPP_SUFFIXES and formatter:
            result = subprocess.run([formatter, "--dry-run", "--Werror", str(path)], cwd=ROOT, check=False)
            if result.returncode:
                failures.append(str(path.relative_to(ROOT)))
    if failures:
        print("clang-format reported issues in: " + ", ".join(failures), file=sys.stderr)
    structural = subprocess.run(
        [sys.executable, str(ROOT / ".agents/hooks/check_ast_grep.py"), "--working", *(str(path) for path in sorted(changed))],
        cwd=ROOT,
        check=False,
    )
    return 1 if failures or structural.returncode else 0


if __name__ == "__main__":
    sys.exit(main())

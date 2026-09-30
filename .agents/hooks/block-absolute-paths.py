#!/usr/bin/env python3
import os
import subprocess
import sys

HOOKS_DIR = os.path.abspath(os.path.dirname(__file__))
if HOOKS_DIR not in sys.path:
    sys.path.insert(0, HOOKS_DIR)
from path_scanner import PathScanner  # noqa: E402

# Hook may be invoked from .agents/ (Antigravity sets cwd to the hooks.json directory)
_ROOT = subprocess.run(
    ["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True
).stdout.strip()
if _ROOT:
    os.chdir(_ROOT)


def main():
    if PathScanner.scan_staged():
        sys.exit(1)


if __name__ == "__main__":
    main()

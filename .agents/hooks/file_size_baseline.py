#!/usr/bin/env python3
"""
file_size_baseline.py

The ratchet's *state*: reading and writing .agents/file-size-baseline.json, measuring a
file, and deciding whether a merge legitimately raises a ceiling.

Split out of check_file_size_budget.py, which had grown past the very budget it
enforces. Rule 10 applies to the tool that enforces rule 10 — and this is not a
cosmetic split: baseline persistence and merge-parent reconciliation change for
entirely different reasons than the command-line surface does.

The third module is file_size_scope.py, which decides WHICH files the budget
covers. Reading a file's head to classify it belongs there, next to the rules
that judge what it finds — not here, where measuring is about counting lines.
"""

import json
import subprocess
from pathlib import Path

BUDGET = 400
BASELINE_PATH = ".agents/file-size-baseline.json"


def count_lines(path: str, staged: bool = False):
    """Count lines in the content that is actually being judged.

    Under --staged that is the INDEX copy, not the working tree. Selecting
    paths from the index while measuring the working tree lets an oversized
    file through and records a ceiling the commit never met.
    """
    if staged:
        result = subprocess.run(["git", "show", ":{}".format(path)],
                                capture_output=True)
        if result.returncode == 0:
            return result.stdout.count(b"\n") + (
                0 if result.stdout.endswith(b"\n") or not result.stdout else 1)
        return None
    try:
        with open(path, "rb") as handle:
            return sum(1 for _ in handle)
    except OSError:
        return None


def load_baseline() -> dict:
    try:
        with open(BASELINE_PATH, encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, json.JSONDecodeError):
        return {"budget": BUDGET, "policy": "ratchet", "files": {}}


def save_baseline(baseline: dict) -> None:
    with open(BASELINE_PATH, "w", encoding="utf-8") as handle:
        json.dump(baseline, handle, indent=2, sort_keys=True)
        handle.write("\n")


def git_files(*args: str):
    res = subprocess.run(["git", *args], capture_output=True, text=True)
    if res.returncode != 0:
        return []
    return [f for f in res.stdout.splitlines() if f.strip()]


def raise_ceilings_for_merge(baseline: dict) -> None:
    """During a merge, accept growth the incoming branch already had approved.

    A baseline seeded before an upstream merge re-litigates that merge: the
    incoming side legitimately grew a grandfathered file, its own PR gated that
    growth, and the merge commit then fails for code this change never wrote.
    So for a merge commit only, each ceiling rises to the largest size among
    the merge parents. Growth introduced *by the resolution itself* still fails,
    because that exceeds every parent.
    """
    merge_head = Path(".git") / "MERGE_HEAD"
    if not merge_head.exists():
        return
    try:
        parents = ["HEAD"] + merge_head.read_text().split()
    except OSError:
        return
    raised = []
    for name, ceiling in list(baseline.get("files", {}).items()):
        for rev in parents:
            try:
                blob = subprocess.run(["git", "show", f"{rev}:{name}"],
                                      capture_output=True, text=True, check=True)
            except (subprocess.CalledProcessError, OSError):
                continue
            size = len(blob.stdout.splitlines())
            if size > baseline["files"][name]:
                baseline["files"][name] = size
                raised.append((name, ceiling, size))
    for name, was, now in raised:
        print(f"File-size ratchet: merge raises the ceiling for {name} "
              f"({was} -> {now}); the incoming branch already gated that growth.")


def commit_baseline(baseline: dict, grandfathered: dict) -> None:
    baseline["files"] = grandfathered
    save_baseline(baseline)
    subprocess.run(["git", "add", BASELINE_PATH], check=False)

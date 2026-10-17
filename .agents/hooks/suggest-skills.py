#!/usr/bin/env python3
"""
suggest-skills.py

Advisory hook: maps the files about to be edited to the UltiCortex skills that
cover them, so the relevant expertise is loaded BEFORE the code is written.

Runs PRE-edit (`--pre-edit`), reading the target path out of the harness's tool
payload on stdin. Reporting after the edit told the agent it should have loaded
a skill it can no longer act on — advice arriving one turn too late.

Not wired for Claude: Claude resolves skills from the `paths:` front matter in
.claude/rules and loads them itself, so a hook there duplicates the harness.

Never fails a build — steering belongs in the rules; this is the reminder.
"""

import fnmatch
import json
import os
import subprocess
import sys

# skill -> (globs, one-line reason)
SKILL_TRIGGERS = {
    "cpp-pro": (("**/*.cpp", "**/*.cc", "**/*.h", "**/*.hpp"),
                "C++20 ownership, geometry and algorithm changes"),
    "cmake": (("**/CMakeLists.txt", "**/*.cmake"),
              "target and test configuration"),
    "conan-2": (("**/conanfile.py", "**/conandata.yml"),
                "dependency graph, package and profile changes"),
    "software-architect": (("**/Communication.h", "**/BeadingStrategy*.h"),
                           "changes to established architectural seams"),
}

_ROOT = subprocess.run(
    ["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True
).stdout.strip()
if _ROOT:
    os.chdir(_ROOT)


# Keys the supported harnesses use for the path a tool is about to touch.
_PATH_KEYS = ("file_path", "filePath", "path", "target_file", "TargetFile",
              "absolute_path", "notebook_path")


def _paths_in(payload):
    """Every plausible target path in a tool-call payload, at any depth.

    Each harness names the field differently and nests it differently; walking
    the structure is cheaper than maintaining a matrix of shapes that silently
    yields nothing the day one of them renames a key.
    """
    found = []
    if isinstance(payload, dict):
        for key, value in payload.items():
            if key in _PATH_KEYS and isinstance(value, str) and value.strip():
                found.append(value.strip())
            else:
                found.extend(_paths_in(value))
    elif isinstance(payload, list):
        for item in payload:
            found.extend(_paths_in(item))
    return found


def pre_edit_targets():
    """The file this tool call is about to write, read from stdin."""
    try:
        raw = sys.stdin.read() if not sys.stdin.isatty() else ""
    except (OSError, ValueError):
        return []
    if not raw.strip():
        return []
    try:
        payload = json.loads(raw)
    except ValueError:
        return []
    root = os.getcwd() + os.sep
    return [p[len(root):] if p.startswith(root) else p
            for p in _paths_in(payload)]


def changed_files():
    for args in (["diff", "--cached", "--name-only"], ["diff", "--name-only", "HEAD"]):
        res = subprocess.run(["git", *args], capture_output=True, text=True)
        files = [f for f in res.stdout.splitlines() if f.strip()]
        if files:
            return files
    return []


def matches(path: str, glob: str) -> bool:
    """fnmatch has no notion of `**`, and its `*` already spans `/`. A pattern
    anchored with `**/` must therefore also be tried without that prefix, or it
    would never match a file sitting at the repository root."""
    if fnmatch.fnmatch(path, glob):
        return True
    if glob.startswith("**/") and fnmatch.fnmatch(path, glob[3:]):
        return True
    return False


def main():
    pre_edit = "--pre-edit" in sys.argv
    files = pre_edit_targets() if pre_edit else []
    if not files and not pre_edit:
        files = changed_files()
    if not files:
        return
    hits = {}
    for skill, (globs, reason) in SKILL_TRIGGERS.items():
        for path in files:
            if any(matches(path, g) for g in globs):
                hits.setdefault(skill, [reason, []])[1].append(path)
    if not hits:
        return
    print("Relevant UltiCortex skills for the files you are about to change —"
          " load these BEFORE writing the code:" if pre_edit else
          "Relevant UltiCortex skills for the files you are changing:", file=sys.stderr)
    for skill, (reason, paths) in sorted(hits.items()):
        sample = ", ".join(paths[:3]) + (" ..." if len(paths) > 3 else "")
        print("  - {}: {}".format(skill, reason), file=sys.stderr)
        print("      triggered by: {}".format(sample), file=sys.stderr)
        print("      Load the installed /{} skill if available.".format(skill), file=sys.stderr)


if __name__ == "__main__":
    main()

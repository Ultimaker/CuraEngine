#!/usr/bin/env python3
"""
check_complexity.py — cyclomatic complexity ratchet, scoped to what the agent
actually produced.

Two deliberate design choices, both of which avoid defect classes that the
file-size ratchet had to be repaired for:

1. **Only the agent's deliverables are examined.** The input is exactly the
   files changed in this commit or this edit. Pre-existing complexity in code
   nobody touched is not a deliverable and is never reported — a wall of
   complaints about untouched code is how a check gets switched off.

2. **git is the baseline; there is no baseline file.** For each changed file the
   pre-edit content is read from HEAD and the post-edit content from the index
   (or the working tree), and the two are compared function by function. That
   means:
     - no full-tree scan, so no slow bootstrap step on a large repository;
     - no stored ceilings to drift out of sync with the code;
     - no delete-and-recreate bypass, because there is no stored entry to go
       stale;
     - no index-versus-worktree mismatch, because both sides come from the same
       source by construction.

   The trade-off, stated plainly: a rename reads as "old function gone, new
   function added", so a renamed complex function must meet the threshold or
   carry an explicit justification.

Requires `lizard` (pip install lizard). It is language-agnostic and needs no
build, so one hook covers every stack this repository contains. If it is not
installed the hook says so and exits 0 — a missing optional tool must never
block a commit.

Modes:
  --staged           compare HEAD against the index (pre-commit gate)
  --changed [paths]  compare HEAD against the working tree (agent hooks); the
                     optional paths restrict the check to those files, which is
                     what an edit hook passes so that one edit does not re-report
                     every other file changed on the branch
  --report [paths]   planning aid: current complexity of the given files

The gate is proportional to what the change actually did: a function that gets
more complex is only a failure once the RESULT exceeds the budget. Failing on
any increase rejected a 2 -> 3 move nine points inside the budget, which taught
authors to route around the gate rather than to simplify.
"""

import os
import subprocess
import sys
import tempfile
from pathlib import Path

MAX_CCN = 12
CHECKED_SUFFIXES: tuple[str, ...] = (
    '.cc',
    '.cpp',
    '.cxx',
    '.h',
    '.hpp',
    '.py',
)
EXCLUDED_PREFIXES: tuple[str, ...] = (
    '.agents/',
    'build/',
    'tests/',
    'test/',
)
ESTABLISHED_PATTERNS: list[str] = ['Strategy', 'Provider', 'Adapter']

# The agent-tooling directories hold generated hooks. Analysing them would
# report this tool's own complexity as if the agent had just written it.
SELF_MANAGED_PREFIXES = (".agents/", ".claude/", ".opencode/", ".github/")

try:
    _ROOT = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True
    ).stdout.strip()
except (OSError, FileNotFoundError):
    # git absent from PATH: nothing to compare against, so there is nothing to
    # enforce. Never fail a commit because a tool is missing.
    _ROOT = ""
if _ROOT:
    os.chdir(_ROOT)


def is_checked(path: str) -> bool:
    if not path.endswith(CHECKED_SUFFIXES):
        return False
    if path.startswith(SELF_MANAGED_PREFIXES):
        return False
    return not any(path.startswith(p) for p in EXCLUDED_PREFIXES)


def have_lizard() -> bool:
    if not _ROOT:
        return False
    try:
        subprocess.run(["lizard", "--version"], capture_output=True, check=False)
        return True
    except (OSError, FileNotFoundError):
        return False


def git_show(ref: str, path: str):
    """Content of a path at a git ref, or None when it does not exist there."""
    result = subprocess.run(["git", "show", "{}:{}".format(ref, path)],
                            capture_output=True)
    return result.stdout if result.returncode == 0 else None


def changed_paths(staged: bool):
    from_ref = os.environ.get("PRE_COMMIT_FROM_REF")
    to_ref = os.environ.get("PRE_COMMIT_TO_REF")
    if staged and from_ref and to_ref:
        args = ["diff", "--name-only", "--diff-filter=ACMR", f"{from_ref}...{to_ref}"]
    elif staged:
        args = ["diff", "--cached", "--name-only", "--diff-filter=ACMR"]
    else:
        args = ["diff", "--name-only", "--diff-filter=ACMR", "HEAD"]
    result = subprocess.run(["git", *args], capture_output=True, text=True)
    paths = result.stdout.splitlines() if result.returncode == 0 else []
    if not staged:
        # A file the agent has just created is untracked, so `git diff HEAD`
        # does not list it — and that is exactly the file worth checking.
        untracked = subprocess.run(
            ["git", "ls-files", "--others", "--exclude-standard"],
            capture_output=True, text=True)
        if untracked.returncode == 0:
            paths += untracked.stdout.splitlines()
    return [p for p in dict.fromkeys(paths) if p.strip() and is_checked(p)]


def complexity_of(content: bytes, suffix: str) -> dict:
    """Run lizard over one blob and return {function_name: max CCN}.

    lizard reads files, not stdin, so the blob is written to a temporary file
    with the original suffix — the suffix is what selects the language.
    `-i -1` disables lizard's own exit-code gate: this hook owns the verdict.
    """
    if content is None:
        return {}
    with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as handle:
        handle.write(content)
        temp = handle.name
    try:
        result = subprocess.run(["lizard", "--csv", "-i", "-1", temp],
                                capture_output=True, text=True)
    finally:
        os.unlink(temp)
    if result.returncode not in (0, 1):
        return {}

    functions = {}
    for line in result.stdout.splitlines():
        # NLOC,CCN,token,PARAM,length,location,file,name,long_name,start,end
        fields = line.split(",")
        if len(fields) < 8:
            continue
        try:
            ccn = int(fields[1])
        except ValueError:
            continue
        name = fields[7].strip().strip('"')
        if not name:
            continue
        # Overloads share a name; judge the worst of them.
        functions[name] = max(functions.get(name, 0), ccn)
    return functions


def guidance(name: str, ccn: int) -> str:
    pattern_hint = ""
    if ESTABLISHED_PATTERNS:
        names = ", ".join("*" + p for p in ESTABLISHED_PATTERNS)
        pattern_hint = (
            "\n     5. This repository already names its seams ({}). Extract into "
            "one of\n        those shapes rather than inventing a new "
            "one.".format(names))
    return (
        "\n  HOW TO FIX THIS (read before editing):\n"
        "  Do NOT satisfy this check by splitting the function at an arbitrary\n"
        "  point, or by moving branches into a helper that is called once. Both\n"
        "  move the complexity without reducing it, and reviewers will say so.\n\n"
        "  Reduce the number of independent paths through the code:\n"
        "     1. Guard clauses — return early on the error and edge cases so the\n"
        "        main path stops being nested.\n"
        "     2. Replace conditional with polymorphism — if the branching is on a\n"
        "        type or a kind, give each case its own implementation.\n"
        "     3. Table or registry — a long if/elif or switch that maps a value to\n"
        "        an action is data, not control flow.\n"
        "     4. Extract a cohesive step — pull out a named operation that makes\n"
        "        sense on its own and is testable on its own."
        + pattern_hint
        + "\n\n  If this complexity is irreducible, say so explicitly in the pull\n"
        "  request rather than restructuring the code to game the number."
    )


def check(staged: bool, advisory: bool, only=None):
    if not have_lizard():
        print("check_complexity: lizard is not installed; skipping. "
              "Install it with: pip install lizard")
        return 0

    failures, improvements = [], []
    paths = changed_paths(staged)
    if only:
        # An edit hook passes the file it just wrote. Re-examining every file
        # changed since HEAD on every edit reports the same findings about
        # untouched files over and over, which is how output stops being read —
        # and the cost grows with the length of the branch, not the edit.
        wanted = {p.lstrip("./") for p in only}
        paths = [p for p in paths if p in wanted]
    from_ref = os.environ.get("PRE_COMMIT_FROM_REF")
    to_ref = os.environ.get("PRE_COMMIT_TO_REF")
    base_ref = from_ref if (staged and from_ref) else "HEAD"
    for path in paths:
        suffix = Path(path).suffix
        before = complexity_of(git_show(base_ref, path), suffix)
        if staged and to_ref:
            after_blob = git_show(to_ref, path)
        elif staged:
            after_blob = git_show(":0", path)
        else:
            after_blob = (Path(path).read_bytes() if Path(path).is_file() else None)
        after = complexity_of(after_blob, suffix)

        for name, ccn in sorted(after.items()):
            was = before.get(name)
            if was is None:
                if ccn > MAX_CCN:
                    failures.append(
                        "NEW FUNCTION EXCEEDS COMPLEXITY BUDGET: {}\n"
                        "  {}: cyclomatic complexity {} | budget {}\n"
                        "  A function written now must meet the budget outright."
                        .format(path, name, ccn, MAX_CCN) + guidance(name, ccn))
            elif ccn > was and ccn > MAX_CCN:
                # Growth is only a defect once the RESULT is out of budget.
                # This gate used to fail on any increase, so a function moving
                # from 2 to 3 was rejected while sitting nine points inside the
                # budget. That bought nothing and cost a restructuring detour
                # on every ordinary edit. A function ALREADY over budget still
                # may only shrink, which is where the ratchet earns its keep.
                failures.append(
                    "FUNCTION GREW MORE COMPLEX: {}\n"
                    "  {}: cyclomatic complexity {} -> {} (budget {})\n"
                    "  This function was already over budget; this change makes\n"
                    "  it worse. It may become simpler, never more tangled."
                    .format(path, name, was, ccn, MAX_CCN) + guidance(name, ccn))
            elif ccn < was:
                improvements.append("  {} :: {}  {} -> {}".format(path, name, was, ccn))

    if improvements:
        print("Complexity reduced by this change:")
        for line in improvements:
            print(line)

    if failures:
        print("=" * 78)
        for failure in failures:
            print(failure)
            print("-" * 78)
        print("=" * 78)
        return 0 if advisory else 1
    return 0


def report(paths):
    if not have_lizard():
        print("check_complexity: lizard is not installed. "
              "Install it with: pip install lizard")
        return
    if not paths:
        paths = changed_paths(staged=False)
    print("Cyclomatic complexity (budget {} per function):".format(MAX_CCN))
    for path in paths:
        if not is_checked(path) or not Path(path).is_file():
            continue
        functions = complexity_of(Path(path).read_bytes(), Path(path).suffix)
        if not functions:
            continue
        worst = sorted(functions.items(), key=lambda kv: -kv[1])[:5]
        print("  {}".format(path))
        for name, ccn in worst:
            flag = ""
            if ccn > MAX_CCN:
                flag = "  <-- over budget; it may not get worse"
            print("      {:>3}  {}{}".format(ccn, name, flag))
    print("\nFunctions already over budget are grandfathered by their current\n"
          "value: you may leave them alone or improve them, but a change that\n"
          "increases one will be rejected. Plan the extraction before editing.")


def main():
    args = sys.argv[1:]
    if "--report" in args:
        report([a for a in args if not a.startswith("--")])
        return
    staged = "--changed" not in args
    advisory = "--changed" in args
    explicit = [a for a in args if not a.startswith("--")]
    sys.exit(check(staged=staged, advisory=advisory, only=explicit or None))


if __name__ == "__main__":
    main()

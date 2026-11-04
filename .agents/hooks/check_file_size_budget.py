#!/usr/bin/env python3
"""
check_file_size_budget.py

File-size ratchet for an existing codebase.

Two tiers:
  * Files at or under the budget must stay at or under it.
  * Files already over the budget when the ratchet was introduced are
    grandfathered at their recorded size: they may shrink, never grow. When one
    shrinks, its ceiling is tightened so the space cannot be reclaimed later.

The baseline lives in .agents/file-size-baseline.json and is part of the repository.
New files are never grandfathered — a file created after the baseline must meet
the budget outright.

Modes:
  --init            seed the baseline from the current tree (run once)
  --staged          check staged files, tighten ceilings that improved (pre-commit)
  --changed         check files changed against HEAD, report only (agent hooks)
  --report [paths]  planning aid: show remaining headroom for the given paths
"""

import os
import subprocess
import sys
from pathlib import Path

_HOOKS_DIR = os.path.abspath(os.path.dirname(__file__))
if _HOOKS_DIR not in sys.path:
    sys.path.insert(0, _HOOKS_DIR)
# The ratchet is three modules, split because they change for different
# reasons. Baseline persistence, measurement and merge reconciliation live in
# file_size_baseline; the decision about which files are in scope at all, and
# WHY any one of them is exempt, lives in file_size_scope. This file stays what
# it says it is: the command-line surface.
from file_size_baseline import (  # noqa: E402
    BASELINE_PATH,
    BUDGET,
    commit_baseline,
    count_lines,
    git_files,
    load_baseline,
    raise_ceilings_for_merge,
)
from file_size_scope import (  # noqa: E402
    exemption_for,
    is_checked,
)

ESTABLISHED_PATTERNS: list[str] = ['Strategy', 'Provider', 'Adapter']

_ROOT = subprocess.run(
    ["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True
).stdout.strip()
if _ROOT:
    os.chdir(_ROOT)


def decomposition_guidance(path: str) -> str:
    pattern_hint = ""
    if ESTABLISHED_PATTERNS:
        names = ", ".join("*" + p for p in ESTABLISHED_PATTERNS)
        pattern_hint = (
            "\n     5. Follow this repository's own vocabulary — it already uses "
            + names + ".\n        Put the extracted unit where those live"
            " and register it the same way."
        )
    return (
        "\n  HOW TO FIX THIS (read before editing):\n"
        "  Do NOT satisfy this check by deleting blank lines, collapsing formatting,\n"
        "  inlining variables, shortening names, or moving code into comments. Those\n"
        "  reduce the line count without reducing complexity, and will be rejected in\n"
        "  review. Reduce the file by moving responsibility out of it.\n\n"
        "  Work through these in order:\n"
        "     1. Single Responsibility — list the distinct reasons this file has to\n"
        "        change. Each separate reason is a separate module.\n"
        "     2. Open/Closed — find the conditional or switch that grows\n"
        "        every time a\n"
        "        case is added, and replace it with polymorphism (Strategy) or a\n"
        "        registry/Factory, so new cases are added without editing this file.\n"
        "     3. DRY — extract logic repeated here or duplicated elsewhere.\n"
        "     4. Dependency direction — separate I/O, parsing, and\n"
        "        configuration from\n"
        "        the core logic so each part is independently testable."
        + pattern_hint
        + "\n\n  If the extraction genuinely does not belong to the current\n"
        "  ticket, state\n"
        "  that explicitly in the pull request description. Do not bypass this check\n"
        "  silently."
    )


def check(paths, baseline, tighten: bool):
    grandfathered = baseline.get("files", {})
    budget = baseline.get("budget", BUDGET)
    extra = tuple(baseline.get("exclude", []))
    failures, tightened = [], {}

    for path in paths:
        # Under --staged the file may legitimately be absent from the working
        # tree (staged deletion is handled by the prune below).
        if not Path(path).is_file() or not is_checked(path, extra, tighten):
            continue
        lines = count_lines(path, tighten)
        if lines is None:
            continue
        ceiling = grandfathered.get(path)

        if ceiling is None:
            if lines > budget:
                failures.append(
                    "FILE SIZE BUDGET EXCEEDED: {}\n"
                    "  current: {} lines | budget: {} lines | over by: {}\n"
                    "  This file is not grandfathered, so it must meet "
                    "the budget.".format(
                        path, lines, budget, lines - budget)
                    + decomposition_guidance(path))
        elif lines > ceiling:
            failures.append(
                "GRANDFATHERED FILE GREW: {}\n"
                "  current: {} lines | ceiling: {} lines | budget: {} lines\n"
                "  This file was already over budget when the ratchet was introduced,\n"
                "  so it may shrink but never grow. This change adds {} line(s).\n"
                "  Put the new code in a new, properly sized module, or\n"
                "  bring this file\n"
                "  below its ceiling as part of the same change.".format(
                    path, lines, ceiling, budget, lines - ceiling)
                + decomposition_guidance(path))
        elif lines < ceiling:
            tightened[path] = lines

    # Only tighten when the run is otherwise clean. Recording a tighter ceiling
    # for a commit that is about to be rejected would strand the baseline ahead
    # of the tree: if the author then reverts the shrink, the file would be
    # measured against a ceiling it never actually reached.
    if failures:
        tighten = False

    # Drop entries for files that no longer exist. Leaving them behind is a
    # bypass, not a courtesy: deleting a grandfathered path and recreating it
    # would otherwise let a brand-new oversized file inherit the old ceiling.
    removed = [p for p in grandfathered if not Path(p).is_file()]
    if removed and tighten:
        for path in removed:
            grandfathered.pop(path, None)
        commit_baseline(baseline, grandfathered)
        print("File-size ratchet: dropped {} deleted file(s) from the baseline:"
              .format(len(removed)))
        for path in sorted(removed):
            print("  {}".format(path))

    if tightened and tighten:
        for path, lines in tightened.items():
            if lines <= budget:
                grandfathered.pop(path, None)
            else:
                grandfathered[path] = lines
        commit_baseline(baseline, grandfathered)
        print("File-size ratchet tightened (this improvement is now the new ceiling):")
        for path, lines in sorted(tightened.items()):
            print("  {} -> {} lines{}".format(
                path, lines, " (now within budget, no longer grandfathered)"
                if lines <= budget else ""))
    elif tightened and not failures:
        print("File-size ratchet: {} file(s) shrank. Run with --staged to record "
              "the tighter ceilings.".format(len(tightened)))

    return failures


def do_init():
    existing = load_baseline()
    extra = tuple(existing.get("exclude", []))
    tracked = git_files("ls-files")
    files = {}
    for path in tracked:
        if not Path(path).is_file() or not is_checked(path, extra):
            continue
        lines = count_lines(path)
        if lines is not None and lines > BUDGET:
            files[path] = lines
    baseline = {
        "budget": BUDGET,
        "policy": "ratchet",
        "comment": ("Files recorded here were already over budget when the ratchet "
                    "was introduced. They may shrink, never grow. Entries are removed "
                    "automatically once a file falls within budget. Do not add entries "
                    "by hand to excuse a new violation."),
        "exclude": list(extra),
        "exclude_comment": ("Path fragments excluded from the budget entirely — use "
                            "for vendored or generated trees this project does not "
                            "own. After editing, re-run with --init to reseed."),
        "files": files,
    }
    commit_baseline(baseline, files)
    print("Seeded {} with {} grandfathered file(s) over {} lines.".format(
        BASELINE_PATH, len(files), BUDGET))
    if files:
        worst = sorted(files.items(), key=lambda kv: -kv[1])[:5]
        print("Largest:")
        for path, lines in worst:
            print("  {} lines  {}".format(lines, path))
    if len(files) > 50:
        tops = {}
        for path in files:
            top = path.split("/")[0] if "/" in path else "."
            tops[top] = tops.get(top, 0) + 1
        ranked = sorted(tops.items(), key=lambda kv: -kv[1])[:5]
        print("")
        print("WARNING: {} files were grandfathered. That is high enough to suggest "
              "vendored or".format(len(files)))
        print("generated code is being counted as this project's own. Contributors by "
              "top-level directory:")
        for top, count in ranked:
            print("  {}: {} file(s)".format(top, count))
        print("Review the entries; add path fragments to the \"exclude\" list in "
              "{} and re-run".format(BASELINE_PATH))
        print("with --init to reseed. A baseline full of third-party code makes the "
              "ratchet meaningless.")


def _report_whole_tree(extra):
    """Every tracked file the budget covers, plus the ones it silently did not.

    Listing every exempt path here would bury the report under the whole tree's
    documentation and assets, so only the CONTENT-addressed exemptions are named
    individually: those are the ones invisible from a listing, and the ones that
    were found excusing a first-party file. Path-shaped exemptions are
    self-evident from the path, so they are counted rather than enumerated.
    """
    checked, exempt = [], []
    for path in git_files("ls-files"):
        verdict = exemption_for(path, extra)
        if verdict is None:
            checked.append(path)
        elif verdict.content_addressed:
            exempt.append((path, verdict))
    if exempt:
        print("NOT MEASURED — {} file(s) exempted by their content, not their "
              "path:".format(len(exempt)))
        for path, verdict in sorted(exempt):
            print("  {}: {}".format(path, verdict))
        print("")
    return checked


def do_report(paths):
    baseline = load_baseline()
    grandfathered = baseline.get("files", {})
    budget = baseline.get("budget", BUDGET)
    extra = tuple(baseline.get("exclude", []))
    if not paths:
        paths = _report_whole_tree(extra)
    print("File-size headroom (budget {} lines):".format(budget))
    for path in paths:
        # Say why a file is out of scope rather than skipping it in silence.
        # The old report answered "unchecked suffix" for a file whose suffix
        # was fine, which is how a content-addressed exemption stayed hidden.
        exempt = exemption_for(path, extra)
        if exempt is not None:
            print("  {}: NOT MEASURED — {}".format(path, exempt))
            continue
        if not Path(path).is_file():
            print("  {}: new file — must be written within {} lines"
                  .format(path, budget))
            continue
        lines = count_lines(path)
        if lines is None:
            continue
        ceiling = grandfathered.get(path, budget)
        state = "grandfathered" if path in grandfathered else "budget"
        headroom = ceiling - lines
        verdict = ("{} line(s) of headroom".format(headroom) if headroom > 0
                   else "AT CEILING — any addition must be offset by an extraction")
        print("  {}: {} lines / {} {} — {}".format(
            path, lines, ceiling, state, verdict))
    print("\nPlan extractions before writing code: if the work does not fit the "
          "headroom above,\ndecide now which responsibility moves out and where it "
          "goes.")


def main():
    args = sys.argv[1:]
    if "--init" in args:
        do_init()
        return
    if "--report" in args:
        do_report([a for a in args if not a.startswith("--")])
        return

    baseline = load_baseline()
    raise_ceilings_for_merge(baseline)
    if "--changed" in args:
        paths = git_files("diff", "--name-only", "HEAD")
        tighten = False
    else:
        from_ref = os.environ.get("PRE_COMMIT_FROM_REF")
        to_ref = os.environ.get("PRE_COMMIT_TO_REF")
        if from_ref and to_ref:
            paths = git_files("diff", "--name-only", f"{from_ref}...{to_ref}")
        else:
            paths = git_files("diff", "--cached", "--name-only")
        tighten = True
    explicit = [a for a in args if not a.startswith("--")]
    if explicit:
        paths, tighten = explicit, False

    failures = check(paths, baseline, tighten)
    if failures:
        print("=" * 78)
        for failure in failures:
            print(failure)
            print("-" * 78)
        print("=" * 78)
        sys.exit(1)


if __name__ == "__main__":
    main()

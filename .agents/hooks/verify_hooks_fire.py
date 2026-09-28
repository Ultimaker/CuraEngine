#!/usr/bin/env python3
"""
verify_hooks_fire.py — fire-proofing.

"A hook you have not watched fire is unproven." Every rollout PR claimed
"N/N hooks passed 100% cleanly" while shipping hooks that could not execute.
This script actually triggers each blocking guard once, with a deliberately
violating synthetic fixture, and records the observed result in
.agents/bootstrap-profile.json under `hook_verification`. The PR's V&V table
is generated from that record — a pass-count nobody ran becomes unwritable.

Exit non-zero if any exercised guard failed to block its violation.
"""

import json
import os
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

_ROOT = subprocess.run(["git", "rev-parse", "--show-toplevel"],
                       capture_output=True, text=True).stdout.strip()
if _ROOT:
    os.chdir(_ROOT)

HOOKS = Path(".agents/hooks")
PROFILE = Path(".agents/bootstrap-profile.json")

# Fixtures are ASSEMBLED AT RUNTIME, never written out as literals.
#
# This file's whole job is to feed each guard something the guard must reject.
# Spelling those violations out in the source made the guards reject THIS FILE:
# every bootstrap commit then failed its own secret scanner, path scanner and
# adversarial audit, and the tree could not be committed at all. Exempting the
# file from the guards would have been a weakening — a real leaked key in a
# skipped file is exactly what these guards exist to catch. Composing the
# fixtures from fragments keeps the guards at full strength and leaves nothing
# here for them to match.
_PEM_EDGE = "-" * 5
_KEY_WORD = "KEY"
_PRIVATE = "PRIVATE"
FAKE_KEY = (f"{_PEM_EDGE}BEGIN RSA {_PRIVATE} {_KEY_WORD}{_PEM_EDGE}\n"
            f"MIIEvFAKEFAKEFAKE\n{_PEM_EDGE}END RSA {_PRIVATE} {_KEY_WORD}{_PEM_EDGE}\n")
_HOME_ROOT = "/" + "home"
FAKE_PATH = f"config = '{_HOME_ROOT}/exampleuser/secret/config.yaml'\n"


def _run(cmd, **kwargs):
    return subprocess.run(cmd, capture_output=True, text=True, **kwargs)


def _record(results, hook, fired, blocked, detail=""):
    results[hook] = {
        "fired": fired, "blocked": blocked, "detail": detail,
        "at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    status = "BLOCKED" if blocked else ("ran" if fired else "NOT RUN")
    print(f"    [{status:>7}] {hook}{' — ' + detail if detail else ''}")


def _fixture(content, suffix=".py"):
    handle = tempfile.NamedTemporaryFile(
        "w", suffix=suffix, dir=".", prefix=".fireproof-", delete=False)
    handle.write(content)
    handle.close()
    return handle.name


def exercise_file_guard(results, script, content, hook_id):
    """These guards scan the STAGED diff, so the violating fixture is staged
    for the duration of the probe and unstaged afterwards."""
    path = HOOKS / script
    if not path.exists():
        return
    fixture = _fixture(content)
    try:
        _run(["git", "add", "--force", fixture])
        res = _run([sys.executable, str(path)])
        _record(results, hook_id, True, res.returncode != 0,
                "staged violation rejected" if res.returncode != 0
                else "staged violation ACCEPTED")
    finally:
        _run(["git", "reset", "-q", "--", fixture])
        os.unlink(fixture)


def exercise_pretool_guard(results):
    path = HOOKS / "pretool_guard.py"
    if not path.exists():
        return
    payload = json.dumps({
        "hook_event_name": "PreToolUse", "tool_name": "Write",
        "tool_input": {"file_path": "src/x.py", "content": FAKE_KEY},
    })
    res = _run([sys.executable, str(path)], input=payload)
    blocked = res.returncode == 2 or '"deny"' in res.stdout
    _record(results, "pretool-guard", True, blocked,
            "secret payload denied" if blocked else "secret payload ALLOWED")


def exercise_kill_guard(results):
    path = HOOKS / "block_name_matched_kill.py"
    if not path.exists():
        return
    payload = json.dumps({
        "hook_event_name": "PreToolUse", "tool_name": "Bash",
        "tool_input": {"command": "pkill -f my_service"},
    })
    res = _run([sys.executable, str(path)], input=payload)
    blocked = res.returncode == 2 or '"deny"' in res.stdout
    _record(results, "block-name-matched-kill", True, blocked,
            "pkill denied" if blocked else "pkill ALLOWED")


def exercise_run_only(results, script, hook_id, *args):
    """Advisory/analysis hooks must complete successfully."""
    path = HOOKS / script
    if not path.exists():
        return
    res = _run([sys.executable, str(path), *args])
    _record(results, hook_id, True, None if res.returncode == 0 else False,
            f"exit {res.returncode} (advisory check)"
            + (f": {res.stderr[:300]}" if res.returncode else ""))


def exercise_e2e_evidence_gate(results):
    """The E2E-evidence gate (cloud/frontend repos only). Prove BOTH directions
    on synthetic fixtures fed through the hook's testing seam: a user-facing
    diff with an evidence-less body must BLOCK, and the same diff with evidence
    present must PASS. A gate proven only to reject is half-proven."""
    path = HOOKS / "check_e2e_evidence.py"
    if not path.exists():
        return
    changed = _fixture("frontend/src/App.tsx\nsrc/api/JobHandler.py\n", suffix=".txt")
    evidence_less = _fixture(
        "## Why\nFix the print button.\n\n## Verification & Validation\n"
        "Unit tests pass (42/42). No browser run.\n", suffix=".md")
    with_evidence = _fixture(
        "## Why\nFix the print button.\n\n## Verification & Validation\n"
        "Logged in and printed a job end to end:\n\n"
        "![print button enabled](https://github.com/user-attachments/assets/"
        "abc123.png)\n", suffix=".md")
    try:
        rej = _run([sys.executable, str(path),
                    "--changed-paths-file", changed,
                    "--body-file", evidence_less])
        _record(results, "check-e2e-evidence", True, rej.returncode != 0,
                "evidence-less cloud diff rejected" if rej.returncode != 0
                else "evidence-less cloud diff ACCEPTED")
        acc = _run([sys.executable, str(path),
                    "--changed-paths-file", changed,
                    "--body-file", with_evidence])
        # This direction must NOT block; a wrongful block is a false positive.
        _record(results, "check-e2e-evidence-passthrough", True,
                False if acc.returncode != 0 else None,
                "evidence-present cloud diff accepted (exit 0)"
                if acc.returncode == 0
                else "FALSE POSITIVE: evidence present but gate BLOCKED")
    finally:
        for f in (changed, evidence_less, with_evidence):
            os.unlink(f)


def _outcomes(record: dict) -> dict:
    """The parts of a verification record that carry information.

    Excludes `at`, which changes on every run by construction.
    """
    return {
        hook: {k: v for k, v in entry.items() if k != "at"}
        for hook, entry in (record or {}).items()
    }


def main():
    if not HOOKS.is_dir():
        print("No .agents/hooks directory; nothing to verify.")
        return 0
    print("==> Probing guards directly; parity audit separately checks registration")
    results = {}
    exercise_file_guard(results, "block-secrets.py", FAKE_KEY, "block-secrets")
    exercise_file_guard(results, "block-absolute-paths.py", FAKE_PATH,
                        "block-absolute-paths")
    exercise_pretool_guard(results)
    exercise_kill_guard(results)
    exercise_run_only(results, "compile_rule_frontmatter.py",
                      "check-rule-frontmatter", "--check")
    exercise_run_only(results, "audit_quad_agent_parity.py",
                      "audit-quad-agent-parity", ".")
    exercise_run_only(results, "check_upstream_alignment.py",
                      "check-upstream-alignment")
    exercise_e2e_evidence_gate(results)

    head = _run(["git", "rev-parse", "--short", "HEAD"]).stdout.strip()
    for entry in results.values():
        entry["sha"] = head

    if PROFILE.exists():
        try:
            profile = json.loads(PROFILE.read_text())
        except ValueError:
            profile = {}
        # Idempotent: `verify_and_create_pr.sh` runs this on every verification,
        # and rewriting the record with fresh timestamps each time dirtied the
        # working tree and churned the committed profile for no new information.
        # Only the OUTCOMES matter for comparison; `at` is volatile by design.
        if _outcomes(profile.get("hook_verification", {})) == _outcomes(results):
            print(f"    Hook verification unchanged for {head}; "
                  f"{PROFILE} left as-is")
        else:
            profile["hook_verification"] = results
            PROFILE.write_text(json.dumps(profile, indent=2) + "\n")
            print(f"    Recorded {len(results)} result(s) in {PROFILE}")

    failures = [h for h, r in results.items() if r["blocked"] is False]
    if failures:
        print(f"==> FIRE-PROOFING FAILED: guards that did not block: {failures}")
        return 1
    print("==> Fire-proofing complete.")
    return 0


def render_vv_table() -> str:
    """The V&V table for the PR body, generated from the recorded runs."""
    try:
        results = json.loads(PROFILE.read_text()).get("hook_verification", {})
    except (OSError, ValueError):
        results = {}
    if not results:
        return "No hook_verification record — run verify_hooks_fire.py first.\n"
    lines = ["| Hook | Fired | Blocked violation | When |",
             "| --- | --- | --- | --- |"]
    for hook in sorted(results):
        r = results[hook]
        blocked = {True: "yes", False: "NO", None: "n/a (advisory)"}[r["blocked"]]
        lines.append(f"| {hook} | {'yes' if r['fired'] else 'no'} | {blocked} "
                     f"| {r['at']} ({r.get('sha', '?')}) |")
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    if "--vv-table" in sys.argv:
        print(render_vv_table(), end="")
        sys.exit(0)
    sys.exit(main())

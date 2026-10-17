#!/usr/bin/env python3
"""
audit_quad_agent_parity.py

Asserts that every hook a repository qualifies for is present in every platform
rendering that supports hooks.

The required set is derived from the generated configs themselves rather than
hardcoded. A hardcoded list is how the previous version came to require six
hooks while the bootstrap installed eleven — and the five it never checked were
the five added most recently.

Platform support, verified rather than assumed:
  * Antigravity  .agents/hooks.json
  * Claude Code  .claude/settings.json
  * Copilot      .github/hooks/copilot-hooks.json
  * OpenCode     has no command-hook mechanism, so it is checked for
                 configuration presence only; its enforcement floor is
                 pre-commit.
"""

import json
import re
import sys
from pathlib import Path

# Installed beside this script in .agents/hooks/, and beside it in the skill's
# own scripts/ directory, so the same import works in both places.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from copilot_index import check as check_copilot_index  # noqa: E402

CONFLICT_MARKER = re.compile(r"^(<{7} |={7}$|>{7} )", re.MULTILINE)


def load_json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"[X] {path}: cannot be read ({exc})")
        return None


def script_name(command: str) -> str:
    """Reduce a platform-specific command line to the hook script's basename.

    Only managed hooks — commands that route through .agents/hooks/ or hooks/ — take part
    in the parity contract. A platform may additionally wire repository-local
    hooks living elsewhere (scripts/hooks/, an inline shell guard); those speak
    that platform's dialect by construction and demanding they exist on every
    other platform would force either a false failure or a broken port. The
    old last-token heuristic also choked on compound shell commands, reporting
    '}' as a missing hook.
    """
    if not command:
        return ""
    for token in command.split():
        # Explicitly routed through the managed hooks directory (Claude,
        # Copilot: .agents/hooks/), or relative to .agents/ (Antigravity: hooks/).
        if ".agents/hooks/" in token or "hooks/" in token:
            return token.rsplit("/", 1)[-1]
        if "/" not in token and token.endswith((".py", ".sh")):
            return token
    return ""


def antigravity_hooks(repo: Path):
    data = load_json(repo / ".agents" / "hooks.json")
    if data is None:
        return None
    names = set()
    for group in data.values():
        if not isinstance(group, dict):
            continue
        for entries in group.values():
            for entry in entries:
                for hook in entry.get("hooks", []):
                    names.add(script_name(hook.get("command", "")))
    return names - {""}


def claude_hooks(repo: Path):
    data = load_json(repo / ".claude" / "settings.json")
    if data is None:
        return None
    names = set()
    for blocks in (data.get("hooks") or {}).values():
        for block in blocks:
            for hook in block.get("hooks", []):
                names.add(script_name(hook.get("command", "")))
    return names - {""}


def copilot_hooks(repo: Path):
    data = load_json(repo / ".github" / "hooks" / "copilot-hooks.json")
    if data is None:
        return None
    names = set()
    for entries in (data.get("hooks") or {}).values():
        for hook in entries:
            names.add(script_name(hook.get("bash", "")))
    return names - {""}


def platform_commands(repo: Path):
    """Every literal command each platform will execute."""
    commands = []
    data = load_json(repo / ".agents" / "hooks.json") or {}
    for group in data.values():
        if isinstance(group, dict):
            for entries in group.values():
                for entry in entries:
                    commands += [h.get("command", "")
                                 for h in entry.get("hooks", [])]
    data = load_json(repo / ".claude" / "settings.json") or {}
    for blocks in (data.get("hooks") or {}).values():
        for block in blocks:
            commands += [h.get("command", "") for h in block.get("hooks", [])]
    data = load_json(repo / ".github" / "hooks" / "copilot-hooks.json") or {}
    for entries in (data.get("hooks") or {}).values():
        commands += [h.get("bash", "") for h in entries]
    return [c for c in commands if c]


def audit_command_paths(repo: Path) -> bool:
    """Literal path validity. Reducing to basenames is exactly how a doubled
    `.agents/hooks/.agents/hooks/` path passed its own audit."""
    ok = True
    for command in platform_commands(repo):
        for token in command.split():
            if ".agents/hooks/" in token:
                if token.count(".agents/hooks/") != 1:
                    print(f"[X] doubled hook path in command: {command}")
                    ok = False
                elif not (repo / token.removeprefix("./")).exists():
                    print(f"[X] command references a missing script: {command}")
                    ok = False
            elif token.startswith("hooks/") and token.endswith((".py", ".sh")):
                if not (repo / ".agents" / token).exists():
                    print(f"[X] command references a missing script: {command}")
                    ok = False
    return ok


def audit_rule_mirrors(repo: Path) -> bool:
    """Structural rule-set diff across platform dirs, not a count."""
    ok = True
    canonical = {p.name for p in (repo / ".agents" / "rules").glob("*.md")
                 if not p.is_symlink()} if (repo / ".agents" / "rules").is_dir() else set()
    seen_prefix = {}
    for name in sorted(canonical):
        m = re.match(r"^(\d{2})-", name)
        if m:
            if m.group(1) in seen_prefix:
                print(f"[X] duplicate rule prefix {m.group(1)}: {name} and "
                      f"{seen_prefix[m.group(1)]}")
                ok = False
            seen_prefix[m.group(1)] = name
    for mirror in (repo / ".claude" / "rules", repo / ".opencode" / "rules"):
        if not mirror.is_dir():
            continue
        for entry in sorted(mirror.glob("*.md")):
            if entry.name == "agents.md":
                continue
            if entry.is_symlink() and not entry.exists():
                print(f"[X] broken symlink: {entry}")
                ok = False
            elif entry.name not in canonical:
                print(f"[X] platform-only stray rule: {entry} "
                      "(not in .agents/rules/)")
                ok = False
            elif entry.read_bytes() != (repo / ".agents" / "rules" / entry.name).read_bytes():
                print(f"[X] stale rule mirror: {entry}; run scripts/sync_agentic_configs.sh")
                ok = False
        missing = canonical - {e.name for e in mirror.glob("*.md")}
        if missing:
            print(f"[X] {mirror}: missing mirrors for {sorted(missing)}")
            ok = False
    return ok


def audit_copilot_rule_index(repo: Path) -> bool:
    """BOTH directions of the Copilot rule list.

    Copilot has no rule-directory convention: `.github/copilot-instructions.md`
    is the only place it learns which rules exist. The previous check ran one
    way only — every listed rule must exist — so the two failures that actually
    happened in the field went unreported and the audit printed PASSED:

      * a rule present in `.agents/rules/` but absent from the index (a
        repo-owned or custom-band rule, or any rule added after the bootstrap
        run that rendered the list), and
      * an index never refreshed after the rule set changed.

    Missing-from-the-index is the direction that loses enforcement, so it is a
    failure, not a warning.
    """
    problems = check_copilot_index(repo)
    for problem in problems:
        print(f"[X] {problem}")
    if problems:
        print("    The Copilot rule index is stale. Refresh it with "
              "`python3 .agents/hooks/copilot_index.py` (or "
              "`bash scripts/sync_agentic_configs.sh`).")
        return False
    print("[ok] Copilot: rule index matches .agents/rules/ in both directions")
    return True


def _check_duplicate_top_keys(path: Path) -> bool:
    """PyYAML silently last-wins on duplicate top-level keys — the mechanism
    that dropped 8+ hooks from half-merged configs without a parse error."""
    seen, ok = set(), True
    for line in path.read_text(errors="ignore").splitlines():
        m = re.match(r"^([A-Za-z_][\w-]*):", line)
        if m:
            if m.group(1) in seen:
                print(f"[X] {path.name}: duplicate top-level key "
                      f"'{m.group(1)}' — YAML silently drops one side")
                ok = False
            seen.add(m.group(1))
    return ok


def audit_structured_files(repo: Path) -> bool:
    ok = True
    precommit = repo / ".pre-commit-config.yaml"
    if precommit.exists():
        ok &= _check_duplicate_top_keys(precommit)
        try:
            import yaml  # type: ignore
            yaml.safe_load(precommit.read_text(errors="ignore"))
        except ImportError:
            pass
        except Exception as exc:  # noqa: BLE001
            print(f"[X] .pre-commit-config.yaml does not parse: {exc}")
            ok = False
    for rel in (".agents/hooks.json", ".claude/settings.json",
                ".github/hooks/copilot-hooks.json", "opencode.json",
                ".agents/rules-manifest.json", ".agents/bootstrap-profile.json"):
        path = repo / rel
        if path.exists() and load_json(path) is None:
            ok = False
    for rel in (".pre-commit-config.yaml", ".talismanrc", "opencode.json",
                ".agents/hooks.json", ".claude/settings.json"):
        path = repo / rel
        if path.exists() and CONFLICT_MARKER.search(path.read_text(errors="ignore")):
            print(f"[X] {rel}: contains conflict markers")
            ok = False
    return ok


# Parity is the default and a divergence is a defect — EXCEPT where a harness
# already provides the capability natively. Recording the exception here (rather
# than relaxing the check) keeps an accidental omission a failure.
HARNESS_EXEMPT = {
    # Claude reads the `paths:` front matter in .claude/rules and loads the
    # matching skill itself; a hook there duplicates the harness on every edit.
    "suggest-skills.py": ("Claude Code",),
}


def audit_parity(repo_path: Path) -> bool:
    repo = repo_path.resolve()
    print(f"==> Auditing agent-platform parity in: {repo.name}")
    passed = True

    platforms = {
        "Antigravity": antigravity_hooks(repo),
        "Claude Code": claude_hooks(repo),
        "Copilot": copilot_hooks(repo),
    }

    available = [hooks for hooks in platforms.values() if hooks]
    if not available:
        print("[X] No platform hook configuration found — run the bootstrap first.")
        return False

    # The union is what this repository qualifies for; every platform must carry
    # all of it. This catches a hook added to one config and forgotten in another.
    expected: set[str] = available[0].union(*available[1:])
    for name, hooks in platforms.items():
        if hooks is None:
            print(f"[X] {name}: configuration missing or unreadable")
            passed = False
            continue
        missing = {script for script in expected - hooks
                   if name not in HARNESS_EXEMPT.get(script, ())}
        deliberate = sorted((expected - hooks) - missing)
        if missing:
            print(f"[X] {name}: missing {sorted(missing)}")
            passed = False
        else:
            note = (f" (deliberately not wired: {', '.join(deliberate)} — "
                    "the harness provides this natively)" if deliberate else "")
            print(f"[ok] {name}: {len(hooks)} hook(s){note}")

    # Every referenced script must exist, or the config entry is a silent no-op.
    hooks_dir = repo / ".agents" / "hooks"
    for script in sorted(expected):
        if not (hooks_dir / script).exists():
            print(f"[X] {script} is referenced by a platform config "
                  "but is not installed")
            passed = False

    # ...and the reverse: a hook installed but called by nothing is dead code
    # that reads as enforcement. This is how check_upstream_alignment.py sat in
    # .agents/hooks/ enforcing nothing while a PR opened 21 commits behind its
    # base. Scan every place a hook can legitimately be invoked from.
    config_callers = ""
    for caller in (repo / ".pre-commit-config.yaml",
                   repo / "scripts" / "verify_and_create_pr.sh",
                   repo / "scripts" / "sync_agentic_configs.sh",
                   repo / ".claude" / "settings.json",
                   repo / ".agents" / "hooks.json",
                   repo / ".github" / "hooks" / "copilot-hooks.json",
                   repo / "opencode.json"):
        if caller.exists():
            config_callers += caller.read_text(errors="ignore")

    # A hook invoked by a sibling hook is wired too (post-edit-linter.sh calls
    # suggest-skills.py), so sibling sources count as callers — but a script
    # must not vouch for itself, hence the per-script exclusion below.
    hook_sources = {}
    if hooks_dir.is_dir():
        for path in sorted(hooks_dir.iterdir()):
            if path.is_file() and path.suffix in (".py", ".sh"):
                hook_sources[path.name] = path.read_text(errors="ignore")

    # The auditor is an entry point, run by hand and by the PR gate's
    # instructions rather than referenced from a config.
    ENTRY_POINTS = {"audit_quad_agent_parity.py"}

    for script in sorted(hook_sources):
        if script in ENTRY_POINTS:
            continue
        siblings = "".join(src for name, src in hook_sources.items()
                           if name != script)
        stem = script.rsplit(".", 1)[0]
        if script not in config_callers + siblings and stem not in config_callers + siblings:
            print(f"[X] {script} is installed but referenced by no "
                  "config or script — it enforces nothing")
            passed = False

    if not (repo / "opencode.json").exists():
        print("[X] OpenCode: opencode.json missing")
        passed = False
    else:
        print("[ok] OpenCode: configured (no command-hook mechanism; "
              "its enforcement floor is pre-commit)")

    for required in (".pre-commit-config.yaml", ".aiignore", "AGENTS.md"):
        if not (repo / required).exists():
            print(f"[X] {required} is missing")
            passed = False

    passed &= audit_command_paths(repo)
    passed &= audit_rule_mirrors(repo)
    passed &= audit_copilot_rule_index(repo)
    passed &= audit_structured_files(repo)

    print("==> Parity audit " + ("PASSED" if passed else "FAILED"))
    return bool(passed)


def main():
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(".")
    if not audit_parity(target):
        sys.exit(1)


if __name__ == "__main__":
    main()

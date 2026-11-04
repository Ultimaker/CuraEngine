#!/usr/bin/env python3
"""
rule_advisories.py

The checks on the rule set that are HEURISTICS rather than decisions.

These are kept apart from the blocking checks in `compile_rule_frontmatter.py`
deliberately. Each one answers a question that is not decidable from the text —
"does this description earn its context load", "is this enforcement claim
true", "is this sentence a rule or orientation" — so each will occasionally be
wrong. They report; they do not block. When one has run for a cycle and its
false-positive rate is known, promoting it to the blocking registry is a
one-line change, because the seam is here.

A heuristic that blocks from day one turns its first false positive into an
argument about the checker instead of an argument about the guidance.

WHY THESE FOUR. An audit of one repository's 24 generated rules found eight
defects that no mechanism would have caught: a build rule shipping the literal
placeholder "(no test runner detected)"; an OWASP rule prescribing REST auth
and SQL parameterisation for a project with no network surface; a C++ rule
requiring `ctest` where no test was ever registered; a PR rule pointing at a
template path that did not exist; and a UI rule mandating a skill that resolves
nowhere on the machine. Each advisory below is the generalisation of one of
those findings.
"""

from __future__ import annotations

import os
import re
from pathlib import Path

NORMATIVE = re.compile(
    r"\b(MUST NOT|MUST|SHALL NOT|SHALL|NEVER|ALWAYS|STRICTLY FORBIDDEN"
    r"|is forbidden|are forbidden|is blocked|is prohibited)\b")

RULE_POINTER = re.compile(r"(\.agents/rules/|rules?\s+\d{2}\b|rule\s+`?\d{2})",
                          re.IGNORECASE)

ENFORCEMENT = re.compile(
    r"\b(blocks?|blocked|refuses?|prevents?|rejects?|forbids?)\b", re.IGNORECASE)

HOOK_REFERENCE = re.compile(r"[\w/]*hooks?/[\w-]+\.(?:py|sh)")

#: Placeholders a generated rule must never ship with. A rule whose build
#: command is "(no test runner detected)" costs a turn and teaches doubt about
#: every other rule in the set.
#:
#: Deliberately NOT matching a bare `<lowercase words>`. Measured against a
#: real 23-rule set, that shape produced six findings and every one was a
#: false positive: `docker compose build <service>`, `git merge origin/<base>`,
#: `gh pr edit --body-file <updated_template>` — ordinary command-line
#: metavariables, which are exactly how a rule SHOULD write a command the
#: reader must fill in. An advisory that fires on correct documentation is
#: noise, and noise is what stops advisories being read at all.
PLACEHOLDER = re.compile(
    r"(\bTBD\b|\bTODO\b|\bFIXME\b|\(no [a-z ]+ detected\)"
    r"|\bXXX\b|\bPLACEHOLDER\b|<(?:project|repo|repository|insert|your)[ _-])")

#: Skills are named as slash commands and are usually written in backticks, so
#: only a preceding word character or slash (a URL or path) disqualifies a
#: match. A neighbouring slash or angle bracket means it was a filesystem path
#: (`/dev/shm`, `/proc/<pid>/cwd`), and a trailing colon means it was a
#: namespace (`/opsx:*`) rather than a skill name.
#:
#: `*` and `.` are excluded on both sides because a glob is not a skill:
#: `**/conandata.yml` and `**/conanfile.py` were reported as the skills
#: `/conandata` and `/conanfile` against a real rule set, and a rule that
#: declares its own file scope is the single most common place a `/` appears.
SLASH_SKILL = re.compile(r"(?<![\w/>*.])/([a-z][a-z0-9-]{2,})\b(?![/:.])")

#: Words that would make a description a restatement of its own file name
#: rather than a statement of when the rule matters.
_FILLER = {"rules", "rule", "standards", "guidelines", "conventions", "and",
           "for", "the", "of", "in", "this", "repository"}


def skill_search_dirs() -> list[Path]:
    """Every directory a skill name could resolve in on this machine.

    Returns empty when none exist, and the caller then skips the check: a
    developer without a skills directory must not be told every skill the rules
    name is missing.
    """
    home = Path(os.path.expanduser("~"))
    dirs = [Path(".claude/skills"), Path(".skills"), home / ".claude/skills"]
    dirs.extend(sorted((home / ".claude/plugins/cache").glob("*/*/skills")))
    return [d for d in dirs if d.is_dir()]


def check_description_quality(name: str, description: str) -> list[str]:
    """A description that only restates the name cannot earn a model's load."""
    if not description:
        return []
    words = {w.strip(".,`").lower() for w in description.split()}
    meaningful = words - _FILLER - set(name.split("-"))
    if len(meaningful) < 4:
        return ["description adds little beyond the rule name: {!r}".format(
            description)]
    return []


def check_no_placeholders(body: str) -> list[str]:
    """A generated rule that shipped its own template hole is worse than absent."""
    findings = []
    fenced = False
    for number, line in enumerate(body.splitlines(), start=1):
        if line.lstrip().startswith("```"):
            fenced = not fenced
            continue
        if fenced:
            continue
        match = PLACEHOLDER.search(line)
        if match:
            findings.append(
                "line {}: unfilled placeholder {!r} — the bootstrap could not "
                "detect this value, so the rule states nothing actionable"
                .format(number, match.group(0)))
    return findings


def check_skills_resolve(body: str, search_dirs: list[Path]) -> list[str]:
    """A rule that mandates an unavailable skill costs a turn and teaches doubt."""
    if not search_dirs:
        return []
    known = set()
    for parent in search_dirs:
        try:
            known |= {d.name for d in parent.iterdir() if d.is_dir()}
        except OSError:
            continue
    findings = []
    for match in SLASH_SKILL.finditer(body):
        skill = match.group(1)
        if skill not in known:
            findings.append(
                "names skill `/{}`, which resolves in no skill search "
                "path".format(skill))
    return sorted(set(findings))


def check_enforcement_claims(body: str) -> list[str]:
    """Surface every claim that a mechanism blocks something, for audit.

    Whether such a claim is true is not decidable here — a guard can be
    correctly configured and still not intercept the actor the sentence is
    addressed to. So this LISTS the claims rather than judging them, and asks
    for each to name the actor and context it actually covers.
    """
    findings = []
    for number, line in enumerate(body.splitlines(), start=1):
        if not HOOK_REFERENCE.search(line) or not ENFORCEMENT.search(line):
            continue
        findings.append(
            "line {}: enforcement claim — confirm it still holds and that it "
            "names the actor it covers: {}".format(number, line.strip()[:90]))
    return findings


def _normative_lines(text: str) -> list[tuple[int, str]]:
    out = []
    fenced = False
    for number, line in enumerate(text.splitlines(), start=1):
        if line.lstrip().startswith("```"):
            fenced = not fenced
            continue
        if fenced or not NORMATIVE.search(line) or RULE_POINTER.search(line):
            continue
        out.append((number, line.strip()))
    return out


def check_layering(paths: list[Path]) -> list[tuple[Path, str]]:
    """Orientation files point at rules; they do not restate them.

    A duplicated rule drifts from its original, and an agent that finds the
    stale copy follows it. Normative language outside the rule set is therefore
    reported — unless the sentence names the rule it is pointing at.
    """
    findings = []
    for path in paths:
        if not path.is_file():
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        for number, line in _normative_lines(text):
            findings.append((
                path,
                "line {}: normative language outside the rule set — point at "
                "the rule instead: {}".format(number, line[:90])))
    return findings


def advisories_for_rule(name: str, description: str, body: str,
                        search_dirs: list[Path]) -> list[str]:
    """Every heuristic finding for one rule file, in report order."""
    findings = list(check_description_quality(name, description))
    findings.extend(check_no_placeholders(body))
    findings.extend(check_skills_resolve(body, search_dirs))
    findings.extend(check_enforcement_claims(body))
    return findings

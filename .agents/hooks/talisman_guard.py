#!/usr/bin/env python3
"""
talisman_guard.py — the only sanctioned way to change .talismanrc.

The whitelist is generated state. Hand-editing is a documented trap: Talisman
honours the FIRST entry for a filename, so a hand-appended second entry is a
silent no-op. Verbs:

    merge %O %A %B   git union merge driver (semantic union of entries; always
                     exits 0 — a driver that reports conflict reintroduces the
                     textual-conflict problem it exists to remove). Genuinely
                     divergent checksums for the same filename are deferred to
                     `restamp` and reported.
    allow <file>...  add checksum entries for the given files (new findings).
    refresh          re-checksum already-whitelisted files whose content
                     drifted. Deliberately narrower than allow: it never adds
                     an entry, so a genuinely new finding still blocks and
                     still needs a human.
    restamp          resolve entries deferred by a merge.

Every digest comes from `talisman --checksum` itself, one call per path, after
the path has been staged — Talisman's checksum is computed over what git knows
about, so an untracked file yields nothing and a call covering several patterns
yields one collective digest matching none of them. `allow`, `refresh` and
`restamp` therefore stage what they checksum and fail loudly without the
binary; a locally computed hash would look like an exemption while exempting
nothing. `merge` computes no digest and works on any clone.

Register the driver per-clone (done by scripts/sync_agentic_configs.sh and
verify_and_create_pr.sh):
    git config merge.talismanrc.driver \
        "python3 .agents/hooks/talisman_guard.py merge %O %A %B"
GitHub's server-side "mergeable" badge cannot see a repo-local driver: a
CONFLICTING badge on a .talismanrc-only conflict is cosmetic.
"""

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

_ROOT = subprocess.run(["git", "rev-parse", "--show-toplevel"],
                       capture_output=True, text=True).stdout.strip()
if _ROOT:
    os.chdir(_ROOT)

RC = Path(".talismanrc")
DEFERRED = Path(".agents/.talismanrc-deferred")


def parse(text: str):
    """(entries dict, other_sections text). Entries are an unordered map.

    An entry's value is its checksum, or "" when it has none. A checksum-less
    entry is real state — a gitlink carries `ignore_detectors` instead, and a
    file whose digest could not be computed is listed bare on purpose — so it
    must survive a rewrite. Dropping those entries silently re-enabled every
    detector they configured.
    """
    entries = {}
    extras = {}
    other = []
    skip = False
    current = None
    for line in text.splitlines():
        if re.match(r"^fileignoreconfig:", line):
            skip = True
            current = None
            continue
        if re.match(r"^[A-Za-z_][\w]*:", line):
            skip = False
            current = None
        if not skip:
            other.append(line)
            continue
        name_match = re.match(r"^-\s*filename:\s*(\S+)", line)
        if name_match:
            current = name_match.group(1)
            entries[current] = ""
            continue
        if current is None:
            continue
        sum_match = re.match(r"^\s+checksum:\s*(\S+)", line)
        if sum_match:
            entries[current] = sum_match.group(1)
        else:
            extras.setdefault(current, []).append(line)
    return entries, "\n".join(other).strip("\n"), extras


def emit(entries: dict, other: str, extras: dict = None) -> str:
    lines = ["fileignoreconfig:"]
    for name in sorted(entries):
        lines.append(f"- filename: {name}")
        if entries[name]:
            lines.append(f"  checksum: {entries[name]}")
        lines.extend((extras or {}).get(name, []))
    if other:
        lines.append(other)
    return "\n".join(lines) + "\n"


ENTRY_PAIR_RE = re.compile(
    r"-\s*filename:\s*(\S+)\s*\n\s*checksum:\s*([a-fA-F0-9]{64})")


class TalismanMissing(RuntimeError):
    """No talisman binary, so no digest can be produced. Never guess one."""


def checksums(paths):
    """Talisman's own digest for each path: {path: digest}.

    Talisman's digest is not a hash of the file bytes that anything else can
    reproduce, and it is computed from what git knows about — an untracked file
    yields nothing at all. Three hand-rolled approximations in this toolchain
    each wrote digests Talisman rejected, so every exemption they produced was
    inert. Hence: stage first, ask the binary, one call per path (a call with
    several patterns collapses into one collective digest), and refuse to
    invent anything when the binary is absent.
    """
    paths = list(paths)
    if not paths:
        return {}
    if shutil.which("talisman") is None:
        raise TalismanMissing(
            "talisman is not installed, so no checksum can be computed. A "
            "guessed digest looks like an exemption while exempting nothing. "
            "Install talisman (https://github.com/thoughtworks/talisman) and "
            "re-run this command.")
    if subprocess.run(["git", "add", "--", *paths],
                      capture_output=True).returncode != 0:
        for path in paths:
            subprocess.run(["git", "add", "--", path], capture_output=True)
    digests = {}
    for path in paths:
        res = subprocess.run(["talisman", "--checksum", path],
                             capture_output=True, text=True)
        pairs = ENTRY_PAIR_RE.findall(res.stdout)
        for name, digest in pairs:
            if name.strip("'\"") == path:
                digests[path] = digest
                break
        else:
            if len(pairs) == 1:
                digests[path] = pairs[0][1]
    return digests


def merge(base_p, ours_p, theirs_p) -> int:
    read = lambda p: Path(p).read_text(errors="ignore") if Path(p).exists() else ""
    base_e, _, _ = parse(read(base_p))
    ours_e, ours_other, ours_extras = parse(read(ours_p))
    theirs_e, theirs_other, theirs_extras = parse(read(theirs_p))
    merged = dict(ours_e)
    extras = dict(theirs_extras)
    extras.update(ours_extras)
    deferred = []
    for name, digest in theirs_e.items():
        if name not in merged:
            merged[name] = digest
        elif merged[name] != digest:
            if base_e.get(name) == merged[name]:
                merged[name] = digest  # only theirs moved
            elif base_e.get(name) == digest:
                pass  # only ours moved
            else:
                deferred.append(name)  # genuinely divergent: defer, keep ours
    Path(ours_p).write_text(
        emit(merged, ours_other or theirs_other, extras))
    if deferred:
        DEFERRED.parent.mkdir(exist_ok=True)
        with open(DEFERRED, "a") as f:
            f.write("\n".join(deferred) + "\n")
        print(f"talisman_guard: {len(deferred)} divergent checksum(s) deferred "
              "- run: python3 .agents/hooks/talisman_guard.py restamp",
              file=sys.stderr)
    return 0  # never report conflict; that is the whole point


def _report_unresolved(names) -> None:
    print("talisman_guard: talisman produced no checksum for "
          f"{len(names)} path(s); they are NOT exempt: " + ", ".join(sorted(names)),
          file=sys.stderr)


def allow(paths) -> int:
    entries, other, extras = parse(
        RC.read_text(errors="ignore") if RC.exists() else "")
    for path in paths:
        if not Path(path).exists():
            print(f"talisman_guard: no such file: {path}", file=sys.stderr)
            return 1
    digests = checksums(paths)
    unresolved = [p for p in paths if p not in digests]
    if unresolved:
        # Writing the entry anyway would leave a whitelist line that whitelists
        # nothing, which reads as protection at review time.
        _report_unresolved(unresolved)
        return 1
    for path in paths:
        entries[path] = digests[path]
    RC.write_text(emit(entries, other, extras))
    print(f"talisman_guard: whitelisted {len(paths)} file(s).")
    return 0


def refresh() -> int:
    entries, other, extras = parse(
        RC.read_text(errors="ignore") if RC.exists() else "")
    # Only entries that name an existing file: a glob entry has no single file
    # to re-checksum, and `refresh` never adds an entry that is not already here.
    present = [name for name in entries if Path(name).is_file()]
    digests = checksums(present)
    changed = 0
    for name in present:
        digest = digests.get(name)
        if digest and digest != entries[name]:
            entries[name] = digest
            changed += 1
    RC.write_text(emit(entries, other, extras))
    unresolved = [name for name in present if name not in digests]
    if unresolved:
        _report_unresolved(unresolved)
    print(f"talisman_guard: refreshed {changed} drifted entr(y/ies). "
          "New findings still need `allow` and a human.")
    return 1 if unresolved else 0


def restamp() -> int:
    if not DEFERRED.exists():
        print("talisman_guard: nothing deferred.")
        return 0
    names = [n for n in DEFERRED.read_text().splitlines() if n.strip()]
    entries, other, extras = parse(
        RC.read_text(errors="ignore") if RC.exists() else "")
    present = [n for n in names if Path(n).is_file()]
    digests = checksums(present)
    for name, digest in digests.items():
        entries[name] = digest
    RC.write_text(emit(entries, other, extras))
    unresolved = [n for n in present if n not in digests]
    if unresolved:
        _report_unresolved(unresolved)
        return 1
    DEFERRED.unlink()
    print(f"talisman_guard: restamped {len(names)} deferred entr(y/ies).")
    return 0


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    verb = sys.argv[1]
    # `merge` never computes a digest, so it keeps working without talisman —
    # that is what makes it safe as a git merge driver on any clone.
    if verb == "merge" and len(sys.argv) >= 5:
        return merge(sys.argv[2], sys.argv[3], sys.argv[4])
    try:
        if verb == "allow":
            return allow(sys.argv[2:])
        if verb == "refresh":
            return refresh()
        if verb == "restamp":
            return restamp()
    except TalismanMissing as exc:
        print(f"talisman_guard: {exc}", file=sys.stderr)
        return 1
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())

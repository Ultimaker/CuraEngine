#!/usr/bin/env python3
import os
import re
import subprocess
import sys
from pathlib import Path, PurePosixPath

# --- shared detection patterns (generated from ONE source) -----------------
# Every guard that scans content imports these: the pre-flight guard
# (`pretool_guard.py`), the two pre-commit scanners (via `secret_scanner.py`
# and `path_scanner.py`), and the pre-PR gate (`run_adversarial_audit.py`).
#
# WHY THIS PARTIAL EXISTS. Downstream, four guards each carried their own copy
# under a comment claiming the copies were "generated from one source" and so
# could not drift apart. There was no source, and they HAD drifted: the pre-PR
# gate held four of the eight secret shapes — missing PKCS#8 private keys and
# the Slack, OpenAI and AWS credentials — and its GitHub regex matched only
# `ghp_` rather than the whole `gh[pousr]_` family that `gh auth` mints. A
# security policy is knowledge, so it is written once, here, and included.
#
# Do NOT re-declare a credential shape at a call site. Add it here.
#
# One caution for whoever edits this next: a pattern written as a literal token
# rather than a prefix plus a character class would make this catalogue match
# its own source, and the scanners would block the commit that adds it. None of
# the shapes below do — `[` sits outside every character class — and the
# two files this partial is inlined into (`secret_scanner.py`,
# `path_scanner.py`) are already listed in `_self_exempt.py.j2`, so a future
# literal example cannot turn the catalogue into the thing it blocks.

SECRET_PATTERNS = [
    # PKCS#1 / OpenSSH / PGP and the PKCS#8 forms that `openssl genpkey` and
    # `ssh-keygen -m PKCS8` emit by default — the latter were previously missed.
    re.compile(r"-----BEGIN (?:RSA|OPENSSH|DSA|EC|PGP) PRIVATE KEY-----"),
    re.compile(r"-----BEGIN(?: ENCRYPTED)? PRIVATE KEY-----"),
    re.compile(r"AIzaSy[A-Za-z0-9_-]{33}"),          # Google API key
    # The whole `gh` token family, not just `ghp_`: `gh auth` mints `gho_`,
    # `ghu_`, `ghs_` and `ghr_` too, and each is equally a live credential.
    re.compile(r"gh[pousr]_[A-Za-z0-9]{36,}"),        # GitHub tokens
    # GitHub fine-grained PAT
    re.compile(r"github_pat_[0-9a-zA-Z]{22}_[0-9a-zA-Z]{59}"),
    re.compile(r"glpat-[A-Za-z0-9_-]{20}"),           # GitLab PAT
    re.compile(r"xox[baprs]-[A-Za-z0-9-]{10,}"),      # Slack
    re.compile(r"sk-[A-Za-z0-9]{32,}"),               # OpenAI-style
    re.compile(r"\bAKIA[0-9A-Z]{16}\b"),              # AWS access key id
    # AWS secret keys carry no prefix, so they are only recognisable next to
    # the assignment that names them.
    re.compile(r"(?i)aws_secret_access_key\s*=\s*['\"]?([0-9a-zA-Z/+]{40})['\"]?"),
]

HOME_PATH_PATTERN = re.compile(r"/home/[a-zA-Z0-9_-]+/")
USERS_PATH_PATTERN = re.compile(r"/Users/[a-zA-Z0-9_-]+/")
ABSOLUTE_PATH_PATTERNS = [HOME_PATH_PATTERN, USERS_PATH_PATTERN]


# --- self-exemption, by exact identity (generated from one source) ---------
# A guard's own source is the one place in the repository where the pattern it
# forbids MUST appear as a literal: a regex that matches a leaked private key
# necessarily contains a description of a leaked private key. Scanning the
# guards made a freshly bootstrapped repository unable to commit itself.
#
# The exemption is deliberately by EXACT PATH, not by directory prefix.
# Exempting `.agents/hooks/` wholesale would create a blind spot big enough to
# hide a real credential in — the precise failure these guards exist to
# prevent. Every other file under `.agents/`, including new hooks, is scanned
# at full strength, and the fire-proofing fixtures are assembled at runtime
# (see `verify_hooks_fire.py`) rather than exempted.
SELF_EXEMPT_FILES = frozenset({
    ".agents/hooks/secret_scanner.py",
    ".agents/hooks/path_scanner.py",
    ".agents/hooks/block-secrets.py",
    ".agents/hooks/block-absolute-paths.py",
    ".agents/hooks/check_security_downgrades.py",
    ".agents/hooks/pretool_guard.py",
    ".agents/hooks/run_adversarial_audit.py",
    ".agents/hooks/verify_hooks_fire.py",
    "scripts/check_security_downgrades.py",
    "scripts/run_adversarial_audit.py",
})


# `_normalise_diff_path` assumes git's default `a/`/`b/` header prefixes, but
# user config can change them: `diff.mnemonicPrefix` writes `i/`/`w/`,
# `diff.noprefix` writes none, and `diff.external` or `color.diff=always`
# replace the format entirely. Every guard that parses `git diff` headers
# passes these arguments so the output format is fixed regardless of config;
# otherwise a guard's own path stops matching and it flags its own source.
DIFF_FORMAT_ARGS = ("--no-ext-diff", "--no-color",
                    "--src-prefix=a/", "--dst-prefix=b/")


def _normalise_diff_path(path: str) -> str:
    """Strip the `a/` or `b/` prefix git puts on diff headers."""
    path = path.strip()
    if path.startswith(("a/", "b/")):
        path = path[2:]
    return path


def _is_self_exempt(path: str) -> bool:
    return _normalise_diff_path(path) in SELF_EXEMPT_FILES


class PathScanner:
    """Absolute user-home paths, reported with the offending path itself.

    Structural parsing rather than a bare regex: `PurePosixPath` tells us
    whether the token really is an absolute path under a user home, which is
    what lets the message name the path a developer has to remove instead of
    only the line it sat on.

    The shared catalogue (`ABSOLUTE_PATH_PATTERNS`) is still consulted, as a
    cross-check in the one direction that matters. If the catalogue recognises
    a shape the structural extractor missed, that is a FALSE NEGATIVE in this
    scanner, and a silently unenforced rule is worse than a noisy one — so the
    line is reported anyway and the mismatch is called out by name.
    """

    PATH_CANDIDATE_REGEX = re.compile(
        r"(?:/(?:[a-zA-Z0-9_.-]+/)+[a-zA-Z0-9_.-]*"
        r"|/home/[a-zA-Z0-9_-]+|/Users/[a-zA-Z0-9_-]+)"
    )

    @classmethod
    def extract_path_candidates(cls, text: str) -> list[str]:
        return cls.PATH_CANDIDATE_REGEX.findall(text)

    @classmethod
    def evaluate_path_string(cls, candidate: str) -> dict:
        is_absolute = False
        is_forbidden_user_path = False
        try:
            p = PurePosixPath(candidate)
            if p.is_absolute():
                is_absolute = True
                parts = p.parts
                if (len(parts) >= 3 and parts[0] == "/"
                        and parts[1] in ("home", "Users")):
                    is_forbidden_user_path = True
        except (ValueError, TypeError):
            pass
        return {
            "is_absolute": is_absolute,
            "is_forbidden_user_path": is_forbidden_user_path,
        }

    @classmethod
    def scan_line(cls, line: str) -> tuple[bool, list[str]]:
        """(violation_found, offending_paths) for a single line of content."""
        bad = [c for c in cls.extract_path_candidates(line)
               if cls.evaluate_path_string(c)["is_forbidden_user_path"]]
        if bad:
            return True, bad
        # Catalogue cross-check. Reaching here means the shared patterns saw a
        # user-home path that structural extraction did not; report it rather
        # than let it through, and name the gap so it gets closed.
        for pattern in ABSOLUTE_PATH_PATTERNS:
            match = pattern.search(line)
            if match:
                return True, ["{} (matched by the shared catalogue only — "
                              "PathScanner.extract_path_candidates missed it)"
                              .format(match.group(0))]
        return False, []

    @classmethod
    def check_file_existence(cls, rel_path_str: str, root_dir: Path = None) -> bool:
        root = root_dir or Path(os.getcwd())
        try:
            target = (root / rel_path_str).resolve()
            return target.exists()
        except (OSError, ValueError):
            return False

    @classmethod
    def scan_staged(cls) -> bool:
        diff_cmd = subprocess.run(
            ["git", "diff", "--cached", "-U0", *DIFF_FORMAT_ARGS],
            capture_output=True,
            text=True,
            check=False,
        )
        if diff_cmd.returncode != 0:
            return False

        found_forbidden = False
        skip_file = False
        for line in diff_cmd.stdout.splitlines():
            if line.startswith("+++ "):
                target = line[4:].strip()
                skip_file = _is_self_exempt(target)
                continue
            if skip_file:
                continue
            if line.startswith("+"):
                # Routed through scan_line so the staged-commit gate and the
                # pre-flight guard (which calls scan_line directly) can never
                # disagree about what counts as a violation.
                violated, offenders = cls.scan_line(line[1:])
                for candidate in offenders:
                    print(
                        "❌ PathScanner blocked commit: detected hardcoded "
                        f"user path: {candidate}",
                        file=sys.stderr,
                    )
                found_forbidden = found_forbidden or violated

        return found_forbidden

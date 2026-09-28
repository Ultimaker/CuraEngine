#!/usr/bin/env python3
import os
import re
import subprocess
import sys

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


class SecretScanner:
    @classmethod
    def scan_line(cls, line: str) -> list[str]:
        findings = []
        for pattern in SECRET_PATTERNS:
            match = pattern.search(line)
            if match:
                findings.append(match.group(0))
        return findings

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

        found_secrets = False
        skip_file = False
        for line in diff_cmd.stdout.splitlines():
            if line.startswith("+++ "):
                target = line[4:].strip()
                skip_file = _is_self_exempt(target)
                continue
            if skip_file:
                continue
            if line.startswith("+"):
                findings = cls.scan_line(line[1:])
                if findings:
                    print(
                        "❌ SecretScanner blocked commit: detected hardcoded "
                        f"secret token: {findings[0][:8]}...",
                        file=sys.stderr,
                    )
                    found_secrets = True

        return found_secrets

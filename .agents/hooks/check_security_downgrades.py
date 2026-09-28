#!/usr/bin/env python3
"""
check_security_downgrades.py
----------------------------
Deterministic pre-commit & pre-PR hook script to detect and block security
feature downgrades in staged diffs.
"""

import re
import subprocess
import sys

# Deliberately BROAD. A false positive costs one turn — the author looks at the
# line and says "no, that is a test fixture". A missed downgrade ships an agent's
# workaround for a failing check straight into the product. The asymmetry is not
# close, so these patterns are written to over-match on purpose.
#
# Two families:
#   * `_OFF` — a named security control being set to a disabling value.
#   * explicit entries — flags and calls whose mere presence is the downgrade.
_OFF = r"\s*[:=]\s*(?:false|0|none|null|off|no|\"\"|'')\b"

_DISABLED_CONTROLS = [
    # Authentication and authorisation
    ("signInRequired", "sign-in requirement disabled"),
    ("needs?_authentication", "authentication requirement disabled"),
    ("require[_a-z]*auth[a-z_]*", "authentication requirement disabled"),
    ("auth[a-z_]*required", "authentication requirement disabled"),
    ("authenticat(?:e|ed|ion)", "authentication disabled"),
    ("authoriz(?:e|ed|ation)", "authorization disabled"),
    ("login_?required", "login requirement disabled"),
    ("check_?permissions?", "permission checking disabled"),
    ("permission_?check", "permission checking disabled"),
    ("enforce[_a-z]*", "an enforcement switch disabled"),
    ("scope_?check|check_?scopes?", "scope checking disabled"),
    ("csrf[_a-z]*|xsrf[_a-z]*", "CSRF protection disabled"),
    # Transport and certificate validation
    ("ssl_?verify|verify_?ssl|tls_?verify|verify_?certs?", "TLS verification disabled"),
    ("verify", "verification disabled (often TLS certificate validation)"),
    ("check_?hostname", "TLS hostname checking disabled"),
    ("reject_?unauthorized", "unauthorized TLS peers accepted"),
    ("secure|https_?only|http_?only|strict_?transport",
     "a transport-security flag disabled"),
    ("validate[_a-z]*|validation", "validation disabled"),
    ("sanitiz|sanitis|escape[_a-z]*", "input sanitisation disabled"),
    # Sandboxing / isolation
    ("sandbox(?:ed|ing)?", "sandboxing disabled"),
    ("web_?security|same_?origin|cors_?check", "browser origin protection disabled"),
    ("read_?only|readonly", "a read-only protection removed"),
    ("signature_?check|check_?signature|verify_?signature",
     "signature verification disabled"),
    ("encrypt(?:ed|ion)?", "encryption disabled"),
]

SECURITY_DOWNGRADE_PATTERNS = [
    # The trailing suffix allowance matters: `SANDBOX_ENABLED = False` is the
    # same downgrade as `sandbox = false`, and anchoring on the bare name missed it.
    # The suffix and closing-quote allowances matter: `SANDBOX_ENABLED = False`
    # and `config['CSRF_ENABLED'] = 0` are the same downgrade as `sandbox: false`,
    # and anchoring on the bare name missed both.
    (r"\b(?:" + name + r")[_a-z0-9]*['\"\]]*" + _OFF, description)
    for name, description in _DISABLED_CONTROLS
] + [
    # Presence, not assignment: these ARE the downgrade wherever they appear.
    (r"NODE_TLS_REJECT_UNAUTHORIZED\s*[:=]\s*['\"]?0", "Node TLS validation disabled"),
    (r"PYTHONHTTPSVERIFY\s*[:=]\s*['\"]?0", "Python HTTPS validation disabled"),
    (r"curl[^\n]*\s(?:-k|--insecure)\b", "curl invoked with certificate checks off"),
    (r"(?:wget)[^\n]*--no-check-certificate", "wget certificate checking disabled"),
    (r"ssl\._create_unverified_context", "unverified SSL context created"),
    (r"InsecureRequestWarning|disable_warnings", "TLS warnings suppressed"),
    (r"CERT_NONE", "certificate validation set to CERT_NONE"),
    (r"--no-verify\b|--no-gpg-sign\b", "a commit/push verification step bypassed"),
    (r"SKIP\s*=\s*\S+.*(?:secret|talisman|security|audit)",
     "a security pre-commit hook added to SKIP"),
    (r"(?:#|//)\s*(?:nosec|noqa:\s*S\d|bandit:\s*skip|semgrep-disable|"
     r"trivy:ignore|codeql\[[^\]]*\])", "a security linter suppressed inline"),
    (r"eslint-disable[^\n]*security", "an ESLint security rule disabled"),
    (r"strict\s*[:=]\s*false", "a strict-mode switch disabled"),
    (r"trust[_a-z]*\s*[:=]\s*true", "a trust-everything switch enabled"),
    (r"allow[_a-z]*(?:insecure|unsafe|any|all)[a-z_]*\s*[:=]\s*true",
     "an allow-insecure switch enabled"),
    (r"(?:chmod\s+(?:777|a\+rwx))", "world-writable permissions granted"),
]

# Second tier: patterns that are frequently legitimate but are also exactly how
# a weakening arrives. Blocking on these would make ordinary work impossible, so
# they are REPORTED and the agent judges them in the same turn. Silence would be
# the worse error; a hard failure would be the more annoying one.
ADVISORY_PATTERNS = [
    (r"@ts-(?:ignore|nocheck)", "TypeScript checking suppressed"),
    (r"dangerouslySetInnerHTML|\.innerHTML\s*=", "raw HTML injection sink"),
    (r"shell\s*=\s*True", "subprocess invoked through a shell"),
    (r"\beval\s*\(|\bexec\s*\(|new\s+Function\s*\(", "dynamic code execution"),
    (r"pickle\.loads?\s*\(|yaml\.load\s*\((?![^)]*Safe)", "unsafe deserialisation"),
    (r"subprocess\.[a-z_]+\([^)]*\+", "subprocess argument built by concatenation"),
    (r"(?:SELECT|INSERT|UPDATE|DELETE)\b[^\n]*(?:%s|\+\s*[a-z_]|\bf\")",
     "SQL assembled by string building rather than parameterised"),
    (r"\ballow_?origins?\s*[:=]\s*\[?\s*['\"]\*", "CORS opened to any origin"),
]


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


def get_staged_diff() -> str:
    try:
        res = subprocess.run(
            ["git", "diff", "--cached", "-U0", *DIFF_FORMAT_ARGS],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True
        )
        return res.stdout
    except Exception:
        return ""


def main():
    diff = get_staged_diff()
    if not diff:
        sys.exit(0)

    added = []          # (path, added line)
    current = "?"
    skip_file = False
    for line in diff.splitlines():
        if line.startswith("+++ "):
            current = line[4:].strip()
            skip_file = _is_self_exempt(current)
            continue
        if skip_file:
            continue
        if line.startswith("+"):
            added.append((current, line[1:]))

    violations, advisories = [], []
    for path, text in added:
        for pattern, description in SECURITY_DOWNGRADE_PATTERNS:
            if re.search(pattern, text, re.IGNORECASE):
                violations.append(f"  ❌ {description}\n     {path}: {text.strip()}")
                break
        for pattern, description in ADVISORY_PATTERNS:
            if re.search(pattern, text, re.IGNORECASE):
                advisories.append(
                    f"  ⚠️ {description}\n     {path}: {text.strip()}")
                break

    if advisories:
        print("\n🔍 SECURITY REVIEW REQUESTED (advisory — not a failure):")
        for advisory in advisories:
            print(advisory)
        print("\nThese patterns are often legitimate. Judge each one now, in this "
              "turn, and say in the commit or PR why it is safe here.\n")

    if violations:
        print("\n🔒 SECURITY GUARD VIOLATION DETECTED:")
        for violation in violations:
            print(violation)
        print("\nAI agents are strictly forbidden from disabling security or "
              "authentication features to bypass test/verification roadblocks.")
        print("These patterns are deliberately broad: if this is a false positive, "
              "say so explicitly rather than rewording the line to slip past the "
              "check. Otherwise revert the downgrade and fix the underlying "
              "configuration or test issue properly.\n")
        sys.exit(1)

    sys.exit(0)


if __name__ == "__main__":
    main()

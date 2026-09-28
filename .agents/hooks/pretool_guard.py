#!/usr/bin/env python3
"""
pretool_guard.py — pre-flight gate for agent tool calls.

This runs BEFORE a tool executes and inspects the *pending* call: the content
about to be written, or the command about to run. That is the whole point. The
pre-commit scanners look at the index, which by definition does not yet contain
what the agent is about to do, so they can audit but they cannot prevent.

Three platform contracts, all verified against the platforms themselves rather
than assumed. They differ in both the input shape and how a block is signalled:

  Claude Code   in : {"tool_name": "Write", "tool_input": {...}, "cwd": ...}
                out: {"hookSpecificOutput": {"hookEventName": "PreToolUse",
                      "permissionDecision": "deny",
                      "permissionDecisionReason": "..."}}
                NOTE: exit code 1 is explicitly NON-blocking in Claude Code —
                it logs the error and proceeds. Only exit 2, or an explicit
                deny decision, actually stops the call.

  Antigravity   in : {"toolCall": {"name": "write_to_file",
                      "args": {"TargetFile": ..., "CodeContent": ...}}}
                out: {"decision": "deny", "reason": "..."}

  Copilot       in : {"toolName": ..., "toolArgs": {...}}
                out: {"permissionDecision": "deny",
                      "permissionDecisionReason": "..."}

Fail open, never closed: a payload this script cannot understand must not block
the agent's work. A guard that halts every tool call the moment a platform
changes its schema gets switched off, and then nothing is guarded at all.
"""

import json
import os
import re
import subprocess
import sys

HOOKS_DIR = os.path.abspath(os.path.dirname(__file__))
if HOOKS_DIR not in sys.path:
    sys.path.insert(0, HOOKS_DIR)
from secret_scanner import SecretScanner  # noqa: E402
from path_scanner import PathScanner  # noqa: E402

FORBIDDEN_BRANCHES = ("main", "master", "staging")

# Commands that produce history. The branch guard applies to these only:
# blocking every Bash call on a protected branch would stop `git status` and
# the agent could not even diagnose its way out.
COMMIT_COMMAND = re.compile(r"\bgit\s+(?:commit|push|merge|rebase)\b")

# Tool names that write file content, per platform.
WRITE_TOOLS = {
    "write", "edit", "multiedit", "notebookedit",            # Claude Code
    "write_to_file", "replace_file_content",                  # Antigravity
    "multi_replace_file_content", "create_file", "edit_file",
    "str_replace_editor", "applypatch", "apply_patch",
}
SHELL_TOOLS = {"bash", "run_command", "shell", "terminal", "runcommand"}

# Keys that may carry a path or content, across every platform's arg naming.
PATH_KEYS = ("file_path", "filePath", "path", "TargetFile", "target_file",
             "notebook_path", "filename")
CONTENT_KEYS = ("content", "CodeContent", "code_content", "new_string",
                "new_str", "text", "contents", "ReplacementContent",
                "new_source", "patch", "Patch")
COMMAND_KEYS = ("command", "CommandLine", "cmd", "commandLine", "script")


def collect_strings(value, out, depth=0):
    """Antigravity's replace_file_content nests replacement chunks, so the
    content is not always at a predictable top-level key."""
    if depth > 6:
        return
    if isinstance(value, str):
        out.append(value)
    elif isinstance(value, dict):
        for v in value.values():
            collect_strings(v, out, depth + 1)
    elif isinstance(value, list):
        for v in value:
            collect_strings(v, out, depth + 1)


def parse_request(payload):
    """Normalise the three payload shapes into (platform, tool, path, blobs)."""
    if "toolCall" in payload:                      # Antigravity
        platform = "antigravity"
        call = payload.get("toolCall") or {}
        tool = (call.get("name") or "").lower()
        args = call.get("args") or {}
    elif "toolName" in payload:                    # Copilot
        platform = "copilot"
        tool = (payload.get("toolName") or "").lower()
        args = payload.get("toolArgs") or {}
    elif "tool_name" in payload:                   # Claude Code
        platform = "claude"
        tool = (payload.get("tool_name") or "").lower()
        args = payload.get("tool_input") or {}
    else:
        return None, None, None, []

    if isinstance(args, str):
        try:
            args = json.loads(args)
        except json.JSONDecodeError:
            args = {"command": args}
    if not isinstance(args, dict):
        args = {}

    path = next((args[k] for k in PATH_KEYS
                 if isinstance(args.get(k), str)), None)

    blobs = []
    if tool in SHELL_TOOLS:
        for key in COMMAND_KEYS:
            if isinstance(args.get(key), str):
                blobs.append(args[key])
    else:
        for key in CONTENT_KEYS:
            if isinstance(args.get(key), str):
                blobs.append(args[key])
        if not blobs and tool in WRITE_TOOLS:
            # Fall back to a deep scan rather than silently checking nothing.
            collect_strings(args, blobs)
    return platform, tool, path, blobs


def current_branch():
    result = subprocess.run(["git", "rev-parse", "--abbrev-ref", "HEAD"],
                            capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else ""


# Generated/compiled files with an owning CLI. Hand-editing them is a silent
# trap (Talisman honours only the FIRST entry for a filename, the compiled
# exclusion targets are overwritten by the compiler, the manifest is the
# regeneration ledger), so edits are denied and redirected at the tool.
MANAGED_FILE_OWNERS = {
    ".talismanrc": "python3 .agents/hooks/talisman_guard.py allow|refresh",
    ".ignore": "edit .aiignore, then python3 .agents/hooks/compile_aiignore.py",
    ".github/copilot-content-exclusion.yml":
        "edit .aiignore, then python3 .agents/hooks/compile_aiignore.py",
    ".agents/rules-manifest.json": "re-run the agentic bootstrap",
    "GEMINI.md": "edit AGENTS.md, then bash scripts/sync_agentic_configs.sh",
}


def managed_file_violation(tool, path):
    if tool in SHELL_TOOLS or not path:
        return None
    normalised = path.replace("\\", "/").removeprefix("./")
    for managed, owner in MANAGED_FILE_OWNERS.items():
        if normalised == managed or normalised.endswith("/" + managed):
            return (f"{managed} is generated state and must never be "
                    f"hand-edited (edits silently no-op or are overwritten). "
                    f"Use the owning command instead: {owner}")
    return None


def find_violation(tool, path, blobs):
    """Return a human-readable reason to block, or None to allow."""
    managed = managed_file_violation(tool, path)
    if managed:
        return managed
    for blob in blobs:
        for line in blob.splitlines():
            if SecretScanner.scan_line(line):
                return ("This change contains what looks like a credential "
                        "(private key, API token, or passphrase). Secrets "
                        "must never be written into the repository.")

    # Absolute home paths are only meaningful in file content; a shell command
    # legitimately references absolute paths all the time.
    if tool not in SHELL_TOOLS:
        for blob in blobs:
            for line in blob.splitlines():
                has_viol, bad_paths = PathScanner.scan_line(line)
                if has_viol:
                    return ("This change hardcodes an absolute local path "
                            "({!r}). Use a path relative to the repository "
                            "root, or resolve it at runtime."
                            .format(bad_paths[0]))

    if tool in SHELL_TOOLS:
        for blob in blobs:
            if COMMIT_COMMAND.search(blob):
                branch = current_branch()
                if branch in FORBIDDEN_BRANCHES:
                    return ("Refusing to run a history-changing git command on "
                            "'{}'. Create a feature branch named after the "
                            "active ticket first.".format(branch))
    return None


def deny(platform, reason):
    if platform == "antigravity":
        print(json.dumps({"decision": "deny", "reason": reason}))
    elif platform == "copilot":
        print(json.dumps({"permissionDecision": "deny",
                          "permissionDecisionReason": reason}))
    else:
        print(json.dumps({"hookSpecificOutput": {
            "hookEventName": "PreToolUse",
            "permissionDecision": "deny",
            "permissionDecisionReason": reason,
        }}))
    sys.exit(0)


def main():
    try:
        try:
            raw = sys.stdin.read()
        except (OSError, ValueError):
            return
        if not raw.strip():
            return
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return

        platform, tool, path, blobs = parse_request(payload)
        if platform is None:
            return

        reason = find_violation(tool, path, blobs)
        if reason:
            deny(platform, reason)
    except Exception:
        # Fail open, never closed: an unhandled exception inside the guard must
        # not block the agent's work.
        return
    # Silence means "no opinion" on every platform: the normal permission flow
    # continues. Never print an allow decision — that would override the user's
    # own settings.


if __name__ == "__main__":
    main()

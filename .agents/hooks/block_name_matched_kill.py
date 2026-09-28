#!/usr/bin/env python3
"""
block_name_matched_kill.py

Refuses to kill a process chosen by name pattern.
"""

import json
import re
import shlex
import sys

BY_NAME = ("pkill", "killall")
SUBSTITUTION = re.compile(r"\$\(|`|\$\{")

REASON = """Refused: this kills processes selected by name, not by PID.

Worktrees on this machine share a repository, a Godot binary and a build cache,
so a name match reaches other sessions' processes as readily as your own.

Do this instead:

    pgrep -af <pattern>              # find candidates (pgrep -fl on macOS)
    ls -l /proc/<pid>/cwd            # confirm it is YOUR worktree (Linux)
    lsof -a -p <pid> -d cwd          # same, on macOS
    kill <pid>                       # signal that one, by number

If the process is not in your worktree, it is not yours to signal."""


def offending_command(command: str) -> str | None:
    """Return the reason this command is refused, or None if it is fine."""
    try:
        tokens = shlex.split(command)
    except ValueError:
        tokens = command.split()

    for index, token in enumerate(tokens):
        name = token.rsplit("/", 1)[-1]
        if name in BY_NAME:
            return f"`{name}` selects processes by name"
        if name == "kill":
            rest = tokens[index + 1:]
            targets = [t for t in rest if not t.startswith("-")]
            if not targets:
                continue
            if any(SUBSTITUTION.search(t) for t in targets):
                return "`kill` is being given a command substitution, not a PID"
            if not all(t.isdigit() for t in targets):
                return "`kill` is being given something that is not a literal PID"
    return None


# This hook is wired into all four harnesses, and they neither deliver the tool
# call nor read a decision in the same shape. Reading only Claude's
# `tool_input.command` and always printing Antigravity's `{"decision": "deny"}`
# meant that under Copilot and Antigravity the hook could not even see the
# command: it FAILED OPEN and blocked nothing at all, while appearing wired.
#
# Extraction and emission are therefore both per-harness, and the harness is
# inferred from the payload rather than configured, so one script stays correct
# in every configuration that references it.

def extract_command(payload: dict) -> str:
    """The shell command from any supported harness's tool-call payload."""
    # Claude Code: {"tool_name": "Bash", "tool_input": {"command": ...}}
    command = (payload.get("tool_input") or {}).get("command")
    if command:
        return command
    # GitHub Copilot: {"toolName": ..., "toolArgs": {"command": ...}}
    args = payload.get("toolArgs") or payload.get("tool_args") or {}
    if isinstance(args, dict) and args.get("command"):
        return args["command"]
    # Antigravity: {"toolCall": {"args": {"command"|"CommandLine": ...}}}
    call_args = ((payload.get("toolCall") or payload.get("tool_call") or {})
                 .get("args") or {})
    if isinstance(call_args, dict):
        for key in ("command", "CommandLine", "commandLine"):
            if call_args.get(key):
                return call_args[key]
    return ""


def detect_harness(payload: dict) -> str:
    if payload.get("tool_input") is not None or payload.get("tool_name"):
        return "claude"
    if payload.get("toolArgs") is not None or payload.get("toolName"):
        return "copilot"
    if payload.get("toolCall") is not None or payload.get("tool_call"):
        return "antigravity"
    return "claude"


def deny_payload(harness: str, reason: str) -> dict:
    """The refusal in the schema THIS harness actually reads."""
    if harness == "claude":
        return {
            "hookSpecificOutput": {
                "hookEventName": "PreToolUse",
                "permissionDecision": "deny",
                "permissionDecisionReason": reason,
            }
        }
    if harness == "copilot":
        return {"permissionDecision": "deny", "permissionDecisionReason": reason}
    return {"decision": "deny", "reason": reason}


def main() -> int:
    try:
        payload = json.load(sys.stdin)
    except (json.JSONDecodeError, ValueError):
        return 0  # never block because the hook could not parse its own input

    command = extract_command(payload)
    if not command:
        return 0

    # A whole-command substitution can hide the verb; check the raw text too.
    reason = offending_command(command)
    if reason is None and SUBSTITUTION.search(command):
        for name in BY_NAME:
            if re.search(rf"\b{name}\b", command):
                reason = f"`{name}` selects processes by name"
                break

    if reason is None:
        return 0

    harness = detect_harness(payload)
    print(json.dumps(deny_payload(harness, f"{reason}.\n\n{REASON}")))
    # Exit 2 as well: Claude Code treats a non-zero PreToolUse exit as a block
    # even when it does not parse the JSON, so the refusal cannot be lost.
    return 2


if __name__ == "__main__":
    sys.exit(main())

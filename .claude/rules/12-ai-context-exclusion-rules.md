---
name: ai-context-exclusion
description: What must never be read by an AI agent in this repository, how exclusion is enforced, and proactive .aiignore maintenance.
trigger: always_on
---
# AI Context Exclusion

`.aiignore` at the repository root is the single source of truth for files that
must not enter a model's context: secrets, third-party code and SDKs, build
output, large binaries, and anything carrying personal data.

## How it is enforced

No agent platform reads `.aiignore` natively. `.agents/hooks/compile_aiignore.py`
translates it into the mechanism each platform actually honours:

| Platform | Mechanism |
|---|---|
| Antigravity / ripgrep-based search | `.ignore` (generated) |
| Claude Code | `permissions.deny` `Read(./…)` rules in `.claude/settings.json` |
| OpenCode | `permission.read` / `glob` / `grep` deny map in `opencode.json` |
| GitHub Copilot | org-level content exclusion, applied server side — paste `.github/copilot-content-exclusion.yml` into GitHub settings |

## Rules for agents

1. **Never read, quote, or summarise a file matching `.aiignore`.** If a task
   appears to require one, stop and say so rather than working around the
   exclusion.
2. **Never weaken the exclusion to finish a task** — do not delete patterns,
   add negations, or bypass the derived deny rules.
3. **Edit `.aiignore`, never the generated targets.** `.ignore` and
   `.github/copilot-content-exclusion.yml` are overwritten by the compiler, and
   the deny rules in the platform configs are rewritten in place.
4. **After changing `.aiignore`, run the compiler and commit the results
   together**, or pre-commit will reject the change as out of date:

   ```bash
   python3 .agents/hooks/compile_aiignore.py
   ```
5. **Copilot exclusion is not active until a human applies it in GitHub.**
   Adding a secret pattern to `.aiignore` does not retroactively hide it from
   Copilot; treat any exposed credential as compromised and rotate it.
6. **Proactive AI-Ignore Maintenance**:
   Maintain `.aiignore` whenever introducing new heavy, generated, log, or vendor artifacts. Run `python3 .agents/hooks/compile_aiignore.py` immediately to recompile platform deny rules.

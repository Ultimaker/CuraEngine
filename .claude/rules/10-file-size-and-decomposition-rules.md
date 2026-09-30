---
name: file-size-and-decomposition
description: File-size budget with grandfathering ratchet, and the decomposition expected to meet it.
trigger: always_on
---
# File Size Budget & Decomposition Rules

A large file is expensive for every agent that reads it afterwards. This
repository enforces a budget of **400 lines**, with a ratchet so that
existing large files are not a blocker but can never get worse.

## 1. The Two Tiers

1. **Files within budget** must stay at or under 400 lines.
2. **Files already over budget** when the ratchet was introduced are recorded in
   `.agents/file-size-baseline.json` at their size at that moment. They **may
   shrink but must never grow**. When one shrinks, its ceiling tightens
   automatically — the reclaimed space cannot be spent later.
3. **New files are never grandfathered.** A file created from now on must meet
   the budget outright.

Check status at any time:

```bash
python3 .agents/hooks/check_file_size_budget.py --report <paths>
```

## 2. During Design and Planning — Before Writing Code

Treat the budget as a design input, not a gate you discover at commit time.

- Run the `--report` command above on every file the change is expected to
  touch, and read the headroom before deciding where code goes.
- If the planned work does not fit the headroom, the plan must say **which
  responsibility moves out, where it goes, and what the new module is called**.
  Decide this during design; do not defer it until the hook fails.
- When a task's natural home is a file already at its ceiling, the default
  answer is a new module, not an exception.
- State the intended decomposition in the implementation plan and in the pull
  request description, so a reviewer sees the structural intent rather than an
  unexplained new file.

## 3. Meeting the Budget Honestly

Reducing the line count without reducing complexity is a violation of this rule,
even when the number goes down. The following are **not** acceptable ways to
pass the check:

- deleting blank lines or collapsing formatting
- inlining variables, shortening identifiers, or packing statements onto one line
- moving code into comments, or relocating it to an already-oversized file
- disabling or excluding the check for the file

Reduce the file by moving responsibility out of it:

1. **Single Responsibility (SRP)** — enumerate the distinct reasons the file has
   to change. Each separate reason belongs in its own module.
2. **Open/Closed (OCP)** — find the conditional or `switch` that grows whenever a
   case is added, and replace it with polymorphism (Strategy) or a
   registry/Factory, so future cases are added without editing this file.
3. **DRY** — extract logic that is repeated inside the file or duplicated
   elsewhere in the codebase.
4. **Dependency direction** — separate I/O, parsing, and configuration from core
   logic so each side is testable on its own.
5. **Interface Segregation / composition** — split a class that serves several
   callers with disjoint needs, rather than growing one wide interface.
   - This repository already names its seams: `*Strategy`, `*Provider`, `*Adapter`. Extract into those shapes and register the new unit the same way the existing ones are registered.

Load the **`software-architect`** skill when deciding how to split a file: it
carries the SOLID guidance and the design-pattern catalogue (Facade, Strategy,
Observer, Factory) that these steps refer to. Reach for it during design, not
after the hook rejects the commit.

## 4. Enforcement

- **Pre-commit** blocks a commit that pushes a file over budget or grows a
  grandfathered file, and tightens ceilings for files that shrank.
- **Agent PostToolUse hooks** report the same violation immediately after an
  edit, so the problem surfaces while the context is still open.
- `.agents/file-size-baseline.json` is committed. Do **not** hand-edit the
  `files` map to excuse a violation; entries are removed automatically once a
  file is within budget.

Generated files (`@generated`, `DO NOT EDIT` headers) and third-party code
(recognised vendor copyright headers, `vendor/`, `third_party/`, submodules) are
neither checked nor grandfathered — they are not ours to decompose. If vendored
code still slips into the baseline, add a path fragment to the `exclude` list in
`.agents/file-size-baseline.json` and re-run:

```bash
python3 .agents/hooks/check_file_size_budget.py --init
```

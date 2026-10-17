---
name: complexity-budget
description: Cyclomatic complexity budget for code the agent writes, ratcheted against git history.
trigger: always_on
---
# Complexity Budget

A function with many independent paths is hard to test, hard to review, and the
place bugs accumulate. This repository budgets **cyclomatic complexity 12
per function** — chosen from this codebase's own distribution (median 2, 90th percentile 11, worst 105).

## 1. What Is Actually Checked

Only **what you change**. The hook compares each changed file against its
version in git, function by function:

- a **new** function must be at or under 12;
- an **existing** function may become simpler, never more complex;
- a function you do not touch is never reported, however complex it is.

There is no repository-wide scan and no stored baseline — git is the baseline.
Pre-existing complexity is not your deliverable, but you may not add to it.

```bash
python3 .agents/hooks/check_complexity.py --report <paths>
```

## 2. During Design and Planning

Run the report on the functions a change will touch before deciding how to
implement it. If the work adds branching to a function that is already at or
over budget, the plan must say **which paths move out and where** — a new
strategy, a lookup table, a separate step. Decide that during design, not when
the hook rejects the commit.

## 3. Reducing Complexity Honestly

Splitting a function at an arbitrary line, or moving branches into a helper
called from exactly one place, moves complexity without reducing it. Both will
be rejected in review even though the number drops.

Reduce the number of independent paths:

1. **Guard clauses** — return early on error and edge cases so the main path
   stops being nested.
2. **Replace conditional with polymorphism** — when branching is on a type or a
   kind, give each case its own implementation.
3. **Table or registry** — a long `if`/`elif` chain mapping a value to an action
   is data, not control flow.
4. **Extract a cohesive step** — a named operation that makes sense on its own
   and can be tested on its own.

5. **Use the seams this repository already has** — `*Strategy`, `*Provider`, `*Adapter`. Extract into
   one of those shapes rather than inventing a new one.

If the complexity is genuinely irreducible — a parser, a state machine, a
hardware protocol — say so explicitly in the pull request. That is a reviewable
claim; silently restructuring the code to game the metric is not.

## 4. Enforcement

- **Pre-commit** blocks a commit that adds a function over budget or makes an
  existing function more complex.
- **PostToolUse** reports the same immediately after an edit, advisory only, so
  the problem surfaces while the context is still open.
- Requires `lizard` (`pip install lizard`). Without it the check reports that it
  is skipping and exits cleanly — it never blocks a commit because a tool is
  missing.

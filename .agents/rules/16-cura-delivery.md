---
name: cura-delivery
description: Follow CuraEngine's existing PR template, Conan release pipeline, and branch history without inventing tickets.
trigger: always_on
---
# Delivery and build evidence

Use `.github/PULL_REQUEST_TEMPLATE.md` for PR descriptions: describe the change, test command and host OS; preserve its human checklist. Use the actual ticket if one was supplied or already attached to the branch; do not fabricate `CURA-123` or enforce a Jira key on unrelated existing history. The repo's recent history includes CURA and NP tickets as well as non-ticket commits (`git log --format=%s`), and keeps merge commits; do not rewrite published branches to satisfy a generic convention. Open agent-authored PRs as drafts and leave merge approval to people.

Conan 2 (`conanfile.py`, `conandata.yml`) is the dependency source of truth; CMake consumes its generated toolchain and dependencies. Before adding code or a dependency, inspect an existing standard-library operation, declared Conan package, and relevant implementation in `Ultimaker/conan-ultimaker-index` or related Cura repositories on GitHub (or local checkouts when available). Build and test commands depend on a configured Conan profile and `tools.build:skip_test`; do not claim `ctest --test-dir build` or `pytest -x -q` is a verified universal command. For changes in build, packaging or CI inspect the delegated jobs in `Ultimaker/cura-workflows` and the platform profiles in `Ultimaker/conan-config`; validate native Linux, Windows and macOS consumers and the Emscripten target when affected. Avoid mandatory Bash-only steps for Windows contributors; Python-based checks run under an installed Python on all three native platforms.

---
name: build
description: Build CuraEngine (conan install, CMake configure and compile) and run the unit tests, e.g. to check that code changes compile without errors or warnings.
---

# Building CuraEngine

CuraEngine is built with conan2 + CMake, using the presets that conan generates. This works on Linux, macOS and Windows.
Follow these steps in order, and stop at the first one that fails.

Never assume a specific shell: check which one your shell tool uses (sh/bash on Linux and macOS; PowerShell, cmd or Git
Bash on Windows) and use the matching syntax and script variant below.

## 1. Make sure conan is available

Developers install conan in different ways (virtualenv, pipx, system package...), so never assume a specific location.

1. Look for a personal, git-ignored activation script at the repository root, and run it first. Pick the one matching
   the current shell:
    * `.conan-env.sh` for sh/bash/zsh (`. ./.conan-env.sh`; use the `.` form because `source` does not exist in plain `sh`)
    * `.conan-env.ps1` for PowerShell (`. .\.conan-env.ps1`)
    * `.conan-env.bat` for cmd (`call .conan-env.bat`)
2. Run `conan --version`. It must report a 2.x version.
3. If conan is still not found, **do not install it and do not guess a path**. Ask the user how to activate their conan
   2 environment, and suggest they create the script matching their shell for future sessions. If they prefer, they can
   also start OpenCode from a shell where conan is already activated, because the environment is inherited.

Each shell command runs in a fresh shell, so chain the environment activation and the command in the same invocation (
`&&` in sh/cmd, `;` or `&&` in PowerShell 7).

## 2. Install dependencies (only on request)

Installing dependencies is slow and is rarely needed, so do **not** run `conan install` on your own. Go straight to step
3, and only **suggest** the install and **ask the user for confirmation** when one of these applies:

* The conan generators folder (`build/<Type>/generators`, or `build/generators`) does not exist, so nothing can be
  configured yet.
* `conanfile.py`, `conandata.yml` or `CMakeLists.txt` changed (check with `git status` / `git diff`), or are newer than
  the generated `CMakePresets.json`.
* The configure or build step fails with a dependency-related error: missing package or header, a `find_package`
  failure, a conan-generated file that is missing or outdated, or a version mismatch.

When suggesting it, say which of these reasons applies. Once the user agrees, prefer the Debug configuration for
checking code issues:

```
conan install . --build=missing --update -s "&:build_type=Debug" -c tools.build:skip_test=False -o "&:enable_benchmarks=True" -o "with_curaviz=True"
```

For Release, drop the `-s "&:build_type=Debug"` setting.

## 3. Configure and build

The conan-generated `CMakePresets.json` files are included from `CMakeUserPresets.json`. Run `cmake --list-presets` to
see the exact preset names, since they depend on the platform and generator. Usually:

* Linux/macOS (single-config generators): presets `conan-debug` / `conan-release`, generators in
  `build/Debug/generators` and `build/Release/generators`.
* Windows (multi-config Visual Studio generator): preset `conan-default` to configure, and `conan-debug` /
  `conan-release` to build, with generators in `build/generators`.

Load the conan build environment before running cmake (in the generators folder):

* sh/bash: `. build/Debug/generators/conanbuild.sh`
* cmd: `call build\generators\conanbuild.bat`
* PowerShell: `& build\generators\conanbuild.ps1`

Then, for example on Linux/macOS:

```
cmake --preset conan-debug
cmake --build --preset conan-debug
```

and on Windows:

```
cmake --preset conan-default
cmake --build --preset conan-debug
```

Building can take a long time, so run it with a generous timeout or in the background.

## 4. Unit tests (only on request)

Do **not** run the unit tests systematically after a build. Instead, look at the changed code and check whether there
are relevant tests, for instance in `tests/` for the changed class or module. If there are, tell the user which tests
are relevant and **ask whether they want them to be run**. If there are none, just say so and do not ask.

Once the user agrees, run only the relevant ones, e.g.:

```
ctest --preset conan-debug --output-on-failure -R <regex>
```

Only run the whole test suite if the user explicitly asks for it.

## Reporting

* Report compiler errors and warnings that come from files that were changed, with their file and line.
* Ignore warnings from third-party headers.
* Never edit `build/`, `CMakeUserPresets.json` or the conan-generated files by hand; regenerate them with the commands
  above.

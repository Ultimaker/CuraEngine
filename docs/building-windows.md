# Building CuraEngine on Windows

CuraEngine uses Conan 2 and CMake. Its Conan recipe accepts newer MSVC versions;
there is no VS 2022-only compiler check. The shared
[UltiMaker Conan configuration](https://github.com/Ultimaker/conan-config) uses
Ninja and inherits the compiler from the detected `default` profile.

## Choose a compiler

Install the **Desktop development with C++** workload, its MSVC tools, and a
Windows SDK. These instructions target Windows x86_64 with the selected IDE's
native toolset:

| Visual Studio | Conan `compiler.version` | Toolset | Developer command prompt |
| --- | --- | --- | --- |
| 2022 | `193` (MSVC 19.3x) or `194` (MSVC 19.4x) | `v143` | x64 Native Tools Command Prompt for VS 2022 |
| 2026 | `195` (MSVC 19.5x) | `v145` | x64 Native Tools Command Prompt for VS 2026 |

Existing VS 2022 users can keep their current tools, Conan home, profiles, and
build commands. Adding VS 2026 does not require migrating that environment.

For VS 2026, use Python 3.12 or newer, Conan **2.24 or newer within Conan 2**, CMake
**4.2 or newer**, and Ninja. The older `conan==2.7.1` command in the
[general build guide](https://github.com/Ultimaker/CuraEngine/wiki/Building-CuraEngine-From-Source)
predates MSVC 195. Conan 2.24 includes a
[VS 2026 detection fix](https://docs.conan.io/2/changelog.html).
CMake 4.2 also adds the
[Visual Studio 18 2026 generator](https://cmake.org/cmake/help/latest/generator/Visual%20Studio%2018%202026.html).
The commands below retain the shared configuration's Ninja generator, which can
also be used when opening the checkout as a folder in Visual Studio.

## Build with the Windows helper

The optional `scripts/build_windows.py` launcher checks the active compiler and
prerequisite tool versions, installs the shared Conan configuration into a
compiler-specific cache on first use, detects a profile there, and invokes the
existing `conan build` recipe. Activate a Python environment containing Conan,
CMake, and Ninja, then run from the matching **x64 Native Tools Command Prompt**:

```bat
rem VS 2022 is the default. Use Conan 2.24+ in the helper environment.
python scripts\build_windows.py
python scripts\build_windows.py --vs 2022 --build-type Debug
```

From the VS 2026 prompt, with Conan >=2.24,<3 and CMake >=4.2 installed:

```bat
python scripts\build_windows.py --vs 2026
python scripts\build_windows.py --vs 2026 --build-type Debug --with-tests
```

The helper requires Conan >=2.24,<3 for **both** compiler selections so it can
disable root preset generation, and CMake >=3.23 for VS 2022. Keep an older
VS 2022 Conan environment for direct builds if needed; create a separate Python
environment for the helper instead of upgrading it in place. It rejects a
mismatched compiler instead of silently selecting another installation. It sets
`CC`/`CXX` and `CONAN_HOME` only for its subprocesses. It does not modify the
caller's Conan home, profiles, or root `CMakeUserPresets.json`.

Helper outputs are isolated under `build/windows-vs2022` and
`build/windows-vs2026`; the existing recipe places binaries in the nested
`build/Release` or `build/Debug` directory. Its Conan cache is `conan-home` under
the corresponding output root. Both compiler paths can therefore use the same
checkout. Do not run two helper builds for the same compiler concurrently.

`--with-tests` enables compilation of the unit tests; run them separately after a
successful build. For the VS 2026 Debug example above:

```bat
call build\windows-vs2026\build\Debug\generators\conanrun.bat
ctest --test-dir build/windows-vs2026/build/Debug --output-on-failure --no-tests=error
build\windows-vs2026\build\Debug\CuraEngine.exe help
```

Use `windows-vs2022` or `Release` for the other selections. Errors from Conan,
CMake, or compilation stop the helper with a nonzero exit status. The helper's
own portable checks can be run with
`python -m unittest discover -s scripts -p "test_*.py"`; they simulate external
tools and do not replace native Windows build validation.

## Manual setup and builds

The direct Conan/CMake path remains available below. It uses the active shell's
Conan home and generates root presets, so keep its compiler checkouts separate.
The helper already isolates these files and does not require a separate checkout.

## Set up a separate VS 2026 environment

Use a separate checkout for each compiler so generated presets and CMake caches
do not overwrite those of an existing VS 2022 build. Open the **x64 Native Tools
Command Prompt for VS 2026** and change to the new checkout. All command blocks
below use **cmd.exe**, not PowerShell.

Create a Python environment and Conan home dedicated to this compiler. Choose
unused paths if these names already contain another environment:

```bat
py -3.12 -m venv "%USERPROFILE%\.venvs\cura-vs2026"
call "%USERPROFILE%\.venvs\cura-vs2026\Scripts\activate.bat"
python -m pip install "conan>=2.24,<3" "cmake>=4.2" ninja
set "CONAN_HOME=%USERPROFILE%\.conan2-cura-vs2026"
conan config install https://github.com/Ultimaker/conan-config.git
set "CC=cl"
set "CXX=cl"
where cl
conan profile detect --force
conan profile show
```

If using a newer Python, replace `-3.12` with its installed version. `CC` and
`CXX` make Conan detect `cl` from the chosen developer prompt, rather than select
another installed Visual Studio. Verify that both host and build profiles show
`os=Windows`, `arch=x86_64`, `compiler=msvc`, and `compiler.version=195`. The host
profile must also retain `curaengine*:compiler.cppstd=20` from `cura.jinja`.
If the compiler is wrong, stop and reopen the matching developer prompt before
regenerating the profile. Do not label a VS 2022 compiler as `195` manually.

For a new, isolated **VS 2022** environment, use its developer prompt and replace
`cura-vs2026` with `cura-vs2022` in the paths above. The newer Conan 2 tooling also
recognizes VS 2022; verify `compiler.version=193` or `194` instead. This optional
setup does not change the tools required by an existing VS 2022 environment.

In subsequent sessions, open the same developer prompt, activate the matching
Python environment, and set its `CONAN_HOME`, `CC`, and `CXX` again. Installing the
configuration and detecting the profile are only needed during setup or after an
intentional toolchain change. Close the prompt before switching compiler versions.

## Build Release or Debug

Run from the checkout root, stopping if any command fails. Release:

```bat
conan install . --build=missing --update
cmake --preset conan-release
cmake --build --preset conan-release
```

Debug uses a separate configuration directory:

```bat
conan install . --build=missing --update -s build_type=Debug
cmake --preset conan-debug
cmake --build --preset conan-debug
```

To run the Release executable with dependency DLLs available:

```bat
call build\Release\generators\conanrun.bat
build\Release\CuraEngine.exe help
```

Use `Debug` instead of `Release` for the Debug executable. Use a fresh developer
prompt when changing configurations so runtime DLL paths from the previous build
are not retained.

## Validate both compiler paths

The shared Conan configuration skips tests by default. To enable the existing
CuraEngine unit tests, repeat installation and configuration with testing enabled:

```bat
conan install . --build=missing -c tools.build:skip_test=False
cmake --preset conan-release
cmake --build --preset conan-release
call build\Release\generators\conanrun.bat
ctest --test-dir build/Release --output-on-failure --no-tests=error
build\Release\CuraEngine.exe help
```

Run this in each compiler's separate checkout/environment. For Debug tests, add
`-s build_type=Debug` to `conan install`, select `conan-debug` for both CMake
commands, and use `build\Debug` for the runtime script, tests, and executable.
Record the compiler, Conan, and CMake versions with the results. Native Windows
builds and tests with both VS 2022 and VS 2026 are needed before treating a new
VS 2026 setup as validated; profile detection alone does not prove compatibility.

## Troubleshooting

- **MSVC 195 is not a valid setting:** check `conan --version`, `where conan`, and
  `conan config home`. VS 2026 needs the newer Conan installation and its own
  configuration, not the older VS 2022 environment.
- **Visual Studio 18 is not installed:** install VS 2026's C++ build tools or use
  the VS 2022 environment/profile. Changing only the version number in a profile
  does not install a compiler.
- **CMake reports a generator or compiler mismatch:** use the separate checkout
  for that compiler. Do not reuse a build directory created by another toolchain.
- **A dependency fails:** `--build=missing` compiles packages without a matching
  binary; it cannot fix an incompatible dependency. Save the first failing
  package's name/version and error. Resolve that failure before claiming a full
  CuraEngine build, without changing the working VS 2022 environment.

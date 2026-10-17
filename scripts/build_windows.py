# Copyright (c) 2026 UltiMaker
# CuraEngine is released under the terms of the AGPLv3 or higher.
"""Build with the selected MSVC toolchain from its x64 Native Tools prompt."""

import argparse
import hashlib
import os
from pathlib import Path
import re
import subprocess
import sys


def runCommand(command: list[str], source: Path, environment: dict[str, str], capture: bool = False) -> str:
    result = subprocess.run(
        command,
        cwd=source,
        env=environment,
        check=True,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.STDOUT
    )
    return result.stdout or ""


def getVersion(command: list[str], source: Path, environment: dict[str, str]) -> tuple[int, int, int]:
    output = runCommand(command, source, environment, capture=True)
    match = re.search(r"\b(\d+)\.(\d+)\.(\d+)\b", output)
    if not match:
        raise RuntimeError("Cannot read version from {}: {}".format(command[0], output.strip()))
    return int(match[1]), int(match[2]), int(match[3])


def getConanStorage(source: Path, visual_studio: str, environment: dict[str, str]) -> Path:
    local_app_data = environment.get("LOCALAPPDATA", "")
    if not local_app_data or not Path(local_app_data).is_absolute():
        raise RuntimeError("Cannot find LOCALAPPDATA for the isolated Conan package cache.")
    source_id = hashlib.sha256(str(source.resolve()).casefold().encode("utf-8")).hexdigest()[:12]
    return Path(local_app_data) / "CuraEngine" / "conan-storage" / source_id / "vs{}".format(visual_studio)


def buildWindows(source: Path, visual_studio: str, build_type: str, with_tests: bool) -> None:
    if sys.platform != "win32":
        raise RuntimeError("Run this script on Windows from an x64 Native Tools Command Prompt.")
    environment = os.environ.copy()
    if environment.get("VSCMD_ARG_TGT_ARCH", "").lower() != "x64":
        raise RuntimeError("Open the x64 Native Tools Command Prompt for VS {}.".format(visual_studio))

    compiler = getVersion(["cl", "/?"], source, environment)
    valid_compiler = compiler[0] == 19 and (
        30 <= compiler[1] < 50 if visual_studio == "2022" else 50 <= compiler[1] < 60)
    if not valid_compiler:
        raise RuntimeError("Active MSVC {} does not match VS {}. Open its developer prompt.".format(
            ".".join(map(str, compiler)), visual_studio))
    visual_studio_path = environment.get("VSINSTALLDIR", "")
    if not visual_studio_path or not Path(visual_studio_path).is_dir():
        raise RuntimeError("Cannot find the active Visual Studio installation. Reopen its developer prompt.")
    visual_studio_path = str(Path(visual_studio_path))

    output = source / "build" / "windows-vs{}".format(visual_studio)
    conan_storage = getConanStorage(source, visual_studio, environment)
    # Keep profile detection and generated files away from existing direct Conan builds.
    environment["CONAN_HOME"] = str(output / "conan-home")
    environment["CC"] = "cl"
    environment["CXX"] = "cl"
    conan = getVersion(["conan", "--version"], source, environment)
    minimum_conan = (2, 24, 0)
    if conan[0] != 2 or conan < minimum_conan:
        raise RuntimeError("VS {} requires Conan >= {} and < 3 for this build helper.".format(
            visual_studio, ".".join(map(str, minimum_conan))))
    minimum_cmake = (3, 23, 0) if visual_studio == "2022" else (4, 2, 0)
    if getVersion(["cmake", "--version"], source, environment) < minimum_cmake:
        raise RuntimeError("VS {} requires CMake >= {} for this build helper.".format(
            visual_studio, ".".join(map(str, minimum_cmake))))
    runCommand(["ninja", "--version"], source, environment)

    profiles = Path(environment["CONAN_HOME"]) / "profiles"
    if not all((profiles / name).is_file() for name in ("cura.jinja", "cura_build.jinja")):
        runCommand(["conan", "config", "install", "https://github.com/Ultimaker/conan-config.git"],
                   source, environment)
    runCommand(["conan", "profile", "detect", "--force"], source, environment)
    runCommand([
        "conan", "build", str(source), "--build=missing", "--output-folder", str(output),
        "-cc", "core.cache:storage_path={}".format(conan_storage),
        "-pr:h", "cura.jinja", "-pr:b", "cura_build.jinja", "-s:h", "build_type={}".format(build_type),
        "-c:h", "tools.cmake.cmaketoolchain:generator=Ninja",
        "-c:b", "tools.cmake.cmaketoolchain:generator=Ninja",
        "-c:h", "tools.cmake.cmaketoolchain:user_presets=",
        "-c:h", "&:tools.build:skip_test={}".format(not with_tests),
        "-c:h", "tools.microsoft.msbuild:installation_path={}".format(visual_studio_path),
        "-c:b", "tools.microsoft.msbuild:installation_path={}".format(visual_studio_path),
    ], source, environment)
    print("Build completed: {}".format(output / "build" / build_type))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vs", choices=("2022", "2026"), default="2022", help="Visual Studio version (default: 2022)")
    parser.add_argument("--build-type", choices=("Release", "Debug"), default="Release")
    parser.add_argument("--with-tests", action="store_true", help="Build unit tests as well as CuraEngine")
    args = parser.parse_args()
    try:
        buildWindows(Path(__file__).resolve().parents[1], args.vs, args.build_type, args.with_tests)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print("Build failed: {}".format(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

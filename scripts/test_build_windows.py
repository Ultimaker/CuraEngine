# Copyright (c) 2026 UltiMaker
# CuraEngine is released under the terms of the AGPLv3 or higher.

import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import build_windows


class WindowsBuildTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.source = Path(self.temporary.name) / "Cura Engine"
        self.source.mkdir()
        self.visual_studio = Path(self.temporary.name) / "Visual Studio"
        self.visual_studio.mkdir()
        self.local_app_data = Path(self.temporary.name) / "Local App Data"
        self.versions = {"cl": "19.44.35207", "conan": "2.24.0", "cmake": "3.23.0", "ninja": "1.10.0"}
        self.process = self.enterContext(patch("build_windows.subprocess.run", side_effect=self.runProcess))
        self.enterContext(patch("build_windows.sys.platform", "win32"))
        self.enterContext(patch.dict(os.environ, {
            "VSCMD_ARG_TGT_ARCH": "x64",
            "VSINSTALLDIR": str(self.visual_studio),
            "LOCALAPPDATA": str(self.local_app_data),
            "CONAN_HOME": "existing-cache",
        }))

    def runProcess(self, command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        return subprocess.CompletedProcess(command, 0, stdout="version {}".format(self.versions[command[0]]))

    def commands(self) -> list[list[str]]:
        return [call.args[0] for call in self.process.call_args_list]

    def testDefaultRemains2022(self) -> None:
        with patch("sys.argv", ["build_windows.py"]), patch("build_windows.buildWindows") as build:
            self.assertEqual(build_windows.main(), 0)
        self.assertEqual(build.call_args.args[1:], ("2022", "Release", False))

    def test2022RetainsOlderCMakeAndIsolatesOutputs(self) -> None:
        for compiler in ("19.39.33519", "19.44.35207"):
            with self.subTest(compiler=compiler):
                self.process.reset_mock()
                self.versions["cl"] = compiler
                original_environment = os.environ.copy()
                build_windows.buildWindows(self.source, "2022", "Release", False)
                self.assertEqual(dict(os.environ), original_environment)
                command = self.commands()[-1]
                output = self.source / "build" / "windows-vs2022"
                self.assertEqual(command[command.index("--output-folder") + 1], str(output))
                storage = command[command.index("-cc") + 1]
                self.assertTrue(storage.startswith("core.cache:storage_path={}".format(self.local_app_data)))
                self.assertTrue(storage.endswith("vs2022"))
                self.assertIn("tools.cmake.cmaketoolchain:user_presets=", command)
                self.assertIn("&:tools.build:skip_test=True", command)
                environment = self.process.call_args.kwargs["env"]
                self.assertEqual(environment["CONAN_HOME"], str(output / "conan-home"))
                self.assertEqual(environment["CC"], "cl")
                self.assertEqual(environment["CXX"], "cl")
                self.assertEqual(list(self.source.iterdir()), [])

    def test2026DebugWithTestsUsesSeparateBuild(self) -> None:
        self.versions.update(cl="19.50.35727", conan="2.24.0", cmake="4.2.0")
        build_windows.buildWindows(self.source, "2026", "Debug", True)
        command = self.commands()[-1]
        self.assertEqual(command[:3], ["conan", "build", str(self.source)])
        self.assertIn(str(self.source / "build" / "windows-vs2026"), command)
        self.assertIn("build_type=Debug", command)
        self.assertIn("&:tools.build:skip_test=False", command)
        self.assertNotIn("tools.build:skip_test=False", command)
        self.assertIn("cura.jinja", command)
        self.assertIn("cura_build.jinja", command)
        installation = "tools.microsoft.msbuild:installation_path={}".format(self.visual_studio)
        self.assertEqual(command.count(installation), 2)

    def testConanStorageIsPerSourceAndCompiler(self) -> None:
        environment = {"LOCALAPPDATA": str(self.local_app_data)}
        source_a = self.source / "a"
        source_b = self.source / "b"
        paths = {
            build_windows.getConanStorage(source_a, "2022", environment),
            build_windows.getConanStorage(source_a, "2026", environment),
            build_windows.getConanStorage(source_b, "2026", environment),
        }
        self.assertEqual(len(paths), 3)
        for path in paths:
            self.assertEqual(path.parents[2], self.local_app_data / "CuraEngine")

    def testMissingLocalAppDataStopsBeforeConan(self) -> None:
        with patch.dict(os.environ, {"LOCALAPPDATA": ""}):
            with self.assertRaisesRegex(RuntimeError, "LOCALAPPDATA"):
                build_windows.buildWindows(self.source, "2022", "Release", False)
        self.assertEqual(self.commands(), [["cl", "/?"]])

    def testCompilerMismatchStopsBeforeConan(self) -> None:
        for selected, compiler in (("2022", "19.50.35727"), ("2026", "19.44.35207")):
            with self.subTest(selected=selected):
                self.process.reset_mock()
                self.versions["cl"] = compiler
                with self.assertRaisesRegex(RuntimeError, "does not match"):
                    build_windows.buildWindows(self.source, selected, "Release", False)
                self.assertEqual(self.commands(), [["cl", "/?"]])

    def testUnsupportedToolsStopBeforeConfiguration(self) -> None:
        for conan, cmake in (("2.7.1", "4.2.0"), ("3.0.0", "4.2.0"), ("2.24.0", "3.31.0")):
            with self.subTest(conan=conan, cmake=cmake):
                self.process.reset_mock()
                self.versions.update(cl="19.50.35727", conan=conan, cmake=cmake)
                with self.assertRaisesRegex(RuntimeError, "requires"):
                    build_windows.buildWindows(self.source, "2026", "Release", False)
                self.assertFalse(any(command[1] in ("config", "profile", "build") for command in self.commands()))

    def testConfigurationFailureStopsBuild(self) -> None:
        self.process.side_effect = [subprocess.CompletedProcess([], 0, "version " + version)
                                    for version in self.versions.values()] + [subprocess.CalledProcessError(1, "conan")]
        with self.assertRaises(subprocess.CalledProcessError):
            build_windows.buildWindows(self.source, "2022", "Release", False)
        self.assertEqual(self.commands()[-1][1:3], ["config", "install"])

    def testOldConanCannotOverwrite2022Presets(self) -> None:
        self.versions["conan"] = "2.7.1"
        with self.assertRaisesRegex(RuntimeError, "Conan >= 2.24.0"):
            build_windows.buildWindows(self.source, "2022", "Release", False)
        self.assertEqual(len(self.commands()), 2)

    def testExistingHelperProfilesArePreserved(self) -> None:
        profiles = self.source / "build" / "windows-vs2022" / "conan-home" / "profiles"
        profiles.mkdir(parents=True)
        for name in ("cura.jinja", "cura_build.jinja"):
            (profiles / name).write_text("include(default)\n")
        build_windows.buildWindows(self.source, "2022", "Release", False)
        self.assertFalse(any(command[1:3] == ["config", "install"] for command in self.commands()))
        self.assertIn(["conan", "profile", "detect", "--force"], self.commands())

    def testMissingDeveloperPromptStopsBeforeCommands(self) -> None:
        with patch.dict(os.environ, {"VSCMD_ARG_TGT_ARCH": "x86"}):
            with self.assertRaisesRegex(RuntimeError, "x64 Native Tools"):
                build_windows.buildWindows(self.source, "2022", "Release", False)
        self.process.assert_not_called()

    def testMissingVisualStudioInstallationStopsBeforeConan(self) -> None:
        with patch.dict(os.environ, {"VSINSTALLDIR": ""}):
            with self.assertRaisesRegex(RuntimeError, "active Visual Studio installation"):
                build_windows.buildWindows(self.source, "2022", "Release", False)
        self.assertEqual(self.commands(), [["cl", "/?"]])

    def testFailureIsReportedAsNonzeroExit(self) -> None:
        with patch("sys.argv", ["build_windows.py"]), patch("build_windows.buildWindows", side_effect=OSError("missing cl")):
            self.assertEqual(build_windows.main(), 1)


if __name__ == "__main__":
    unittest.main()

#  Copyright (c) 2022 Ultimaker B.V.
#  CuraEngine is released under the terms of the AGPLv3 or higher

import os

from conan import ConanFile
from conan.errors import ConanInvalidConfiguration
from conan.tools import files
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake, cmake_layout
from conan.tools.build import check_min_cppstd
from conan.tools.scm import Version

required_conan_version = ">=1.48.0"


class CuraEngineConan(ConanFile):
    name = "curaengine"
    license = "AGPL-3.0"
    author = "Ultimaker B.V."
    url = "https://github.com/Ultimaker/CuraEngine"
    description = "Powerful, fast and robust engine for converting 3D models into g-code instructions for 3D printers. It is part of the larger open source project Cura."
    topics = ("cura", "protobuf", "gcode", "c++", "curaengine", "libarcus", "gcode-generation", "3D-printing")
    build_policy = "missing"
    exports = "LICENSE*"
    settings = "os", "compiler", "build_type", "arch"

    python_requires = "umbase/0.1.5@ultimaker/testing"
    python_requires_extend = "umbase.UMBaseConanfile"

    options = {
        "enable_arcus": [True, False],
        "enable_openmp": [True, False],
        "enable_testing": [True, False],
        "enable_extensive_warnings": [True, False]
    }
    default_options = {
        "enable_arcus": True,
        "enable_openmp": True,
        "enable_testing": False,
        "enable_extensive_warnings": True,
    }
    scm = {
        "type": "git",
        "subfolder": ".",
        "url": "auto",
        "revision": "auto"
    }

    def config_options(self):
        if self.settings.os == "Macos":
            del self.options.enable_openmp

    def configure(self):
        self.options["boost"].header_only = True
        self.options["clipper"].shared = True
        self.options["fmt"].shared = True
        self.options["spdlog"].shared = True
        if self.options.enable_arcus:
            self.options["arcus"].shared = True
            self.options["protobuf"].shared = True

    def validate(self):
        if self.settings.compiler.get_safe("cppstd"):
            check_min_cppstd(self, 20)
        if self.version:
            if Version(self.version) <= Version("4"):
                raise ConanInvalidConfiguration("only versions 5+ are supported")

    def build_requirements(self):
        if self.options.enable_arcus:
            for req in self._um_data()["build_requirements_arcus"]:
                self.tool_requires(req)
        if self.options.enable_testing:
            for req in self._um_data()["build_requirements_testing"]:
                self.test_requires(req)

    def requirements(self):
        for req in self._um_data()["requirements"]:
            self.requires(req)
        if self.options.enable_arcus:
            for req in self._um_data()["requirements_arcus"]:
                self.requires(req)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()

        tc = CMakeToolchain(self)
        tc.variables["CURA_ENGINE_VERSION"] = self.version
        tc.variables["ENABLE_ARCUS"] = self.options.enable_arcus
        tc.variables["ENABLE_TESTING"] = self.options.enable_testing
        tc.variables["EXTENSIVE_WARNINGS"] = self.options.enable_extensive_warnings
        if self.settings.os != "Macos":
            tc.variables["ENABLE_OPENMP"] = self.options.enable_openmp
        tc.generate()

    def layout(self):
        cmake_layout(self)

        self.cpp.build.includedirs = ["."]  # To package the generated headers
        self.cpp.package.libs = ["_CuraEngine"]

    def imports(self):
        self.copy("*.dll", dst=self.build_folder, src="@bindirs")
        self.copy("*.dylib", dst=self.build_folder, src="@bindirs")
        if self.options.enable_testing:
            dest = os.path.join(self.build_folder, "tests")
            self.copy("*.dll", dst=dest, src="@bindirs")
            self.copy("*.dylib", dst=dest, src="@bindirs")

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        packager = files.AutoPackager(self)
        packager.run()
        self.copy("CuraEngine", src = self.build_folder, dst = "bin")
        self.copy("CuraEngine.exe", src = self.build_folder, dst = "bin")

    def package_info(self):
        ext = ".exe" if self.settings.os == "Windows" else ""
        pack_folder = "" if self.package_folder is None else self.package_folder
        self.user_info.curaengine = os.path.join(pack_folder, "bin", f"CuraEngine{ext}")
        self.conf_info.define("user.curaengine:curaengine", os.path.join(pack_folder, "bin", f"CuraEngine{ext}"))

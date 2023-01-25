#  Copyright (c) 2022 Ultimaker B.V.
#  CuraEngine is released under the terms of the AGPLv3 or higher

from os import path

from conan import ConanFile
from conan.errors import ConanInvalidConfiguration
from conan.tools.files import AutoPackager, copy, mkdir
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake, cmake_layout
from conan.tools.build import check_min_cppstd
from conan.tools.scm import Version

required_conan_version = ">=1.50.0"


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

    python_requires = "umbase/[>=0.1.7]@ultimaker/stable"
    python_requires_extend = "umbase.UMBaseConanfile"

    options = {
        "enable_arcus": [True, False],
        "enable_openmp": [True, False],
        "enable_testing": [True, False],
        "enable_benchmarks": [True, False],
        "enable_extensive_warnings": [True, False]
    }
    default_options = {
        "enable_arcus": True,
        "enable_openmp": True,
        "enable_testing": False,
        "enable_benchmarks": False,
        "enable_extensive_warnings": False,
    }
    scm = {
        "type": "git",
        "subfolder": ".",
        "url": "auto",
        "revision": "auto"
    }

    def set_version(self):
        if self.version is None:
            self.version = self._umdefault_version()

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
        if self.options.enable_benchmarks:
            for req in self._um_data()["build_requirements_benchmarks"]:
                self.test_requires(req)

    def requirements(self):
        self.requires("standardprojectsettings/[>=0.1.0]@ultimaker/stable")
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
        tc.variables["ENABLE_BENCHMARKS"] = self.options.enable_benchmarks
        tc.variables["EXTENSIVE_WARNINGS"] = self.options.enable_extensive_warnings
        if self.settings.os != "Macos":
            tc.variables["ENABLE_OPENMP"] = self.options.enable_openmp
        tc.generate()

        for dep in self.dependencies.values():
            if len(dep.cpp_info.libdirs) > 0:
                copy(self, "*.dylib", dep.cpp_info.libdirs[0], self.build_folder)
                copy(self, "*.dll", dep.cpp_info.libdirs[0], self.build_folder)
            if len(dep.cpp_info.bindirs) > 0:
                copy(self, "*.dll", dep.cpp_info.bindirs[0], self.build_folder)
            if self.options.enable_testing:
                test_path = path.join(self.build_folder,  "tests")
                if not path.exists(test_path):
                    mkdir(self, test_path)
                if len(dep.cpp_info.libdirs) > 0:
                    copy(self, "*.dylib", dep.cpp_info.libdirs[0], path.join(self.build_folder,  "tests"))
                    copy(self, "*.dll", dep.cpp_info.libdirs[0], path.join(self.build_folder,  "tests"))
                if len(dep.cpp_info.bindirs) > 0:
                    copy(self, "*.dll", dep.cpp_info.bindirs[0], path.join(self.build_folder,  "tests"))

    def layout(self):
        cmake_layout(self)

        self.cpp.build.includedirs = ["."]  # To package the generated headers
        self.cpp.package.libs = ["_CuraEngine"]

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        packager = AutoPackager(self)
        packager.run()
        copy(self, "CuraEngine*", src = self.build_folder, dst = path.join(self.package_folder, "bin"))
        copy(self, "LICENSE*", src = self.source_folder, dst = path.join(self.package_folder, "license"))

    def package_info(self):
        ext = ".exe" if self.settings.os == "Windows" else ""
        if self.in_local_cache:
            self.conf_info.define("user.curaengine:curaengine", path.join(self.package_folder, "bin", f"CuraEngine{ext}"))
        else:
            self.conf_info.define("user.curaengine:curaengine", path.join(self.build_folder, f"CuraEngine{ext}"))

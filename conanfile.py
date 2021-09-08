import os
import pathlib

from conans import ConanFile, tools
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake
from conan.tools.layout import LayoutPackager, clion_layout


class CuraEngineConan(ConanFile):
    name = "CuraEngine"
    version = "4.11.0"
    license = "AGPL-3.0"
    author = "Ultimaker B.V."
    url = "https://github.com/Ultimaker/CuraEngine"
    description = "Powerful, fast and robust engine for converting 3D models into g-code instructions for 3D printers. It is part of the larger open source project Cura."
    topics = ("conan", "cura", "protobuf", "gcode", "c++", "curaengine", "libarcus", "gcode-generation")
    settings = "os", "compiler", "build_type", "arch"
    revision_mode = "scm"
    build_policy = "missing"
    exports = "LICENSE"
    options = {
        "enable_arcus": [True, False],
        "enable_openmp": [True, False],
        "tests": [True, False]
    }
    default_options = {
        "enable_arcus": True,
        "enable_openmp": True,
        "tests": False
    }
    scm = {
        "type": "git",
        "subfolder": ".",
        "url": "auto",
        "revision": "auto"
    }

    def configure(self):
        self.options["protobuf"].shared = False if self.settings.os == "Macos" else True
        if self.options.enable_arcus:
            self.options["Arcus"].shared = True
        self.options["clipper"].shared = True
        if self.settings.os == "Macos":
            self.options.enable_openmp = False

    def build_requirements(self):
        self.build_requires("cmake/[>=3.16.2]")
        if self.options.tests:
            self.build_requires("gtest/[>=1.10.0]", force_host_context = True)

    def requirements(self):
        self.requires("stb/20200203")
        if self.options.enable_arcus:
            self.requires(f"Arcus/4.11.0@ultimaker/testing")
            self.requires("protobuf/3.17.1")
        self.requires("clipper/[>=6.4.2]")
        self.requires("rapidjson/[>=1.1.0]")

    def validate(self):
        if self.settings.compiler.get_safe("cppstd"):
            tools.check_min_cppstd(self, 17)

    def layout(self):
        clion_layout(self)
        self.cpp.build.bindirs = ["."]
        self.patterns.build.bin = ["*.dll", "*.a", "*.so", "*.exe", "CuraEngine*"]
        self.cpp.package.bindirs = ["bin"]

    def generate(self):
        cmake = CMakeDeps(self)
        cmake.generate()

        tc = CMakeToolchain(self)

        # FIXME: This shouldn't be necessary (maybe a bug in Conan????)
        if self.settings.compiler == "Visual Studio":
            tc.blocks["generic_system"].values["generator_platform"] = None
            tc.blocks["generic_system"].values["toolset"] = None

        tc.variables["USE_SYSTEM_LIBS"] = True
        tc.variables["ALLOW_IN_SOURCE_BUILD"] = True
        tc.variables["ENABLE_ARCUS"] = self.options.enable_arcus
        tc.variables["BUILD_TESTS"] = self.options.tests
        tc.variables["ENABLE_OPENMP"] = self.options.enable_openmp
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        LayoutPackager(self).package()

    def package_info(self):
        ext = ".exe" if self.settings.os == "Windows" else ""
        if self.in_local_cache:
            bin_path = os.path.join(self.package_folder, "bin")
            self.user_info.CURAENGINE = str(os.path.join(bin_path, f"CuraEngine{ext}"))
            self.env_info.path.append(bin_path)
        else:
            bin_path = os.path.join(pathlib.Path(__file__).parent.absolute(), f"cmake-build-{self.settings.build_type}".lower())
            self.user_info.CURAENGINE = str(os.path.join(bin_path, f"CuraEngine{ext}"))
            self.env_info.path.append(bin_path)

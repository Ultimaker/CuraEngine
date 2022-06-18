import os

from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake, cmake_layout
from conan.tools import files
from conans import tools
from conan.errors import ConanInvalidConfiguration
from conans.errors import ConanException

required_conan_version = ">=1.47.0"


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

    python_requires = "umbase/0.1@ultimaker/testing"
    python_requires_extend = "umbase.UMBaseConanfile"

    options = {
        "enable_arcus": [True, False],
        "enable_openmp": [True, False],
        "enable_testing": [True, False]
    }
    default_options = {
        "enable_arcus": True,
        "enable_openmp": True,
        "enable_testing": False
    }
    scm = {
        "type": "git",
        "subfolder": ".",
        "url": "auto",
        "revision": "auto"
    }

    def config_options(self):
        if self.settings.os == "Macos":
            self.options.enable_openmp = False

    def configure(self):
        self.options["boost"].header_only = True
        self.options["*"].shared = True

    def validate(self):
        if self.settings.compiler.get_safe("cppstd"):
            tools.check_min_cppstd(self, 17)
        if self.version:
            if tools.Version(self.version) <= tools.Version("4"):
                raise ConanInvalidConfiguration("Only versions 5+ are support")

    def build_requirements(self):
        if self.options.enable_arcus:
            for req in self._um_data(self.version)["build_requirements_arcus"]:
                self.tool_requires(req)
        if self.options.enable_testing:
            for req in self._um_data(self.version)["build_requirements_testing"]:
                self.test_requires(req)

    def requirements(self):
        for req in self._um_data(self.version)["requirements"]:
            self.requires(req)
        if self.options.enable_arcus:
            for req in self._um_data(self.version)["requirements_arcus"]:
                self.requires(req)

    def generate(self):
        cmake = CMakeDeps(self)
        if self.options.enable_arcus:
            cmake.build_context_activated = ["protobuf"]
            cmake.build_context_suffix = {"protobuf": "_BUILD"}
            cmake.build_context_build_modules = ["protobuf"]

        if self.options.enable_testing:
            cmake.build_context_activated = ["gtest"]
        cmake.generate()

        tc = CMakeToolchain(self, generator = "Ninja")

        tc.variables["ENABLE_ARCUS"] = self.options.enable_arcus
        tc.variables["BUILD_TESTING"] = self.options.enable_testing
        tc.variables["ENABLE_OPENMP"] = self.options.enable_openmp
        tc.variables["ALLOW_IN_SOURCE_BUILD"] = True

        # Don't use Visual Studio as the CMAKE_GENERATOR
        if self.settings.compiler == "Visual Studio":
            tc.blocks["generic_system"].values["generator_platform"] = None
            tc.blocks["generic_system"].values["toolset"] = None

        tc.generate()

    def layout(self):
        # TODO: Use the cmake_layout provided by Conan, that requires restructuring the headers and sources,
        # or if we decided against a cmake_layout, we should at least make things uniform across our projects
        # and create the layout by defining a function in the UMBaseConanfile.
        # https://docs.conan.io/en/latest/reference/conanfile/tools/layout.html
        self.folders.source = "."
        try:
            build_type = str(self.settings.build_type)
        except ConanException:
            raise ConanException("'build_type' setting not defined, it is necessary")

        self.folders.build = f"cmake-build-{build_type.lower()}"
        self.folders.generators = os.path.join(self.folders.build, "conan")

        self.cpp.source.includedirs = ["src"]  # TODO: Seperate headers and cpp

        self.cpp.build.libdirs = ["."]
        self.cpp.build.bindirs = ["."]

        self.cpp.build.libs = ["_CuraEngine"]

        self.cpp.package.includedirs = ["include"]
        self.cpp.package.libdirs = ["lib"]
        self.cpp.package.bindirs = ['bin']

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
        if self.options.enable_testing:
            cmake.test()

    def package(self):
        packager = files.AutoPackager(self)
        packager.run()

        files.rmdir(self, os.path.join(self.package_folder, "bin", "CMakeFiles"))

from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake, cmake_layout
from conan.tools import files
from conans import tools

required_conan_version = ">=1.46.2"


class CuraEngineConan(ConanFile):
    name = "curaengine"
    license = "AGPL-3.0"
    author = "Ultimaker B.V."
    url = "https://github.com/Ultimaker/CuraEngine"
    description = "Powerful, fast and robust engine for converting 3D models into g-code instructions for 3D printers. It is part of the larger open source project Cura."
    topics = ("conan", "cura", "protobuf", "gcode", "c++", "curaengine", "libarcus", "gcode-generation")
    build_policy = "missing"
    exports = "LICENSE*"
    settings = "os", "compiler", "build_type", "arch"
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
        if self.settings.os == "Windows":
            self.options.enable_openmp = False

    def configure(self):
        self.options["boost"].header_only = True
        self.options["clipper"].shared = True
        self.options["protobuf"].shared = True
        if self.options.enable_arcus:
            self.options["arcus"].shared = not self.settings.os == "Windows"

    def validate(self):
        if self.settings.compiler.get_safe("cppstd"):
            tools.check_min_cppstd(self, 17)

    def build_requirements(self):
        self.tool_requires("ninja/[>=1.10.0]")
        self.tool_requires("cmake/[>=3.23.0]")
        if self.options.enable_arcus:
            self.tool_requires("protobuf/3.17.1")
        if self.options.enable_testing:
            self.test_requires("gtest/[>=1.10.0]")

    def requirements(self):
        self.requires("protobuf/3.17.1")
        self.requires("clipper/6.4.2")
        self.requires("boost/1.78.0")
        self.requires("rapidjson/1.1.0")
        self.requires("stb/20200203")
        if self.options.enable_arcus:
            self.requires("protobuf/3.17.1")
            self.requires("arcus/latest@ultimaker/stable")
        if self.options.enable_openmp:
            self.requires("llvm-openmp/12.0.1")

    def generate(self):
        cmake = CMakeDeps(self)
        if self.options.enable_arcus:
            if len(cmake.build_context_activated) == 0:
                cmake.build_context_activated = ["protobuf"]
            else:
                cmake.build_context_activated.append("protobuf")
            cmake.build_context_suffix = {"protobuf": "_BUILD"}
            if len(cmake.build_context_activated) == 0:
                cmake.build_context_build_modules = ["protobuf"]
            else:
                cmake.build_context_build_modules.append("protobuf")

        if self.options.enable_testing:
            if len(cmake.build_context_build_modules) == 0:
                cmake.build_context_activated = ["gtest"]
            else:
                cmake.build_context_activated.append("gtest")
        cmake.generate()

        tc = CMakeToolchain(self, generator = "Ninja")

        tc.variables["ALLOW_IN_SOURCE_BUILD"] = True
        tc.variables["ENABLE_ARCUS"] = self.options.enable_arcus
        tc.variables["BUILD_TESTS"] = self.options.enable_testing
        tc.variables["ENABLE_OPENMP"] = self.options.enable_openmp

        # Don't use Visual Studio as the CMAKE_GENERATOR
        if self.settings.compiler == "Visual Studio":
            tc.blocks["generic_system"].values["generator_platform"] = None
            tc.blocks["generic_system"].values["toolset"] = None

        tc.generate()

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        packager = files.AutoPackager(self)
        packager.run()
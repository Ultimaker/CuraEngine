from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps


required_conan_version = ">=1.46.2"


class CuraBuildEnvironemtConan(ConanFile):
    name = "cura-build-environment"
    description = "Building Cura dependencies"
    topics = ("conan", "python", "pypi", "pip")
    settings = "os", "compiler", "build_type", "arch"
    build_policy = "missing"

    def configure(self):
        self.options["boost"].header_only = True

    def requirements(self):
        self.requires("protobuf/3.17.1")
        self.requires("clipper/6.4.2")
        self.requires("boost/1.78.0")
        self.requires("gtest/1.8.1")
        self.requires("rapidjson/1.1.0")
        self.requires("stb/20200203")
        self.requires("arcus/latest@ultimaker/stable")

    def generate(self):
        cmake = CMakeDeps(self)
        cmake.generate()

        tc = CMakeToolchain(self)

        # Don't use Visual Studio as the CMAKE_GENERATOR
        if self.settings.compiler == "Visual Studio":
            tc.blocks["generic_system"].values["generator_platform"] = None
            tc.blocks["generic_system"].values["toolset"] = None

        tc.generate()

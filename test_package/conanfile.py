from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake
from conan.tools.env import VirtualRunEnv
from conans import tools


class CuraEngineTestConan(ConanFile):
    build_policy = "missing"
    settings = "os", "compiler", "build_type", "arch"

    python_requires = "umbase/0.1.5@ultimaker/testing"
    python_requires_extend = "umbase.UMBaseConanfile"

    def requirements(self):
        for req in self._um_data()["requirements"]:
            self.requires(req)

    def generate(self):
        cmake = CMakeDeps(self)
        cmake.generate()

        venv = VirtualRunEnv(self)
        venv.generate()

        tc = CMakeToolchain(self, generator = "Ninja")
        if self.settings.compiler == "Visual Studio":
            tc.blocks["generic_system"].values["generator_platform"] = None
            tc.blocks["generic_system"].values["toolset"] = None
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def imports(self):
        self.copy("*.lib", dst=".", src="@bindirs")
        #self.copy("*.pb.h", dst=".", src="@bindirs")  # Needs to be in if test_package is extended to arcus use, but that would also defy the minimalist 'only test if it can build and run' that 'test_package' is supposed to have.
        if self.settings.os == "Windows" and not tools.cross_building(self, skip_x64_x86 = True):
            self.copy("*.dll", dst=".", src="@bindirs")

    def test(self):
        if not tools.cross_building(self):
            (prefix, postfix) = ("", ".exe") if self.settings.os == "Windows" else ("./", "")
            self.run(f"{prefix}PackageTest{postfix}", env = "conanrun")

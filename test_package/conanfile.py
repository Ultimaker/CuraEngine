from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake
from conan.tools.env import VirtualRunEnv
from conans import tools


class CuraEngineTestConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"

    python_requires = "umbase/0.1.5@ultimaker/testing"
    python_requires_extend = "umbase.UMBaseConanfile"

    def requirements(self):
        # NOTE: Also put what would otherwise be 'build_requirements' here, since this is a test package.
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
        self.copy("*.pb.h", dst=".", src="@bindirs")
        if self.settings.os == "Windows" and not tools.cross_building(self, skip_x64_x86 = True):
            self.copy("*.dll", dst=".", src="@bindirs")

    def test(self):
        if not tools.cross_building(self):
            import os
            for program in [f for f in os.listdir(".") if f.replace(".exe", "").endswith("Test")]:
                prefix_path = "" if self.settings.os == "Windows" else "./"
                self.run(f"{prefix_path}{program}", env = "conanrun")

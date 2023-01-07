#  Copyright (c) 2022 Ultimaker B.V.
#  CuraEngine is released under the terms of the AGPLv3 or higher

from conan import ConanFile
from conan.tools.env import VirtualRunEnv
from conans import tools
from conans.errors import ConanException
from io import StringIO


class CuraEngineTestConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "VirtualRunEnv"

    def generate(self):
        venv = VirtualRunEnv(self)
        venv.generate()

    def build(self):
        pass

    def imports(self):
        self.copy("*.lib", dst = ".", src = "@bindirs")
        if self.settings.os == "Windows" and not tools.cross_building(self):
            self.copy("*.dll", dst = self.build_folder, src = "@bindirs")
            self.copy("*.dylib", dst = self.build_folder, src = "@bindirs")

    def test(self):
        if not tools.cross_building(self):
            test_buf = StringIO()
            self.run(f"{self.deps_user_info['curaengine'].curaengine}", env = "conanrun", output=test_buf)
            out_val = test_buf.getvalue()
            if f"Cura_SteamEngine version" not in out_val:
                raise ConanException("CuraEngine wasn't build correctly!")

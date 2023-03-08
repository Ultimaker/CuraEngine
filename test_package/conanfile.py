#  Copyright (c) 2023 Ultimaker B.V.
#  CuraEngine is released under the terms of the AGPLv3 or higher

from conan import ConanFile
from conan.tools.env import VirtualRunEnv
from conan.tools.build import can_run


class CuraEngineTestConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "VirtualRunEnv"
    test_type = "explicit"

    def requirements(self):
        self.requires(self.tested_reference_str)

    def generate(self):
        venv = VirtualRunEnv(self)
        venv.generate()

    def build(self):
        pass

    def test(self):
        if can_run(self):
            self.run("CuraEngine help", env = "conanrun")

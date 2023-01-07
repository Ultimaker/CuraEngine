#  Copyright (c) 2023 Ultimaker B.V.
#  CuraEngine is released under the terms of the AGPLv3 or higher

from os import path

from conan import ConanFile
from conan.errors import ConanInvalidConfiguration
from conan.tools.files import copy, mkdir, update_conandata
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake, cmake_layout
from conan.tools.build import check_min_cppstd
from conan.tools.scm import Version, Git

required_conan_version = ">=1.55.0"


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

    def export(self):
        print("exporting")
        git = Git(self, self.recipe_folder)
        scm_url, scm_commit = git.get_url_and_commit()
        update_conandata(self, {"sources": {"commit": scm_commit, "url": scm_url}})

    def source(self):
        print("source")
        git = Git(self)
        sources = self.conan_data["sources"]
        git.clone(url=sources["url"], target=".")
        git.checkout(commit=sources["commit"])

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
            self.test_requires("protobuf/3.21.4")
            self.test_requires("arcus/5.2.2")
            self.test_requires("zlib/1.2.12")
        if self.options.enable_testing:
            self.test_requires("gtest/1.12.1")
        if self.options.enable_benchmarks:
            self.test_requires("benchmark/1.7.0")

    def requirements(self):
        self.requires("standardprojectsettings/[>=0.1.0]@ultimaker/stable")
        self.requires("clipper/6.4.2")
        self.requires("boost/1.79.0")
        self.requires("rapidjson/1.1.0")
        self.requires("stb/20200203")
        self.requires("spdlog/1.10.0")
        self.requires("fmt/9.0.0")
        self.requires("range-v3/0.12.0")

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
        ext = ".exe" if self.settings.os == "Windows" else ""
        copy(self, f"CuraEngine{ext}", src = self.build_folder, dst = path.join(self.package_folder, "bin"))
        copy(self, f"_CuraEngine.*", src = self.build_folder, dst = path.join(self.package_folder, "lib"))
        copy(self, "LICENSE*", src = self.source_folder, dst = path.join(self.package_folder, "license"))

    def package_info(self):
        ext = ".exe" if self.settings.os == "Windows" else ""
        if self.in_local_cache:
            self.conf_info.define("user.curaengine:curaengine", path.join(self.package_folder, "bin", f"CuraEngine{ext}"))
        else:
            self.conf_info.define("user.curaengine:curaengine", path.join(self.build_folder, f"CuraEngine{ext}"))

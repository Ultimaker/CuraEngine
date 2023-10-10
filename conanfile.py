#  Copyright (c) 2023 UltiMaker
#  CuraEngine is released under the terms of the AGPLv3 or higher

from os import path

from conan import ConanFile
from conan.errors import ConanInvalidConfiguration
from conan.tools.files import copy, mkdir
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake, cmake_layout
from conan.tools.build import check_min_cppstd
from conan.tools.scm import Version

required_conan_version = ">=1.58.0 <2.0.0"


class CuraEngineConan(ConanFile):
    name = "curaengine"
    license = "AGPL-3.0"
    author = "UltiMaker"
    url = "https://github.com/Ultimaker/CuraEngine"
    description = "Powerful, fast and robust engine for converting 3D models into g-code instructions for 3D printers. It is part of the larger open source project Cura."
    topics = ("cura", "protobuf", "gcode", "c++", "curaengine", "libarcus", "gcode-generation", "3D-printing")
    exports = "LICENSE*"
    settings = "os", "compiler", "build_type", "arch"

    options = {
        "enable_arcus": [True, False],
        "enable_benchmarks": [True, False],
        "enable_extensive_warnings": [True, False],
        "enable_plugins": [True, False],
        "enable_remote_plugins": [True, False],
    }
    default_options = {
        "enable_arcus": True,
        "enable_benchmarks": False,
        "enable_extensive_warnings": False,
        "enable_plugins": True,
        "enable_remote_plugins": False,
    }

    def set_version(self):
        if not self.version:
            self.version = "5.6.0-alpha"

    def export_sources(self):
        copy(self, "CMakeLists.txt", self.recipe_folder, self.export_sources_folder)
        copy(self, "Cura.proto", self.recipe_folder, self.export_sources_folder)
        copy(self, "CuraEngine.ico", self.recipe_folder, self.export_sources_folder)
        copy(self, "CuraEngine.rc", self.recipe_folder, self.export_sources_folder)
        copy(self, "LICENSE", self.recipe_folder, self.export_sources_folder)
        copy(self, "*", path.join(self.recipe_folder, "src"), path.join(self.export_sources_folder, "src"))
        copy(self, "*", path.join(self.recipe_folder, "include"), path.join(self.export_sources_folder, "include"))
        copy(self, "*", path.join(self.recipe_folder, "benchmark"), path.join(self.export_sources_folder, "benchmark"))
        copy(self, "*", path.join(self.recipe_folder, "tests"), path.join(self.export_sources_folder, "tests"))

    def config_options(self):
        if not self.options.enable_plugins:
            del self.options.enable_remote_plugins

    def configure(self):
        self.options["boost"].header_only = True
        self.options["clipper"].shared = True

        self.options["protobuf"].shared = False
        if self.options.enable_arcus:
            self.options["arcus"].shared = True

    def validate(self):
        if self.settings.compiler.get_safe("cppstd"):
            check_min_cppstd(self, 20)
        if self.version:
            if Version(self.version) <= Version("4"):
                raise ConanInvalidConfiguration("only versions 5+ are supported")

    def build_requirements(self):
        self.test_requires("standardprojectsettings/[>=0.1.0]@ultimaker/stable")
        self.test_requires("protobuf/3.21.9")
        if not self.conf.get("tools.build:skip_test", False, check_type=bool):
            self.test_requires("gtest/1.12.1")
        if self.options.enable_benchmarks:
            self.test_requires("benchmark/1.7.0")

    def requirements(self):
        if self.options.enable_arcus:
            self.requires("arcus/5.3.0")
        self.requires("asio-grpc/2.6.0")
        self.requires("grpc/1.50.1")
        self.requires("curaengine_grpc_definitions/(latest)@ultimaker/testing")
        self.requires("clipper/6.4.2")
        self.requires("boost/1.82.0")
        self.requires("rapidjson/1.1.0")
        self.requires("stb/20200203")
        self.requires("spdlog/1.10.0")
        self.requires("fmt/9.0.0")
        self.requires("range-v3/0.12.0")
        self.requires("scripta/0.1.0@ultimaker/testing")
        self.requires("neargye-semver/0.3.0")
        self.requires("protobuf/3.21.9")
        self.requires("zlib/1.2.12")
        self.requires("openssl/1.1.1l")

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()

        tc = CMakeToolchain(self)
        tc.variables["CURA_ENGINE_VERSION"] = self.version
        tc.variables["ENABLE_ARCUS"] = self.options.enable_arcus
        tc.variables["ENABLE_TESTING"] = not self.conf.get("tools.build:skip_test", False, check_type=bool)
        tc.variables["ENABLE_BENCHMARKS"] = self.options.enable_benchmarks
        tc.variables["EXTENSIVE_WARNINGS"] = self.options.enable_extensive_warnings
        tc.variables["OLDER_APPLE_CLANG"] = self.settings.compiler == "apple-clang" and Version(self.settings.compiler.version) < "14"
        if self.options.enable_plugins:
            tc.variables["ENABLE_PLUGINS"] = True
            tc.variables["ENABLE_REMOTE_PLUGINS"] = self.options.enable_remote_plugins
        else:
            tc.variables["ENABLE_PLUGINS"] = self.options.enable_plugins
        tc.generate()

        for dep in self.dependencies.values():
            if len(dep.cpp_info.libdirs) > 0:
                copy(self, "*.dylib", dep.cpp_info.libdirs[0], self.build_folder)
                copy(self, "*.dll", dep.cpp_info.libdirs[0], self.build_folder)
            if len(dep.cpp_info.bindirs) > 0:
                copy(self, "*.dll", dep.cpp_info.bindirs[0], self.build_folder)
            if not self.conf.get("tools.build:skip_test", False, check_type=bool):
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
            self.conf_info.define_path("user.curaengine:curaengine", path.join(self.package_folder, "bin", f"CuraEngine{ext}"))
        else:
            self.conf_info.define_path("user.curaengine:curaengine", path.join(self.build_folder, f"CuraEngine{ext}"))

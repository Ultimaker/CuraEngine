#  Copyright (c) 2024 UltiMaker
#  CuraEngine is released under the terms of the AGPLv3 or higher
import os

from conan import ConanFile
from conan.errors import ConanInvalidConfiguration
from conan.tools.build import check_min_cppstd
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake, cmake_layout
from conan.tools.files import copy, mkdir, update_conandata
from conan.tools.microsoft import check_min_vs, is_msvc
from conan.tools.scm import Version, Git

required_conan_version = ">=2.7.0"


class CuraEngineConan(ConanFile):
    name = "curaengine"
    license = "AGPL-3.0"
    author = "UltiMaker"
    url = "https://github.com/Ultimaker/CuraEngine"
    description = "Powerful, fast and robust engine for converting 3D models into g-code instructions for 3D printers. It is part of the larger open source project Cura."
    topics = ("cura", "protobuf", "gcode", "c++", "curaengine", "libarcus", "gcode-generation", "3D-printing")
    exports = "LICENSE*"
    settings = "os", "compiler", "build_type", "arch"
    package_type = "application"
    python_requires = "sentrylibrary/1.0.0", "npmpackage/[>=1.0.0]"
    python_requires_extend = "sentrylibrary.SentryLibrary"

    options = {
        "enable_arcus": [True, False],
        "enable_benchmarks": [True, False],
        "enable_extensive_warnings": [True, False],
        "enable_plugins": [True, False],
        "enable_remote_plugins": [True, False],
        "with_cura_resources": [True, False],
    }
    default_options = {
        "enable_arcus": True,
        "enable_benchmarks": False,
        "enable_extensive_warnings": False,
        "enable_plugins": True,
        "enable_remote_plugins": False,
        "with_cura_resources": False,
    }

    @property
    def _compilers_minimum_version(self):
        return {
            "gcc": "12",
            "clang": "14",
            "apple-clang": "13",
            "msvc": "191",
            "visual_studio": "17",
        }

    def init(self):
        base = self.python_requires["sentrylibrary"].module.SentryLibrary
        self.options.update(base.options, base.default_options)

    def set_version(self):
        if not self.version:
            self.version = self.conan_data["version"]

    def export(self):
        git = Git(self)
        update_conandata(self, {"version": self.version, "commit": git.get_commit()})

    def export_sources(self):
        copy(self, "CMakeLists.txt", self.recipe_folder, self.export_sources_folder)
        copy(self, "Cura.proto", self.recipe_folder, self.export_sources_folder)
        copy(self, "CuraEngine.ico", self.recipe_folder, self.export_sources_folder)
        copy(self, "CuraEngine.rc", self.recipe_folder, self.export_sources_folder)
        copy(self, "LICENSE", self.recipe_folder, self.export_sources_folder)
        copy(self, "*", os.path.join(self.recipe_folder, "src"), os.path.join(self.export_sources_folder, "src"))
        copy(self, "*", os.path.join(self.recipe_folder, "include"),
             os.path.join(self.export_sources_folder, "include"))
        copy(self, "*", os.path.join(self.recipe_folder, "benchmark"),
             os.path.join(self.export_sources_folder, "benchmark"))
        copy(self, "*", os.path.join(self.recipe_folder, "stress_benchmark"),
             os.path.join(self.export_sources_folder, "stress_benchmark"))
        copy(self, "*", os.path.join(self.recipe_folder, "tests"), os.path.join(self.export_sources_folder, "tests"))

    def config_options(self):
        super().config_options()

        if not self.options.enable_plugins:
            del self.options.enable_remote_plugins

    def configure(self):
        super().configure()

        if self.options.enable_arcus or self.options.enable_plugins:
            self.options["protobuf"].shared = False
        if self.options.enable_arcus:
            self.options["arcus"].shared = True

    def validate(self):
        super().validate()

        if self.settings.compiler.get_safe("cppstd"):
            check_min_cppstd(self, 20)
        check_min_vs(self, 191)
        if not is_msvc(self):
            minimum_version = self._compilers_minimum_version.get(str(self.settings.compiler), False)
            if minimum_version and Version(self.settings.compiler.version) < minimum_version:
                raise ConanInvalidConfiguration(
                    f"{self.ref} requires C++{self._min_cppstd}, which your compiler does not support.")

    def build_requirements(self):
        self.test_requires("standardprojectsettings/[>=0.2.0]")
        if not self.conf.get("tools.build:skip_test", False, check_type=bool):
            self.test_requires("gtest/1.14.0")
        if self.options.enable_benchmarks:
            self.test_requires("benchmark/1.8.3")
            self.test_requires("docopt.cpp/0.6.3")

    def requirements(self):
        super().requirements()

        for req in self.conan_data["requirements"]:
            self.requires(req)
        if self.options.enable_arcus:
            for req in self.conan_data["requirements_arcus"]:
                self.requires(req)
        if self.options.enable_plugins:
            self.requires("neargye-semver/0.3.0")
            for req in self.conan_data["requirements_plugins"]:
                self.requires(req)
        if self.options.with_cura_resources:
            for req in self.conan_data["requirements_cura_resources"]:
                self.requires(req)
        self.requires("clipper/6.4.2@ultimaker/stable")
        self.requires("boost/1.86.0")
        self.requires("rapidjson/cci.20230929")
        self.requires("stb/cci.20230920")
        self.requires("spdlog/1.15.1")
        self.requires("fmt/11.1.3")
        self.requires("range-v3/0.12.0")
        self.requires("zlib/1.3.1")
        self.requires("mapbox-wagyu/0.5.0@ultimaker/stable")

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()

        tc = CMakeToolchain(self)
        tc.variables["CURA_ENGINE_VERSION"] = self.version
        tc.variables["CURA_ENGINE_HASH"] = self.conan_data["commit"]
        tc.variables["ENABLE_ARCUS"] = self.options.enable_arcus
        tc.variables["ENABLE_TESTING"] = not self.conf.get("tools.build:skip_test", False, check_type=bool)
        tc.variables["ENABLE_BENCHMARKS"] = self.options.enable_benchmarks
        tc.variables["EXTENSIVE_WARNINGS"] = self.options.enable_extensive_warnings
        tc.variables["OLDER_APPLE_CLANG"] = self.settings.compiler == "apple-clang" and Version(
            self.settings.compiler.version) < "14"
        tc.variables["ENABLE_THREADING"] = not (self.settings.arch == "wasm" and self.settings.os == "Emscripten")
        if self.options.enable_plugins:
            tc.variables["ENABLE_PLUGINS"] = True
            tc.variables["ENABLE_REMOTE_PLUGINS"] = self.options.enable_remote_plugins
        else:
            tc.variables["ENABLE_PLUGINS"] = self.options.enable_plugins
        self.setup_cmake_toolchain_sentry(tc)
        tc.generate()

        for dep in self.dependencies.values():
            if len(dep.cpp_info.libdirs) > 0:
                copy(self, "*.dylib", dep.cpp_info.libdirs[0], self.build_folder)
                copy(self, "*.dll", dep.cpp_info.libdirs[0], self.build_folder)
            if len(dep.cpp_info.bindirs) > 0:
                copy(self, "*.dll", dep.cpp_info.bindirs[0], self.build_folder)

            folder_dists = []
            if not self.conf.get("tools.build:skip_test", False, check_type=bool):
                folder_dists.append("tests")
            if self.options.enable_benchmarks:
                folder_dists.append("benchmark")
                folder_dists.append("stress_benchmark")

            for dist_folder in folder_dists:
                dist_path = os.path.join(self.build_folder, dist_folder)
                if not os.path.exists(dist_path):
                    mkdir(self, dist_path)
                if len(dep.cpp_info.libdirs) > 0:
                    copy(self, "*.dylib", dep.cpp_info.libdirs[0], os.path.join(self.build_folder, dist_folder))
                    copy(self, "*.dll", dep.cpp_info.libdirs[0], os.path.join(self.build_folder, dist_folder))
                if len(dep.cpp_info.bindirs) > 0:
                    copy(self, "*.dll", dep.cpp_info.bindirs[0], os.path.join(self.build_folder, dist_folder))

    def layout(self):
        cmake_layout(self)
        self.cpp.build.includedirs = ["."]  # To package the generated headers
        self.cpp.package.libs = ["_CuraEngine"]

        if self.settings.os == "Emscripten":
            self.cpp.build.bin = ["CuraEngine.js"]
            self.cpp.package.bin = ["CuraEngine.js"]
            self.cpp.build.bindirs += ["CuraEngine"]

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

        self.send_sentry_debug_files(binary_basename="CuraEngine")

    def deploy(self):
        copy(self, "CuraEngine*", src=os.path.join(self.package_folder, "bin"), dst=self.deploy_folder)

    def package(self):
        match self.settings.os:
            case "Windows":
                ext = ".exe"
            case "Emscripten":
                ext = ".js"
            case other:
                ext = ""
        copy(self, f"CuraEngine{ext}", src=self.build_folder, dst=os.path.join(self.package_folder, "bin"))
        copy(self, f"*.d.ts", src=self.build_folder, dst=os.path.join(self.package_folder, "bin"))
        copy(self, f"_CuraEngine.*", src=self.build_folder, dst=os.path.join(self.package_folder, "lib"))
        copy(self, "LICENSE*", src=self.source_folder, dst=os.path.join(self.package_folder, "license"))

    def package_info(self):
        ext = ".exe" if self.settings.os == "Windows" else ""
        self.conf_info.define_path("user.curaengine:curaengine",
                                   os.path.join(self.package_folder, "bin", f"CuraEngine{ext}"))

        if self.settings.os == "Emscripten":
            self.python_requires["npmpackage"].module.conf_package_json(self)

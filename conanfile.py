#  Copyright (c) 2024 UltiMaker
#  CuraEngine is released under the terms of the AGPLv3 or higher
import os
from shutil import which

from conan import ConanFile
from conan.errors import ConanInvalidConfiguration
from conan.tools.files import copy, mkdir, update_conandata
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake, cmake_layout
from conan.tools.build import check_min_cppstd
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

    options = {
        "enable_arcus": [True, False],
        "enable_benchmarks": [True, False],
        "enable_extensive_warnings": [True, False],
        "enable_plugins": [True, False],
        "enable_sentry": [True, False],
        "enable_remote_plugins": [True, False],
        "with_cura_resources": [True, False],
    }
    default_options = {
        "enable_arcus": True,
        "enable_benchmarks": False,
        "enable_extensive_warnings": False,
        "enable_plugins": True,
        "enable_sentry": False,
        "enable_remote_plugins": False,
        "with_cura_resources": False,
    }

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
        copy(self, "*", os.path.join(self.recipe_folder, "include"), os.path.join(self.export_sources_folder, "include"))
        copy(self, "*", os.path.join(self.recipe_folder, "benchmark"), os.path.join(self.export_sources_folder, "benchmark"))
        copy(self, "*", os.path.join(self.recipe_folder, "stress_benchmark"),
             os.path.join(self.export_sources_folder, "stress_benchmark"))
        copy(self, "*", os.path.join(self.recipe_folder, "tests"), os.path.join(self.export_sources_folder, "tests"))

    def config_options(self):
        if not self.options.enable_plugins:
            del self.options.enable_remote_plugins
        sentry_project = self.conf.get("user.curaengine:sentry_project", "", check_type=str)
        sentry_org = self.conf.get("user.curaengine:sentry_org", "", check_type=str)
        if os.environ.get('SENTRY_TOKEN', None) is None or sentry_project == "" or sentry_org == "":
            del self.options.enable_sentry

    def configure(self):
        self.options["clipper"].shared = True
        if self.options.enable_arcus or self.options.enable_plugins:
            self.options["protobuf"].shared = False
        if self.options.enable_arcus:
            self.options["arcus"].shared = True
        if self.settings.os == "Linux":
            self.options["openssl"].shared = True
        if self.options.get_safe("enable_sentry", False):
            self.options["sentry-native"].backend = "breakpad"
            self.options["arcus"].enable_sentry = True
            self.options["clipper"].enable_sentry = True

    def validate(self):
        if self.settings.compiler.get_safe("cppstd"):
            check_min_cppstd(self, 20)
        if self.version:
            if Version(self.version) <= Version("4"):
                raise ConanInvalidConfiguration("only versions 5+ are supported")

    def build_requirements(self):
        self.test_requires("standardprojectsettings/[>=0.2.0]@ultimaker/cura_11622")  # FIXME: use stable after merge
        if self.options.enable_arcus or self.options.enable_plugins:
            self.tool_requires("protobuf/3.21.12")
        if not self.conf.get("tools.build:skip_test", False, check_type=bool):
            self.test_requires("gtest/1.14.0")
        if self.options.enable_benchmarks:
            self.test_requires("benchmark/1.8.3")
            self.test_requires("docopt.cpp/0.6.3")

    def requirements(self):
        for req in self.conan_data["requirements"]:
            self.requires(req)
        if self.options.enable_arcus:
            for req in self.conan_data["requirements_arcus"]:
                self.requires(req)
        if self.options.get_safe("enable_sentry", False):
            self.requires("sentry-native/0.7.0")
        if self.options.enable_plugins:
            self.requires("neargye-semver/0.3.0")
            self.requires("asio-grpc/2.9.2")
            for req in self.conan_data["requirements_plugins"]:
                self.requires(req)
        if self.options.with_cura_resources:
            for req in self.conan_data["requirements_cura_resources"]:
                self.requires(req)
        if self.options.enable_arcus or self.options.enable_plugins:
            self.requires("protobuf/3.21.12")
        self.requires("clipper/6.4.2@ultimaker/cura_11622") # FIXME: use main after merge
        self.requires("boost/1.83.0")
        self.requires("rapidjson/cci.20230929")
        self.requires("stb/cci.20230920")
        self.requires("spdlog/1.12.0")
        self.requires("fmt/10.2.1")
        self.requires("range-v3/0.12.0")
        self.requires("zlib/1.3.1")
        self.requires("openssl/3.2.1")
        self.requires("mapbox-wagyu/0.5.0@ultimaker/cura_11622") # FIXME: use main after merge

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
        tc.variables["ENABLE_THREADING"] = not (self.settings.arch == "wasm" and self.settings.os == "Emscripten")
        if self.options.get_safe("enable_sentry", False):
            tc.variables["ENABLE_SENTRY"] = True
            tc.variables["SENTRY_URL"] = self.conf.get("user.curaengine:sentry_url", "", check_type=str)
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

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

        if self.options.get_safe("enable_sentry", False):
            # Upload debug symbols to sentry
            sentry_project = self.conf.get("user.curaengine:sentry_project", "", check_type=str)
            sentry_org = self.conf.get("user.curaengine:sentry_org", "", check_type=str)
            if sentry_project == "" or sentry_org == "":
                raise ConanInvalidConfiguration("sentry_project or sentry_org is not set")

            if which("sentry-cli") is None:
                self.output.warn("sentry-cli is not installed, skipping uploading debug symbols")
                self.output.warn("sentry-cli is not installed, skipping release creation")
            else:
                if self.settings.os == "Linux":
                    self.output.info("Stripping debug symbols from binary")
                    self.run("objcopy --only-keep-debug --compress-debug-sections=zlib CuraEngine CuraEngine.debug")
                    self.run("objcopy --strip-debug --strip-unneeded CuraEngine")
                    self.run("objcopy --add-gnu-debuglink=CuraEngine.debug CuraEngine")
                elif self.settings.os == "Macos":
                    self.run("dsymutil CuraEngine")

                self.output.info("Uploading debug symbols to sentry")
                build_source_dir = self.build_path.parent.parent.as_posix()
                self.run(
                    f"sentry-cli --auth-token {os.environ['SENTRY_TOKEN']} debug-files upload --include-sources -o {sentry_org} -p {sentry_project} {build_source_dir}")

                # create a sentry release and link it to the commit this is based upon
                self.output.info(
                    f"Creating a new release {self.version} in Sentry and linking it to the current commit {self.conan_data['commit']}")
                self.run(
                    f"sentry-cli --auth-token {os.environ['SENTRY_TOKEN']} releases new -o {sentry_org} -p {sentry_project} {self.version}")
                self.run(
                    f"sentry-cli --auth-token {os.environ['SENTRY_TOKEN']} releases set-commits -o {sentry_org} -p {sentry_project} --commit \"Ultimaker/CuraEngine@{self.conan_data['commit']}\" {self.version}")
                self.run(
                    f"sentry-cli --auth-token {os.environ['SENTRY_TOKEN']} releases finalize -o {sentry_org} -p {sentry_project} {self.version}")

    def deploy(self):
        copy(self, "CuraEngine*", src=os.path.join(self.package_folder, "bin"), dst=self.install_folder)

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

---
name: conan-cura-contract
description: Keep CuraEngine Conan and CMake changes aligned with desktop consumers and shared workflows.
trigger: glob
glob: "**/conanfile.py,conanfile.py,**/conandata.yml,conandata.yml,**/CMakeLists.txt,CMakeLists.txt,**/*.cmake,.github/workflows/*.yml"
paths:
  - "**/conanfile.py"
  - "conanfile.py"
  - "**/conandata.yml"
  - "conandata.yml"
  - "**/CMakeLists.txt"
  - "CMakeLists.txt"
  - "**/*.cmake"
  - ".github/workflows/*.yml"
---
# Conan, CMake and downstream consumers

Load `conan-2` for the recipe, `cmake` for targets, and `cpp-pro` for changes to the supported C++ standard. `conanfile.py:118-151` selects dependencies from `conandata.yml` by options (Arcus, plugins, Cura resources, CuraViz), with `CMakeDeps`/`CMakeToolchain` in `generate()`. Check the exact dependency and recipe in `Ultimaker/conan-ultimaker-index`, profiles in `Ultimaker/conan-config`, and the reusable job in `Ultimaker/cura-workflows` on GitHub (or local checkouts when available) before adding a second dependency manager or overriding a profile option. Respect build/host profile separation and platform settings; the package is an application shipped inside Cura (`conanfile.py:17-46`), not an embedded image.

Arcus protobuf sources are generated from `Cura.proto` when enabled; plugin builds depend on `curaengine_grpc_definitions` (`CMakeLists.txt:20-40`). Changes to a field number, service method, version, exported target, or `CuraEngine` executable location need downstream Cura and plugin compatibility checks. The engine and `CuraEngine_plugin_infill_generate` currently depend on different grpc-definitions versions; do not assume compatibility or update a pin without a consumer test. Emscripten has distinct Conan requirements and threading options (`conanfile.py:100-105,118-155`): validate its package workflow when affected without imposing WASM restrictions on every native implementation.

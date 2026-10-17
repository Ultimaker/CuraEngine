<!-- @generated from AGENTS.md. Edit AGENTS.md and re-run sync. -->
# Agent Operational & Onboarding Guide (AGENTS.md)

This document explains the main structure of the CuraEngine application.

As a dynamic assistant, you must adhere strictly to these principles to maintain codebase sanity and ensure future developers can build upon your work efficiently.


# Global architecture

## Application description

The repository contains the full code to build CuraEngine, a standalone executable that implements the slicing of a 3D model into a GCode that can be read by a 3D printer. The global structure is the following:

* Load the 3D mesh(es) and their associated settings
* Slice the meshes to get a list of 2D polygons
* For each layer, turn the polygons into a list of extrusion paths that will form the model
* For each layer, translate the extrusion paths into actual GCode, while applying a few last-time modifications
* Send the extrusion data (with metadata) to the front-end, and the final gcode alongside

Since the input meshes can have very different shapes, we try to handle all the possible cases and use safe code as much as possible. We also focus very much on efficiency, since some meshes can have a very large number of triangles, or be large in physical size, which means the amount of generated extrusions is huge.

## Development

### Codebase
The codebase is essentially C++. Some parts of it are quite old, and possibly written at a time where there were no strict rules. But every time we make changes, we try to upgrade it with modern standards. The one we use is C++20, so not all features of modern C++ are available to us, because we need to support old platforms that don't support modern compilers. However we try to leverage the modern features as much as possible, in order to simplify our code, make it more portable and faster.

The application will be built for Linux, Windows and Mac platforms. So we have many specific cases here and there for each platform, and it is important that they all keep working.

### Testing
Some parts of the application have very exhaustive unit tests. However we don't always add new tests when adding or changing a feature. Mostly when this is really relevant.

### Package management
Dependencies of CuraEngine are handled using conan2. Most of the recipes are taken from the conan center, but some are custom recipes that we have created/forked. CuraEngine is also a package that is consumed by the global application, Cura, that contains a front-end which calls CuraEngine.

### Project tools and libraries
The project uses various external tools:
* CMake for building
* protobuf to generate messages for the front-end application

And also some major libraries:
* ranges-v3 (because we don't support std::ranges yet)
* libfmt
* clipperlib, for geometrical operations (union, intersections, difference, offsetting)
* boost

It also has a few unit testing and benchmarking sub-projects that are run periodically, so it is critical that they keep working.

### Computational geometry
Given its nature, CuraEngine contains a lot of computational geometry algorithms. The main library we use is clipper. Since it works only with integer values, we have adopted the following conventions for numeric types:
* By default, geometric coordinates are typed with the `coord_t` type, which is an alias to `signed long long`. Its derivatives can also be used: `Point2LL` and `Point3LL`. This way we can give those elements directly to clipper. Physical distances and positions in world space use micrometres and should use this type; `coord_t` can also hold counts and squared distances, so check units at each operation.
* When we require floating-point calculation, we use `float` type by default, and its derivatives: `Point2F` and `Point3F`
* When we require floating-point calculation with a specific need for precision, we use the `double` type, and its derivatives: `Point2D` and `Point3D`

There are also a few specific types that are defined in the engine and that are to be used in all relevant situations. They help making the code more explicit:
* `AngleDegrees` and `AngleRadians` types to store all the angle values
* `Ratio` type when storing a value that is to be multiplied, like speed or flow factor
* `Duration` type to store all the processing and print durations
* `LayerIndex` type to store the index of a layer
* `Temperature` type to store heating temperature
* `Velocity` and `Acceleration` types to store speeds and acceleration, typically of the print head

To ensure the handling of edge-cases in geometrical calculations, we consider geometric points to be coincident if they are 5 microns or less apart. For this purpose, there are global EPSILON and EPSILON_SQUARED values defined and some convenience methods that are defined to help the developers. They should be used whenever there is a possibility of an edge-case, to properly handle it and make sure the code is robust and repeatable.

## Architecture and integration map

CuraEngine is a desktop backend process, not printer firmware. Cura's Python/QML front end (using Uranium) locates the `CuraEngine` executable (`CuraEngine.exe` on Windows) and talks to it through libArcus and messages defined in `Cura.proto`. The engine can also run independently from the command line. The Conan recipe packages it for Cura; `conandata.yml` supplies dependency versions. Workflows in `.github/workflows/` delegate builds, tests, linting and packaging to `Ultimaker/cura-workflows`.

The slicing path is `Slice::compute` -> `Scene::processMeshGroup` -> `FffPolygonGenerator::generateAreas` -> `SliceDataStorage` -> `FffGcodeWriter::writeGCode` -> optimized layer communication and finalization (`src/Slice.cpp:22-42`, `src/Scene.cpp:66-100`). `Settings::get<T>` resolves serialized settings through the scene/mesh-group/extruder hierarchy (`include/settings/Settings.h:41-74`, `src/Scene.cpp:20-25`, `src/Slice.cpp:31-38`). Polygon operations use integer micrometre coordinates (`include/geometry/Point2LL.h:8-40`). `Communication` is the abstract boundary for Arcus and command-line implementations; layer completion and optimized-layer delivery are distinct events (`include/communication/Communication.h:22-32,46-62,102-109`). Unit tests mock this boundary (`tests/arcus/MockCommunication.h`); slicing integration tests exercise geometry (`tests/integration/SlicePhaseTest.cpp`).

Plugins use gRPC and `curaengine_grpc_definitions` when enabled (`CMakeLists.txt:20-40`). That definition package is a separate Conan dependency; check service and field changes against the engine and plugin consumers. Settings formulas are resolved through CuraFormulaeEngine; verify changed setting names and semantics across Cura and the engine. The G-code exporter produces printer instructions; it does not parse or dispatch printer commands like firmware. For executable packaging, Arcus messages, plugin definitions, or settings contracts, inspect `Ultimaker/Cura`, `Ultimaker/CuraEngine_grpc_definitions`, and `Ultimaker/CuraFormulaeEngine` on GitHub (or local checkouts when available); do not assume sibling checkouts exist.

## Development and verification entry points

The supported minimum is C++20 (`conanfile.py:105-115`); C++23-only facilities such as `std::expected`, `std::to_underlying`, and deducing `this` are not available by default. Conan 2 supplies dependencies and generates the CMake toolchain (`conanfile.py:118-175`). Install using appropriate Conan host/build profiles and the shared `Ultimaker/conan-config`, then use the generated CMake preset. CTest runs C++ tests after a test-enabled Conan build. Package, unit-test, clang-format, and clang-tidy workflows are in `.github/workflows/` and call reusable jobs in `Ultimaker/cura-workflows`. Windows contributors can run Python-based checks in PowerShell; Bash-only maintenance scripts require Git Bash or WSL.

Structural C++ checks in `rules/cpp/` run through `.agents/hooks/check_ast_grep.py` on changed lines. Pre-commit installs its pinned `ast-grep-cli` dependency; for immediate post-edit feedback outside pre-commit, install `ast-grep-cli==0.45.1` in your Python environment. Run `python .agents/hooks/test_check_ast_grep.py` to verify the positive and negative rule fixtures.

The repository-specific Copilot PR review instructions live in `.github/skills/code-review/SKILL.md`. Findings should point to changed lines and observable risks; formatting is handled by CI. For C++ work use `cpp-pro`, for CMake use `cmake`, for recipe/dependency changes use `conan-2`, and for architectural seams use `software-architect`. Check the project's standard and tests before applying generic skill advice.

`.github/workflows/copilot-setup-steps.yml` installs those four UltiCortex skills in the hosted review runner, pinned to a specific commit; it does not replace the repository's `code-review` skill. The workflow needs an `ULTICORTEX_READ_TOKEN` Agents secret with read-only access to `Ultimaker/UltiCortex`. GitHub uses review setup workflows only after they are on the default branch. For a manual workflow-dispatch check, configure an Actions secret of the same name too; otherwise that check has no access to Agents secrets.

Agent configuration is sourced from `.agents/rules/` and mirrored as regular files into `.claude/rules/` and `.opencode/rules/` so Windows checkouts without Git symlink support can read it. After changing a custom rule, run `scripts/sync_agentic_configs.sh` in Git Bash or WSL (or update the regular-file mirrors together) and check `.github/copilot-instructions.md`. The bootstrap profile is a detector snapshot, not an architectural authority: its G-code export file was initially mistaken for firmware command dispatch. Do not rerun the upstream bootstrap with `--update` blindly; it can regenerate rejected firmware and C++23 guidance. Inspect the resulting diff and preserve CuraEngine-specific custom rules and the Copilot review skill.

---
name: cpp-slicing-contract
description: Preserve CuraEngine's C++20 geometry, slicing, settings and communication contracts.
trigger: glob
glob: "**/*.cpp,**/*.h,**/*.hpp,**/CMakeLists.txt,CMakeLists.txt,**/*.proto,Cura.proto"
paths:
  - "**/*.cpp"
  - "**/*.h"
  - "**/*.hpp"
  - "**/CMakeLists.txt"
  - "CMakeLists.txt"
  - "**/*.proto"
  - "Cura.proto"
---
# C++20 slicing and communication

Load `cpp-pro` for C++ changes and `software-architect` only for architectural seam changes. `conanfile.py:105-115` requires C++20, not C++23. Use the repository's range-v3 and typed units (`coord_t` and `Point2LL` in micrometres, `Ratio`, `LayerIndex`, etc.) rather than substituting raw floating-point coordinates or C++23 library facilities. The `EPSILON` tolerance applies where geometric coincidence matters (`include/utils/Coord_t.h`). Preserve ownership and exception safety using established RAII; nullable non-owning pointers in legacy APIs are not evidence of ownership.

Trace changes through `Slice::compute` -> `Scene::processMeshGroup` -> `FffPolygonGenerator::generateAreas` -> `SliceDataStorage` -> `FffGcodeWriter::writeGCode` (`src/Slice.cpp:22-42`, `src/Scene.cpp:66-100`). Preserve settings inheritance and typed lookup (`include/settings/Settings.h:41-74`). Keep Arcus and command-line transports behind `Communication`; `sendLayerComplete` and `sendOptimizedLayerData` represent distinct stages (`include/communication/Communication.h:46-62,102-109`). Check tests under `tests/` or add a characterization test at that boundary when behavior changes.

When adding a beading algorithm, inspect the existing `BeadingStrategy` family and `BeadingStrategyFactory` before introducing another dispatch mechanism (`src/BeadingStrategy/`, `CMakeLists.txt`). Do not introduce CRTP, type-erasure, or new factories solely because a pattern is available; prefer the established seam when it fits and a plain function otherwise. The structural rules in `rules/cpp/` check changed C++ lines through `.agents/hooks/check_ast_grep.py` and run after supported agent edits; keep rules specific to this desktop C++20 slicer. Do not port Ultimoco's C++23, firmware or raw-quantity bans.

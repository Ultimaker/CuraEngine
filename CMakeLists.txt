#Copyright (c) 2021 Ultimaker B.V.
#CuraEngine is released under the terms of the AGPLv3 or higher.

cmake_minimum_required(VERSION 3.13)
project(CuraEngine)

set(CURA_ENGINE_VERSION "master" CACHE STRING "Version name of Cura")
option(ENABLE_ARCUS "Enable support for ARCUS" ON)
option(BUILD_TESTS "Build with unit tests" OFF)
option(ENABLE_OPENMP "Use OpenMP for parallel code" ON)
option(ENABLE_MORE_COMPILER_OPTIMIZATION_FLAGS "Enable more optimization flags" ON)
option(USE_SYSTEM_LIBS "Use the system libraries if available" OFF)
option (ENABLE_MORE_COMPILER_OPTIMIZATION_FLAGS "Enable more optimization flags" ON)

include(cmake/StandardProjectSettings.cmake)

# Create Protobuf files if Arcus is used
if (ENABLE_ARCUS)
    message(STATUS "Building with Arcus")

    find_package(Arcus REQUIRED)
    if(NOT TARGET Arcus::Arcus)
        add_library(Arcus::Arcus INTERFACE IMPORTED)
        target_link_libraries(Arcus::Arcus INTERFACE Arcus)
    endif()
    find_package(Protobuf 3.9.2 REQUIRED)

    protobuf_generate_cpp(engine_PB_SRCS engine_PB_HEADERS Cura.proto)
endif ()

### Compiling CuraEngine ###
# First compile all of CuraEngine as library, allowing this to be re-used for tests.

set(engine_SRCS # Except main.cpp.
        src/Application.cpp
        src/bridge.cpp
        src/ConicalOverhang.cpp
        src/ExtruderTrain.cpp
        src/FffGcodeWriter.cpp
        src/FffPolygonGenerator.cpp
        src/FffProcessor.cpp
        src/gcodeExport.cpp
        src/GCodePathConfig.cpp
        src/infill.cpp
        src/InsetOrderOptimizer.cpp
        src/layerPart.cpp
        src/LayerPlan.cpp
        src/LayerPlanBuffer.cpp
        src/MergeInfillLines.cpp
        src/mesh.cpp
        src/MeshGroup.cpp
        src/Mold.cpp
        src/multiVolumes.cpp
        src/PathOrder.cpp
        src/pathOrderOptimizer.cpp
        src/Preheat.cpp
        src/PrimeTower.cpp
        src/raft.cpp
        src/Scene.cpp
        src/skin.cpp
        src/SkirtBrim.cpp
        src/SupportInfillPart.cpp
        src/Slice.cpp
        src/sliceDataStorage.cpp
        src/slicer.cpp
        src/support.cpp
        src/timeEstimate.cpp
        src/TopSurface.cpp
        src/TreeSupport.cpp
        src/WallsComputation.cpp
        src/wallOverlap.cpp
        src/Weaver.cpp
        src/Wireframe2gcode.cpp

        src/communication/ArcusCommunication.cpp
        src/communication/ArcusCommunicationPrivate.cpp
        src/communication/CommandLine.cpp
        src/communication/Listener.cpp

    src/infill/ImageBasedDensityProvider.cpp
    src/infill/NoZigZagConnectorProcessor.cpp
    src/infill/ZigzagConnectorProcessor.cpp
    src/infill/LightningDistanceField.cpp
    src/infill/LightningGenerator.cpp
    src/infill/LightningLayer.cpp
    src/infill/LightningTreeNode.cpp
    src/infill/SierpinskiFill.cpp
    src/infill/SierpinskiFillProvider.cpp
    src/infill/SubDivCube.cpp
    src/infill/GyroidInfill.cpp

        src/pathPlanning/Comb.cpp
        src/pathPlanning/GCodePath.cpp
        src/pathPlanning/LinePolygonsCrossings.cpp
        src/pathPlanning/NozzleTempInsert.cpp
        src/pathPlanning/TimeMaterialEstimates.cpp

        src/progress/Progress.cpp
        src/progress/ProgressStageEstimator.cpp

        src/settings/AdaptiveLayerHeights.cpp
        src/settings/FlowTempGraph.cpp
        src/settings/PathConfigStorage.cpp
        src/settings/Settings.cpp
        src/settings/ZSeamConfig.cpp

    src/utils/AABB.cpp
    src/utils/AABB3D.cpp
    src/utils/Date.cpp
    src/utils/FMatrix4x3.cpp
    src/utils/gettime.cpp
    src/utils/getpath.cpp
    src/utils/LinearAlg2D.cpp
    src/utils/ListPolyIt.cpp
    src/utils/logoutput.cpp
    src/utils/MinimumSpanningTree.cpp
    src/utils/Point3.cpp
    src/utils/PolygonConnector.cpp
    src/utils/PolygonsPointIndex.cpp
    src/utils/PolygonProximityLinker.cpp
    src/utils/polygonUtils.cpp
    src/utils/polygon.cpp
    src/utils/ProximityPointLink.cpp
    src/utils/SVG.cpp
    src/utils/socket.cpp
    src/utils/SquareGrid.cpp
)

add_library(_CuraEngine STATIC ${engine_SRCS} ${engine_PB_SRCS})
use_threads(_CuraEngine)

target_include_directories(_CuraEngine
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> # Include Cura.pb.h
        )
target_compile_definitions(_CuraEngine
        PUBLIC
            $<$<BOOL:${BUILD_TESTS}>:BUILD_TESTS=1>
            $<$<BOOL:${BUILD_TESTS}>:BUILD_TESTS>
            $<$<BOOL:${ENABLE_ARCUS}>:ARCUS>
        PRIVATE
            VERSION=\"${CURA_ENGINE_VERSION}\"
            $<$<BOOL:${WIN32}>:NOMINMAX>
            $<$<CONFIG:Debug>:ASSERT_INSANE_OUTPUT>
            $<$<CONFIG:Debug>:USE_CPU_TIME>
            $<$<CONFIG:Debug>:DEBUG>
            $<$<CONFIG:RelWithDebInfo>:ASSERT_INSANE_OUTPUT>
            $<$<CONFIG:RelWithDebInfo>:USE_CPU_TIME>
            $<$<CONFIG:RelWithDebInfo>:DEBUG>
        )
if(MSVC)
    target_compile_options(_CuraEngine
            PRIVATE
                $<$<AND:$<BOOL:${ENABLE_MORE_COMPILER_OPTIMIZATION_FLAGS}>,$<CONFIG:Release>>:/fp:fast>
            )
else()
    target_compile_options(_CuraEngine
            PRIVATE
                $<$<CONFIG:Debug>:-fno-omit-frame-pointer>
                $<$<CONFIG:RelWithDebInfo>:-fno-omit-frame-pointer>
                $<$<AND:$<BOOL:${ENABLE_MORE_COMPILER_OPTIMIZATION_FLAGS}>,$<CONFIG:Release>>:-Ofast>
                $<$<AND:$<BOOL:${ENABLE_MORE_COMPILER_OPTIMIZATION_FLAGS}>,$<CONFIG:Release>>:-funroll-loops>
            )
endif()

set_project_standards(_CuraEngine)
set_project_warnings(_CuraEngine)
enable_sanitizers(_CuraEngine)

if(ENABLE_OPENMP)
    find_package(OpenMP REQUIRED)
    target_link_libraries(_CuraEngine PUBLIC OpenMP::OpenMP_CXX)
endif()

if(ENABLE_ARCUS)
    target_link_libraries(_CuraEngine PUBLIC Arcus::Arcus protobuf::libprotobuf)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
if(USE_SYSTEM_LIBS)
    find_package(RapidJSON QUIET)
    find_package(polyclipping QUIET)
endif()

if(NOT TARGET RapidJSON::RapidJSON)
    message(STATUS "RapidJSON: Using Shipped library.")
    set(rapidjson_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/libs/rapidjson")
    target_include_directories(_CuraEngine PUBLIC ${rapidjson_INCLUDE_DIRS})
else()
    target_link_libraries(_CuraEngine PUBLIC RapidJSON::RapidJSON)
endif()

if(NOT TARGET polyclipping::polyclipping)
    add_subdirectory(libs/clipper)
endif()
target_link_libraries(_CuraEngine PUBLIC polyclipping::polyclipping)

find_package(stb REQUIRED)
if(CuraEngine_Download_Stb)
    add_dependencies(_CuraEngine stb)
endif()
target_link_libraries(_CuraEngine PRIVATE stb::stb)

### Compile the CLI ###
if(WIN32)
    message(STATUS "Using windres")
    set(RES_FILES "CuraEngine.rc")
    ENABLE_LANGUAGE(RC)
    if(NOT MSVC)
        SET(CMAKE_RC_COMPILER_INIT windres)
        SET(CMAKE_RC_COMPILE_OBJECT "<CMAKE_RC_COMPILER> <FLAGS> -O coff <DEFINES> -i <SOURCE> -o <OBJECT>")
    endif()
endif(WIN32)

# Then compile main.cpp as separate executable, and link the library to it.
if (NOT WIN32)
    add_executable(CuraEngine src/main.cpp)
else()
    add_executable(CuraEngine src/main.cpp ${RES_FILES}) # ..., but don't forget the glitter!
endif(NOT WIN32)
target_link_libraries(CuraEngine PRIVATE _CuraEngine)
set_project_standards(CuraEngine)
set_project_warnings(CuraEngine)
use_threads(CuraEngine)

# Compiling the test environment.
if (BUILD_TESTS)
    add_subdirectory(tests)
endif()

# Installing CuraEngine.
install(TARGETS CuraEngine DESTINATION "${CMAKE_INSTALL_BINDIR}")
# For MinGW64 cross compiling on Debian, we create a ZIP package instead of a DEB
# Because it's the Windows build system that should install the files.
if (CMAKE_CROSSCOMPILING AND CMAKE_SYSTEM_NAME MATCHES "Windows")
    message(STATUS "Include MinGW64 posix DLLs for installation.")
    install(FILES
            /usr/lib/gcc/x86_64-w64-mingw32/8.3-posix/libgcc_s_seh-1.dll
            /usr/lib/gcc/x86_64-w64-mingw32/8.3-posix/libgomp-1.dll
            /usr/lib/gcc/x86_64-w64-mingw32/8.3-posix/libstdc++-6.dll
            DESTINATION bin
            COMPONENT runtime)
endif ()
include(CPackConfig.cmake)

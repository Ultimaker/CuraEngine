#Copyright (c) 2022 Ultimaker B.V.
#CuraEngine is released under the terms of the AGPLv3 or higher.

cmake_minimum_required(VERSION 3.20)
project(CuraEngine)

set(CURA_ENGINE_VERSION "master" CACHE STRING "Version name of Cura")
option(ENABLE_ARCUS "Enable support for ARCUS" ON)
option(BUILD_TESTS "Build with unit tests" OFF)
option(ENABLE_MORE_COMPILER_OPTIMIZATION_FLAGS "Enable more optimization flags" ON)
option(USE_SYSTEM_LIBS "Use the system libraries if available" OFF)
option(ENABLE_MORE_COMPILER_OPTIMIZATION_FLAGS "Enable more optimization flags" ON)
option(EXTENSIVE_WARNINGS "Compile with all warnings" OFF)

if(NOT APPLE)
    option(ENABLE_OPENMP "Use OpenMP for parallel code" ON)
endif()

include(cmake/StandardProjectSettings.cmake)

# Create Protobuf files if Arcus is used
if (ENABLE_ARCUS)
    message(STATUS "Building with Arcus")

    find_package(Arcus REQUIRED 5.0.0)
    find_package(protobuf 3.15.0 QUIET)
    if (NOT TARGET protobuf::protobuf)
        find_package(Protobuf 3.15.0 REQUIRED)
    else ()
        add_library(Protobuf::protobuf ALIAS protobuf::protobuf)
    endif ()

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
        src/mesh.cpp
        src/MeshGroup.cpp
        src/Mold.cpp
        src/multiVolumes.cpp
        src/PathOrderOptimizer.cpp
        src/PathOrder.cpp
        src/Preheat.cpp
        src/PrimeTower.cpp
        src/raft.cpp
        src/Scene.cpp
        src/SkeletalTrapezoidation.cpp
        src/SkeletalTrapezoidationGraph.cpp
        src/skin.cpp
        src/SkirtBrim.cpp
        src/SupportInfillPart.cpp
        src/Slice.cpp
        src/sliceDataStorage.cpp
        src/slicer.cpp
        src/support.cpp
        src/timeEstimate.cpp
        src/TopSurface.cpp
        src/TreeModelVolumes.cpp
        src/TreeSupport.cpp
        src/WallsComputation.cpp
        src/Weaver.cpp
        src/Wireframe2gcode.cpp
        src/WallToolPaths.cpp

        src/BeadingStrategy/BeadingStrategy.cpp
        src/BeadingStrategy/BeadingStrategyFactory.cpp
        src/BeadingStrategy/CenterDeviationBeadingStrategy.cpp
        src/BeadingStrategy/DistributedBeadingStrategy.cpp
        src/BeadingStrategy/LimitedBeadingStrategy.cpp
        src/BeadingStrategy/RedistributeBeadingStrategy.cpp
        src/BeadingStrategy/WideningBeadingStrategy.cpp
        src/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp

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
        src/utils/ExtrusionJunction.cpp
        src/utils/ExtrusionLine.cpp
        src/utils/ExtrusionSegment.cpp
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
        src/utils/PolygonsSegmentIndex.cpp
        src/utils/polygonUtils.cpp
        src/utils/polygon.cpp
        src/utils/PolylineStitcher.cpp
        src/utils/ProximityPointLink.cpp
        src/utils/SVG.cpp
        src/utils/socket.cpp
        src/utils/SquareGrid.cpp
        src/utils/ThreadPool.cpp
        src/utils/ToolpathVisualizer.cpp
        src/utils/VoronoiUtils.cpp
        )

# List of tests. For each test there must be a file tests/${NAME}.cpp.
set(engine_TEST
        ClipperTest
        ExtruderPlanTest
        GCodeExportTest
        InfillTest
        LayerPlanTest
        PathOrderOptimizerTest
        PathOrderMonotonicTest
        TimeEstimateCalculatorTest
        WallsComputationTest
        )
if (ENABLE_ARCUS)
    set(engine_TEST_ARCUS
            ArcusCommunicationTest
            ArcusCommunicationPrivateTest
            )
endif ()
set(engine_TEST_BEADING_STRATEGY
        CenterDeviationBeadingStrategyTest
        )
set(engine_TEST_INFILL
        )
set(engine_TEST_INTEGRATION
        SlicePhaseTest
        )
set(engine_TEST_SETTINGS
        SettingsTest
        )
set(engine_TEST_UTILS
        AABBTest
        AABB3DTest
        ExtrusionLineTest
        IntPointTest
        LinearAlg2DTest
        MinimumSpanningTreeTest
        PolygonConnectorTest
        PolygonTest
        PolygonUtilsTest
        SparseGridTest
        StringTest
        UnionFindTest
        )

# Helper classes for some tests.
set(engine_TEST_ARCUS_HELPERS
        tests/arcus/MockSocket.cpp
        )
set(engine_TEST_HELPERS
        tests/ReadTestPolygons.cpp
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
enable_sanitizers(_CuraEngine)
set_rpath(TARGETS
        _CuraEngine
        PATHS
        "$<$<PLATFORM_ID:Linux>:usr/bin>"
        "$<$<PLATFORM_ID:Linux>:usr/bin/lib>"
        "$<$<PLATFORM_ID:Darwin>:../lib>"
        RELATIVE)

if(${EXTENSIVE_WARNINGS})
    set_project_warnings(_CuraEngine)
endif()

if(NOT APPLE AND ENABLE_OPENMP)
    find_package(OpenMP REQUIRED)
    target_link_libraries(_CuraEngine PUBLIC OpenMP::OpenMP_CXX)
endif()

if(ENABLE_ARCUS)
    target_link_libraries(_CuraEngine PRIVATE Arcus protobuf::protobuf)
endif()

find_package(clipper 6.4.2 QUIET)
if (NOT TARGET clipper::clipper)
    add_library(clipper::clipper STATIC libs/clipper/clipper.cpp)
    target_include_directories(clipper::clipper
            PUBLIC
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/libs/clipper/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
            )
endif ()

find_package(rapidjson 1.1.0 QUIET)
if (NOT TARGET rapidjson::rapidjson)
    add_library(rapidjson::rapidjson INTERFACE)
    target_include_directories(rapidjson::rapidjson
            PUBLIC
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/libs/clipper/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
            )
endif()

find_package(stb REQUIRED)

find_package(Boost REQUIRED 1.75.0)
if (NOT TARGET Boost::boost)  # Workaround for different namespace in non Conan find_packages
    add_library(boost::boost ALIAS Boost::boost)
endif ()

target_link_libraries(_CuraEngine PRIVATE clipper::clipper rapidjson::rapidjson stb::stb boost::boost)

if (WIN32)
    message(STATUS "Using windres")
    set(RES_FILES "CuraEngine.rc")
    ENABLE_LANGUAGE(RC)
    if (NOT MSVC)
        SET(CMAKE_RC_COMPILER_INIT windres)
        SET(CMAKE_RC_COMPILE_OBJECT
                "<CMAKE_RC_COMPILER> <FLAGS> -O coff <DEFINES> -i <SOURCE> -o <OBJECT>"
                )
    endif ()
endif (WIN32)

if (NOT WIN32)
    add_executable(CuraEngine src/main.cpp) # Then compile main.cpp as separate executable, and link the library to it.
else ()
    add_executable(CuraEngine src/main.cpp ${RES_FILES}) # ..., but don't forget the glitter!
endif (NOT WIN32)

# Create the executable
target_link_libraries(CuraEngine PRIVATE _CuraEngine)
target_compile_definitions(CuraEngine PRIVATE VERSION=\"${CURA_ENGINE_VERSION}\")

# Compiling the test environment.
if (BUILD_TESTS)
    include(CTest)

    message(STATUS "Building tests...")
    set(GTEST_USE_STATIC_LIBS true)
    set(GMOCK_ROOT "${CMAKE_CURRENT_BINARY_DIR}/gmock")
    set(GMOCK_VER "1.8.1")
    find_package(GMock REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})
    include_directories(${GMOCK_INCLUDE_DIRS})
    add_dependencies(_CuraEngine GTest::GTest GTest::Main GMock::GMock GMock::Main)
    add_definitions(-DBUILD_TESTS)

    target_compile_definitions(_CuraEngine PUBLIC BUILD_TESTS=1)

    #To make sure that the tests are built before running them, add the building of these tests as an additional test.
    add_custom_target(build_all_tests)
    add_test(BuildTests "${CMAKE_COMMAND}" --build "${CMAKE_CURRENT_BINARY_DIR}" --target build_all_tests)

    foreach (test ${engine_TEST})
        add_executable(${test} tests/main.cpp ${engine_TEST_HELPERS} tests/${test}.cpp)
        target_link_libraries(${test} _CuraEngine ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES})
        add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
        add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
    endforeach ()
    if (ENABLE_ARCUS)
        foreach (test ${engine_TEST_ARCUS})
            add_executable(${test} tests/main.cpp ${engine_TEST_ARCUS_HELPERS} tests/arcus/${test}.cpp)
            target_link_libraries(${test} _CuraEngine ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES})
            add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
            add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
        endforeach ()
    endif ()
    foreach (test ${engine_TEST_BEADING_STRATEGY})
        add_executable(${test} tests/main.cpp tests/beading_strategy/${test}.cpp)
        target_link_libraries(${test} _CuraEngine ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES})
        add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
        add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
    endforeach ()
    foreach (test ${engine_TEST_INFILL})
        add_executable(${test} tests/main.cpp tests/infill/${test}.cpp)
        target_link_libraries(${test} _CuraEngine ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES})
        add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
        add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
    endforeach ()
    foreach (test ${engine_TEST_INTEGRATION})
        add_executable(${test} tests/main.cpp tests/integration/${test}.cpp)
        target_link_libraries(${test} _CuraEngine ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES})
        add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
        add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
    endforeach ()
    foreach (test ${engine_TEST_SETTINGS})
        add_executable(${test} tests/main.cpp tests/settings/${test}.cpp)
        target_link_libraries(${test} _CuraEngine ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES})
        add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
        add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
    endforeach ()
    foreach (test ${engine_TEST_UTILS})
        add_executable(${test} tests/main.cpp tests/utils/${test}.cpp)
        target_link_libraries(${test} _CuraEngine ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES})
        add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
        add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
    endforeach ()
endif ()

# Installing CuraEngine.
include(GNUInstallDirs)
install(TARGETS CuraEngine DESTINATION "${CMAKE_INSTALL_BINDIR}")
include(CPackConfig.cmake)

project(CuraEngine)

cmake_minimum_required(VERSION 2.8.12)

option (ENABLE_ARCUS
    "Enable support for ARCUS" ON)

if (ENABLE_ARCUS)
    message(STATUS "Building with Arcus")
    find_package(Arcus REQUIRED)
    add_definitions(-DARCUS)
endif ()

if(NOT ${CMAKE_VERSION} VERSION_LESS 3.1)
    set(CMAKE_CXX_STANDARD 11)
else()
    set(CMAKE_CXX_FLAGS "-std=c++11")
endif()

if(APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")
endif()

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

set(CURA_ENGINE_VERSION "master" CACHE STRING "Version name of Cura")

option(BUILD_TESTS OFF)

# Add a compiler flag to check the output for insane values if we are in debug mode.
if(CMAKE_BUILD_TYPE MATCHES DEBUG OR CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    message(STATUS "Building debug release of CuraEngine.")
    add_definitions(-DASSERT_INSANE_OUTPUT)
    add_definitions(-DUSE_CPU_TIME)
    add_definitions(-DDEBUG)
endif()

# Add warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

if(NOT APPLE AND NOT WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libstdc++")
endif()

include_directories(${CMAKE_CURRENT_BINARY_DIR} libs)

add_library(clipper STATIC libs/clipper/clipper.cpp)

set(engine_SRCS # Except main.cpp.
    src/bridge.cpp
    src/commandSocket.cpp
    src/ConicalOverhang.cpp
    src/ExtruderTrain.cpp
    src/FffGcodeWriter.cpp
    src/FffPolygonGenerator.cpp
    src/FffProcessor.cpp
    src/gcodeExport.cpp
    src/GCodePathConfig.cpp
    src/gcodePlanner.cpp
    src/infill.cpp
    src/WallsComputation.cpp
    src/LayerPlanBuffer.cpp
    src/MergeInfillLines.cpp
    src/mesh.cpp
    src/MeshGroup.cpp
    src/pathOrderOptimizer.cpp
    src/Preheat.cpp
    src/PrimeTower.cpp
    src/raft.cpp
    src/skin.cpp
    src/SkirtBrim.cpp
    src/sliceDataStorage.cpp
    src/support.cpp
    src/timeEstimate.cpp
    src/WallsComputation.cpp
    src/wallOverlap.cpp
    src/Weaver.cpp
    src/Wireframe2gcode.cpp

    src/infill/NoZigZagConnectorProcessor.cpp
    src/infill/ZigzagConnectorProcessorConnectedEndPieces.cpp
    src/infill/ZigzagConnectorProcessorDisconnectedEndPieces.cpp
    src/infill/ZigzagConnectorProcessorEndPieces.cpp
    src/infill/ZigzagConnectorProcessorNoEndPieces.cpp
    src/infill/SubDivCube.cpp

    src/slicer/LayerPart.cpp
    src/slicer/MultiVolumes.cpp
    src/slicer/SlicerLayer.cpp
    src/slicer/Slicer.cpp

    src/textureProcessing/Material.cpp
    src/textureProcessing/MaterialBase.cpp
    src/textureProcessing/TexturedMesh.cpp
    src/textureProcessing/TextureBumpMapProcessor.cpp

    src/pathPlanning/Comb.cpp
    src/pathPlanning/GCodePath.cpp
    src/pathPlanning/LinePolygonsCrossings.cpp
    src/pathPlanning/NozzleTempInsert.cpp
    src/pathPlanning/TimeMaterialEstimates.cpp

    src/progress/Progress.cpp
    src/progress/ProgressStageEstimator.cpp

    src/settings/SettingConfig.cpp
    src/settings/SettingContainer.cpp
    src/settings/SettingRegistry.cpp
    src/settings/settings.cpp

    src/utils/AABB.cpp
    src/utils/AABB3D.cpp
    src/utils/Date.cpp
    src/utils/gettime.cpp
    src/utils/LinearAlg2D.cpp
    src/utils/ListPolyIt.cpp
    src/utils/logoutput.cpp
    src/utils/PolygonProximityLinker.cpp
    src/utils/polygonUtils.cpp
    src/utils/polygon.cpp
    src/utils/ProximityPointLink.cpp
)

# List of tests. For each test there must be a file tests/${NAME}.cpp and a file tests/${NAME}.h.
set(engine_TEST
    GCodePlannerTest
)
set(engine_TEST_INFILL
)
set(engine_TEST_UTILS
    SparseGridTest
    LinearAlg2DTest
    PolygonUtilsTest
    PolygonTest
    StringTest
)

# Generating ProtoBuf protocol
if (ENABLE_ARCUS)
protobuf_generate_cpp(engine_PB_SRCS engine_PB_HEADERS Cura.proto)
endif ()

# Compiling CuraEngine itself.
add_library(_CuraEngine ${engine_SRCS} ${engine_PB_SRCS}) #First compile all of CuraEngine as library, allowing this to be re-used for tests.
target_link_libraries(_CuraEngine clipper)
if (ENABLE_ARCUS)
    target_link_libraries(_CuraEngine Arcus)
endif ()

set_target_properties(_CuraEngine PROPERTIES COMPILE_DEFINITIONS "VERSION=\"${CURA_ENGINE_VERSION}\"")

if (UNIX)
    target_link_libraries(_CuraEngine pthread)
endif()
add_executable(CuraEngine src/main.cpp) #Then compile main.cpp as separate executable, and link the library to it.
target_link_libraries(CuraEngine _CuraEngine)

# Compiling the test environment.
if (BUILD_TESTS)
    message(STATUS "Building tests...")
    enable_testing()
    foreach (test ${engine_TEST})
        add_executable(${test} tests/main.cpp tests/${test}.cpp)
        target_link_libraries(${test} _CuraEngine cppunit)
        add_test(${test} ${test})
    endforeach()
    foreach (test ${engine_TEST_INFILL})
        add_executable(${test} tests/main.cpp tests/infill/${test}.cpp)
        target_link_libraries(${test} _CuraEngine cppunit)
        add_test(${test} ${test})
    endforeach()
    foreach (test ${engine_TEST_UTILS})
        add_executable(${test} tests/main.cpp tests/utils/${test}.cpp)
        target_link_libraries(${test} _CuraEngine cppunit)
        add_test(${test} ${test})
    endforeach()
endif()


add_custom_command(TARGET CuraEngine POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                       ${CMAKE_SOURCE_DIR}/resources $<TARGET_FILE_DIR:CuraEngine>)

# Installing CuraEngine.
include(GNUInstallDirs)
install(TARGETS CuraEngine DESTINATION ${CMAKE_INSTALL_BINDIR})
include(CPackConfig.cmake)

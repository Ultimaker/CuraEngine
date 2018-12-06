#Copyright (c) 2018 Ultimaker B.V.
#CuraEngine is released under the terms of the AGPLv3 or higher.

project(CuraEngine)

cmake_minimum_required(VERSION 2.8.12)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option (ENABLE_ARCUS
    "Enable support for ARCUS" ON)

if (ENABLE_ARCUS)
    message(STATUS "Building with Arcus")
    find_package(Arcus REQUIRED)
    add_definitions(-DARCUS)
endif ()

#For reading image files.
find_package(Stb REQUIRED)
include_directories(${Stb_INCLUDE_DIRS})

option(USE_SYSTEM_LIBS "Use the system libraries if available" OFF)
if(USE_SYSTEM_LIBS)
    find_package(RapidJSON CONFIG REQUIRED)
    find_package(Polyclipping REQUIRED)
endif()

# convert build type to upper case letters
if(CMAKE_BUILD_TYPE)
    string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPER)
endif()

if(CMAKE_BUILD_TYPE_UPPER MATCHES "DEBUG")
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_DEBUG_INIT})
else()
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_RELEASE_INIT})
endif()

if(NOT ${CMAKE_VERSION} VERSION_LESS 3.1)
    set(CMAKE_CXX_STANDARD 11)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

if(APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")
endif()

if(NOT DEFINED LIB_SUFFIX)
    set(LIB_SUFFIX "")
endif()
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}")

set(CURA_ENGINE_VERSION "master" CACHE STRING "Version name of Cura")

option(BUILD_TESTS OFF)

# Add a compiler flag to check the output for insane values if we are in debug mode.
if(CMAKE_BUILD_TYPE_UPPER MATCHES "DEBUG" OR CMAKE_BUILD_TYPE_UPPER MATCHES "RELWITHDEBINFO")
    message(STATUS "Building debug release of CuraEngine.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra -O0 -g -fno-omit-frame-pointer")
    add_definitions(-DASSERT_INSANE_OUTPUT)
    add_definitions(-DUSE_CPU_TIME)
    add_definitions(-DDEBUG)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall") # Add warnings

option (ENABLE_MORE_COMPILER_OPTIMIZATION_FLAGS
    "Enable more optimization flags" ON)
if (ENABLE_MORE_COMPILER_OPTIMIZATION_FLAGS)
    message (STATUS "Compile with more optimization flags")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Ofast -funroll-loops")
endif ()

if(NOT APPLE AND NOT WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libstdc++")
endif()

option (ENABLE_OPENMP
    "Use OpenMP for parallel code" ON)

if (ENABLE_OPENMP)
    FIND_PACKAGE( OpenMP )
    if( OPENMP_FOUND )
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}" )
    endif()
endif()

if(USE_SYSTEM_LIBS)
    include_directories(${Polyclipping_INCLUDE_DIRS} ${CMAKE_BINARY_DIR} ${RAPIDJSON_INCLUDE_DIRS})
else()
    include_directories(${CMAKE_CURRENT_BINARY_DIR} libs libs/clipper)
    add_library(clipper STATIC libs/clipper/clipper.cpp)
endif()

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
    src/infill/SierpinskiFill.cpp
    src/infill/SierpinskiFillProvider.cpp
    src/infill/SpaghettiInfill.cpp
    src/infill/SpaghettiInfillPathGenerator.cpp
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

    src/utils/AABB.cpp
    src/utils/AABB3D.cpp
    src/utils/Date.cpp
    src/utils/gettime.cpp
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
)

# List of tests. For each test there must be a file tests/${NAME}.cpp and a file tests/${NAME}.h.
set(engine_TEST
    TimeEstimateCalculatorTest
)
set(engine_TEST_INFILL
)
set(engine_TEST_SETTINGS
    SettingsTest
)
if (ENABLE_ARCUS)
    set(engine_TEST_ARCUS
        ArcusCommunicationTest
        ArcusCommunicationPrivateTest
    )
endif ()
set(engine_TEST_UTILS
    SparseGridTest
    IntPointTest
    LinearAlg2DTest
    PolygonConnectorTest
    PolygonUtilsTest
    PolygonTest
    StringTest
    UnionFindTest
)

# Helper classes for some tests.
set(engine_TEST_ARCUS_HELPERS
    tests/arcus/MockSocket.cpp
)

# Generating ProtoBuf protocol
if (ENABLE_ARCUS)
    protobuf_generate_cpp(engine_PB_SRCS engine_PB_HEADERS Cura.proto)
endif ()

# Compiling CuraEngine itself.
add_library(_CuraEngine STATIC ${engine_SRCS} ${engine_PB_SRCS}) #First compile all of CuraEngine as library, allowing this to be re-used for tests.

if (CuraEngine_Download_Stb)
    add_dependencies(_CuraEngine stb)
endif()
if(USE_SYSTEM_LIBS)
    target_link_libraries(_CuraEngine ${Polyclipping_LIBRARIES})
else()
    target_link_libraries(_CuraEngine clipper)
endif()

if (ENABLE_ARCUS)
    target_link_libraries(_CuraEngine Arcus)
endif ()

set_target_properties(_CuraEngine PROPERTIES COMPILE_DEFINITIONS "VERSION=\"${CURA_ENGINE_VERSION}\"")

if(WIN32)
  message(STATUS "Using windres")
  set(RES_FILES "CuraEngine.rc")
  set(CMAKE_RC_COMPILER_INIT windres)
  ENABLE_LANGUAGE(RC)
  SET(CMAKE_RC_COMPILE_OBJECT
      "<CMAKE_RC_COMPILER> <FLAGS> -O coff <DEFINES> -i <SOURCE> -o <OBJECT>"
      )
endif(WIN32)

if (UNIX)
    target_link_libraries(_CuraEngine pthread)
endif()

if (NOT WIN32)
  add_executable(CuraEngine src/main.cpp) # Then compile main.cpp as separate executable, and link the library to it.
else()
  add_executable(CuraEngine src/main.cpp ${RES_FILES}) # ..., but don't forget the glitter!
endif(NOT WIN32)

target_link_libraries(CuraEngine _CuraEngine)
set_target_properties(CuraEngine PROPERTIES COMPILE_DEFINITIONS "VERSION=\"${CURA_ENGINE_VERSION}\"")

# Compiling the test environment.
if (BUILD_TESTS)
    message(STATUS "Building tests...")

    find_package(cppunit)

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
    foreach (test ${engine_TEST_SETTINGS})
        add_executable(${test} tests/main.cpp tests/settings/${test}.cpp)
        target_link_libraries(${test} _CuraEngine cppunit)
        add_test(${test} ${test})
    endforeach()
    if (ENABLE_ARCUS)
        foreach (test ${engine_TEST_ARCUS})
            add_executable(${test} tests/main.cpp ${engine_TEST_ARCUS_HELPERS} tests/arcus/${test}.cpp)
            target_link_libraries(${test} _CuraEngine cppunit)
            add_test(${test} ${test})
        endforeach()
    endif ()
    foreach (test ${engine_TEST_UTILS})
        add_executable(${test} tests/main.cpp tests/utils/${test}.cpp)
        target_link_libraries(${test} _CuraEngine cppunit)
        add_test(${test} ${test})
    endforeach()
endif()

# Installing CuraEngine.
include(GNUInstallDirs)
install(TARGETS CuraEngine DESTINATION ${CMAKE_INSTALL_BINDIR})
include(CPackConfig.cmake)

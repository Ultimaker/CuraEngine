#Copyright (c) 2020 Ultimaker B.V.
#CuraEngine is released under the terms of the AGPLv3 or higher.

cmake_minimum_required(VERSION 3.6.0)

project(CuraEngine)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(ENABLE_ARCUS "Enable support for ARCUS" ON)

if (MSVC)
    option(MSVC_STATIC_RUNTIME "Link the MSVC runtime statically" OFF)
endif()

if(ENABLE_ARCUS)
    message(STATUS "Building with Arcus")
    # We want to have access to protobuf_generate_cpp and other FindProtobuf features.
    # However, if ProtobufConfig is used instead, there is a CMake option that controls
    # this, which defaults to OFF. We need to force this option to ON instead.
    set(protobuf_MODULE_COMPATIBLE ON CACHE INTERNAL "" FORCE)
    find_package(Protobuf 3.0.0 REQUIRED)
    find_package(Arcus REQUIRED)
    add_definitions(-DARCUS)
    find_program(PROTOC "protoc")
    if(${PROTOC} STREQUAL "PROTOC-NOTFOUND")
        message(FATAL_ERROR "Protobuf compiler missing")
    endif()
endif()

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
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG_INIT}")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE_INIT}")
endif()

set(CMAKE_CXX_STANDARD 11)

if(APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
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
    if (NOT MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra -O0 -g -fno-omit-frame-pointer")
    endif()
    add_definitions(-DASSERT_INSANE_OUTPUT)
    add_definitions(-DUSE_CPU_TIME)
    add_definitions(-DDEBUG)
endif()

if (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /permissive- /Zc:twoPhase- /EHsc /W3")
    if (MSVC_STATIC_RUNTIME)
        foreach(flag_var
            CMAKE_CXX_FLAGS
            CMAKE_CXX_FLAGS_DEBUG
            CMAKE_CXX_FLAGS_RELEASE
            CMAKE_CXX_FLAGS_MINSIZEREL
            CMAKE_CXX_FLAGS_RELWITHDEBINFO
            )
            if(${flag_var} MATCHES "/MD")
                string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
            endif()
        endforeach()
    endif()
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall") # Add warnings
endif()

option (ENABLE_MORE_COMPILER_OPTIMIZATION_FLAGS
    "Enable more optimization flags" ON)
if (ENABLE_MORE_COMPILER_OPTIMIZATION_FLAGS AND NOT (CMAKE_BUILD_TYPE_UPPER MATCHES "DEBUG"))
    message (STATUS "Compile with more optimization flags")
    if (MSVC)
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} /fp:fast")
    else()
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Ofast -funroll-loops")
    endif()
endif ()

if(NOT APPLE AND NOT WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libstdc++")
endif()

if (WIN32)
    add_definitions(-DNOMINMAX)
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
    include_directories(${Polyclipping_INCLUDE_DIRS} "${CMAKE_BINARY_DIR}" ${RAPIDJSON_INCLUDE_DIRS})
else()
    include_directories("${CMAKE_CURRENT_BINARY_DIR}" libs libs/clipper)
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
)

# List of tests. For each test there must be a file tests/${NAME}.cpp.
set(engine_TEST
    GCodeExportTest
    InfillTest
    LayerPlanTest
    MergeInfillLinesTest
    TimeEstimateCalculatorTest
)
set(engine_TEST_INFILL
)
set(engine_TEST_INTEGRATION
    SlicePhaseTest
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
    AABBTest
    AABB3DTest
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
  ENABLE_LANGUAGE(RC)
  if(NOT MSVC)
    SET(CMAKE_RC_COMPILER_INIT windres)
    SET(CMAKE_RC_COMPILE_OBJECT
        "<CMAKE_RC_COMPILER> <FLAGS> -O coff <DEFINES> -i <SOURCE> -o <OBJECT>"
        )
 endif()
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
    include(CTest)

    message(STATUS "Building tests...")
    set(GTEST_USE_STATIC_LIBS true)
    set(GMOCK_ROOT "${CMAKE_CURRENT_BINARY_DIR}/gmock")
    set(GMOCK_VER "1.8.0")
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
    endforeach()
    foreach (test ${engine_TEST_INFILL})
        add_executable(${test} tests/main.cpp tests/infill/${test}.cpp)
        target_link_libraries(${test} _CuraEngine ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES})
        add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
        add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
    endforeach()
    foreach (test ${engine_TEST_INTEGRATION})
        add_executable(${test} tests/main.cpp tests/integration/${test}.cpp)
        target_link_libraries(${test} _CuraEngine ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES})
        add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
        add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
    endforeach()
    foreach (test ${engine_TEST_SETTINGS})
        add_executable(${test} tests/main.cpp tests/settings/${test}.cpp)
        target_link_libraries(${test} _CuraEngine ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES})
        add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
        add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
    endforeach()
    if (ENABLE_ARCUS)
        foreach (test ${engine_TEST_ARCUS})
            add_executable(${test} tests/main.cpp ${engine_TEST_ARCUS_HELPERS} tests/arcus/${test}.cpp)
            target_link_libraries(${test} _CuraEngine ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES})
            add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
            add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
        endforeach()
    endif ()
    foreach (test ${engine_TEST_UTILS})
        add_executable(${test} tests/main.cpp tests/utils/${test}.cpp)
        target_link_libraries(${test} _CuraEngine ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES})
        add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/")
        add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
    endforeach()
endif()

# Installing CuraEngine.
include(GNUInstallDirs)
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

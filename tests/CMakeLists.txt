# Copyright (c) 2022 Ultimaker B.V.
# CuraEngine is released under the terms of the AGPLv3 or higher.

message(STATUS "Building tests...")
include(GoogleTest)
include(CheckCXXCompilerFlag)
include(CMakeDependentOption)

set(TESTS_SRC_BASE
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

set(TESTS_SRC_INTEGRATION
        SlicePhaseTest
        )

set(TESTS_SRC_SETTINGS
        SettingsTest
        )

set(TESTS_SRC_UTILS
        AABBTest
        AABB3DTest
        IntPointTest
        LinearAlg2DTest
        MinimumSpanningTreeTest
        PolygonConnectorTest
        PolygonTest
        PolygonUtilsTest
        SimplifyTest
        SparseGridTest
        StringTest
        UnionFindTest
        )

set(TESTS_HELPERS_SRC ReadTestPolygons.cpp)

set(TESTS_SRC_ARCUS)
if (ENABLE_ARCUS)
    list(APPEND TESTS_SRC_ARCUS
            ArcusCommunicationTest
            ArcusCommunicationPrivateTest)
    list(APPEND TESTS_HELPERS_SRC
        arcus/MockSocket.cpp
        arcus/MockSocket.h
        arcus/MockCommunication.h
        )
endif ()

add_library(test_helpers ${TESTS_HELPERS_SRC})
target_compile_definitions(test_helpers PUBLIC $<$<BOOL:${BUILD_TESTING}>:BUILD_TESTS> $<$<BOOL:${ENABLE_ARCUS}>:ARCUS>)
target_include_directories(test_helpers PUBLIC "../include")
target_link_libraries(test_helpers PRIVATE _CuraEngine GTest::gtest GTest::gmock clipper::clipper)
if (ENABLE_ARCUS)
    target_link_libraries(test_helpers PUBLIC arcus::arcus protobuf::libprotobuf)
endif ()

foreach (test ${TESTS_SRC_BASE})
    add_executable(${test} main.cpp ${test}.cpp)
    add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(${test} PRIVATE _CuraEngine test_helpers GTest::gtest GTest::gmock clipper::clipper)
endforeach ()

foreach (test ${TESTS_SRC_ARCUS})
    add_executable(${test} main.cpp arcus/${test}.cpp)
    add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    target_include_directories(${test} PUBLIC "${CMAKE_BINARY_DIR}")
    target_link_libraries(${test} PRIVATE _CuraEngine test_helpers GTest::gtest GTest::gmock clipper::clipper)
endforeach ()

foreach (test ${TESTS_SRC_INTEGRATION})
    add_executable(${test} main.cpp integration/${test}.cpp)
    add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(${test} PRIVATE _CuraEngine test_helpers GTest::gtest GTest::gmock clipper::clipper)
endforeach ()

foreach (test ${TESTS_SRC_SETTINGS})
    add_executable(${test} main.cpp settings/${test}.cpp)
    add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(${test} PRIVATE _CuraEngine test_helpers GTest::gtest GTest::gmock clipper::clipper)
endforeach ()

foreach (test ${TESTS_SRC_UTILS})
    add_executable(${test} main.cpp utils/${test}.cpp)
    add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(${test} PRIVATE _CuraEngine test_helpers GTest::gtest GTest::gmock clipper::clipper)
endforeach ()


# Ensure that basic sanitizer flags are supported before adding fuzzers;
# - Address sanitizer is often used in conjunction with fuzzing as it will detect
#   common high severity bugs. This sanitizer is used as a "default" for fuzzing
#   when the sanitizer isn't otherwise specified.
# - Fuzzer sanitizer will link against libfuzzer and is currently only supported
#   on clang/msvc and isn't supported with GCC. If you need to use these fuzzers
#   with a GCC based project you should consider looking into the LIB_FUZZING_ENGINE
#   env variable defined in `test/fuzz/CMakeLists.txt`.
set(CMAKE_REQUIRED_LINK_OPTIONS "-fsanitize=fuzzer")
set(CMAKE_REQUIRED_FLAGS "-fsanitize=fuzzer-no-link")
check_cxx_source_compiles([[
#include <cstdint>
extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, std::size_t Size) {
  return 0;
}
]] HAS_FUZZ_FLAGS)

cmake_dependent_option(
    WITH_TEST_FUZZ "Build fuzz tests" ON
    HAS_FUZZ_FLAGS OFF
)

if (WITH_TEST_FUZZ)
    message(STATUS "Building fuzz tests enabled")
    add_subdirectory(fuzz)
else ()
    message(STATUS "Building fuzz tests disabled")
endif ()
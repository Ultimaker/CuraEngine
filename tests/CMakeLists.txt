# Copyright (c) 2022 Ultimaker B.V.
# CuraEngine is released under the terms of the AGPLv3 or higher.

message(STATUS "Building tests...")
include(GoogleTest)

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
        SmoothTest
        SparseGridTest
        StringTest
        UnionFindTest
        )

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

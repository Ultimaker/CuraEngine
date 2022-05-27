# Copyright (c) 2022 Ultimaker B.V.
# CuraEngine is released under the terms of the AGPLv3 or higher.

message(STATUS "Building tests...")
find_package(GTest 1.10.1 CONFIG REQUIRED)
include(GoogleTest)

target_link_libraries(_CuraEngine PUBLIC GTest::gtest GTest::gmock)

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

set(TESTS_HELPERS_SRC ReadTestPolygons.cpp)

if(ENABLE_ARCUS)
    list(APPEND TESTS_SRC
            arcus/ArcusCommunicationTest.cpp
            arcus/ArcusCommunicationPrivateTest.cpp)
    list(APPEND TESTS_HELPERS_SRC arcus/MockSocket.cpp)
endif()
add_library(test_helpers ${TESTS_HELPERS_SRC})
target_link_libraries(test_helpers PRIVATE _CuraEngine GTest::gtest GTest::gmock clipper::clipper)
if(ENABLE_ARCUS)
    target_link_libraries(test_helpers PRIVATE arcus::libarcus protobuf::libprotobuf)
endif()

foreach(test ${TESTS_SRC_BASE})
    add_executable(${test} main.cpp ${test}.cpp)
    add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(${test} PRIVATE _CuraEngine test_helpers GTest::gtest GTest::gmock clipper::clipper)
    if(ENABLE_ARCUS)
        target_link_libraries(${test} PRIVATE arcus::libarcus protobuf::libprotobuf)
    endif()
    gtest_discover_tests(${test} WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()

foreach(test ${TESTS_SRC_INTEGRATION})
    add_executable(${test} main.cpp integration/${test}.cpp)
    add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(${test} PRIVATE _CuraEngine test_helpers GTest::gtest GTest::gmock clipper::clipper)
    if(ENABLE_ARCUS)
        target_link_libraries(${test} PRIVATE arcus::libarcus protobuf::libprotobuf)
    endif()
    gtest_discover_tests(${test} WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()

foreach(test ${TESTS_SRC_SETTINGS})
    add_executable(${test} main.cpp settings/${test}.cpp)
    add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(${test} PRIVATE _CuraEngine test_helpers GTest::gtest GTest::gmock clipper::clipper)
    if(ENABLE_ARCUS)
        target_link_libraries(${test} PRIVATE arcus::libarcus protobuf::libprotobuf)
    endif()
    gtest_discover_tests(${test} WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()

foreach(test ${TESTS_SRC_UTILS})
    add_executable(${test} main.cpp utils/${test}.cpp)
    add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(${test} PRIVATE _CuraEngine test_helpers GTest::gtest GTest::gmock clipper::clipper)
    if(ENABLE_ARCUS)
        target_link_libraries(${test} PRIVATE arcus::libarcus protobuf::libprotobuf)
    endif()
    gtest_discover_tests(${test} WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()

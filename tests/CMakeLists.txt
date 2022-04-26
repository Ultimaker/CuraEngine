# Copyright (c) 2022 Ultimaker B.V.
# CuraEngine is released under the terms of the AGPLv3 or higher.

message(STATUS "Building tests...")
find_package(GTest 1.10.1 CONFIG REQUIRED)
include(GoogleTest)

target_link_libraries(_CuraEngine PUBLIC GTest::gtest GTest::gmock)

set(TESTS_SRC_BASE
        ClipperTest.cpp
        ExtruderPlanTest.cpp
        GCodeExportTest.cpp
        InfillTest.cpp
        LayerPlanTest.cpp
        PathOrderOptimizerTest.cpp
        PathOrderMonotonicTest.cpp
        TimeEstimateCalculatorTest.cpp
        WallsComputationTest.cpp
        )

set(TESTS_SRC_INTEGRATION
        integration/SlicePhaseTest.cpp
        )

set(TESTS_SRC_SETTINGS
        settings/SettingsTest.cpp
        )

set(TESTS_SRC_UTILS
        utils/AABBTest.cpp
        utils/AABB3DTest.cpp
        utils/ExtrusionLineTest.cpp
        utils/IntPointTest.cpp
        utils/LinearAlg2DTest.cpp
        utils/MinimumSpanningTreeTest.cpp
        utils/PolygonConnectorTest.cpp
        utils/PolygonTest.cpp
        utils/PolygonUtilsTest.cpp
        utils/SparseGridTest.cpp
        utils/StringTest.cpp
        utils/UnionFindTest.cpp
        )

set(TESTS_HELPERS_SRC ReadTestPolygons.cpp)

if(ENABLE_ARCUS)
    list(APPEND TESTS_SRC
            arcus/ArcusCommunicationTest.cpp
            arcus/ArcusCommunicationPrivateTest.cpp)
    list(APPEND TESTS_HELPERS_SRC arcus/MockSocket.cpp)
endif()

add_executable(base_tests main.cpp ${TESTS_SRC_BASE} ${TESTS_HELPERS_SRC})
target_link_libraries(base_tests PRIVATE _CuraEngine GTest::gtest GTest::gmock clipper::clipper)
if(ENABLE_ARCUS)
    target_link_libraries(base_tests PRIVATE arcus::libarcus protobuf::libprotobuf)
endif()
gtest_discover_tests(base_tests
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )

add_executable(integration_tests main.cpp ${TESTS_SRC_INTEGRATION} ${TESTS_HELPERS_SRC})
target_link_libraries(integration_tests PRIVATE _CuraEngine GTest::gtest GTest::gmock clipper::clipper)
if(ENABLE_ARCUS)
    target_link_libraries(integration_tests PRIVATE arcus::libarcus protobuf::libprotobuf)
endif()
gtest_discover_tests(integration_tests
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )

add_executable(settings_tests main.cpp ${TESTS_SRC_SETTINGS} ${TESTS_HELPERS_SRC})
target_link_libraries(settings_tests PRIVATE _CuraEngine GTest::gtest GTest::gmock clipper::clipper)
if(ENABLE_ARCUS)
    target_link_libraries(settings_tests PRIVATE arcus::libarcus protobuf::libprotobuf)
endif()
gtest_discover_tests(settings_tests
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )

add_executable(util_tests main.cpp ${TESTS_SRC_UTILS} ${TESTS_HELPERS_SRC})
target_link_libraries(util_tests PRIVATE _CuraEngine GTest::gtest GTest::gmock clipper::clipper)
if(ENABLE_ARCUS)
    target_link_libraries(util_tests PRIVATE arcus::libarcus protobuf::libprotobuf)
endif()
gtest_discover_tests(util_tests
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )
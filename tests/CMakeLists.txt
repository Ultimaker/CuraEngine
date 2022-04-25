# Copyright (c) 2022 Ultimaker B.V.
# CuraEngine is released under the terms of the AGPLv3 or higher.

message(STATUS "Building tests...")
find_package(GTest 1.10.1 CONFIG REQUIRED)

enable_testing()

target_link_libraries(_CuraEngine PUBLIC GTest::gtest GTest::gmock)

set(TESTS_SRC
        ClipperTest.cpp
        ExtruderPlanTest.cpp
        GCodeExportTest.cpp
        InfillTest.cpp
        LayerPlanTest.cpp
        PathOrderOptimizerTest.cpp
        PathOrderMonotonicTest.cpp
        TimeEstimateCalculatorTest.cpp
        WallsComputationTest.cpp
        integration/SlicePhaseTest.cpp
        settings/SettingsTest.cpp
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

add_executable(engine_tests main.cpp ${TESTS_SRC} ${TESTS_HELPERS_SRC})
target_link_libraries(engine_tests PRIVATE _CuraEngine GTest::gtest GTest::gmock clipper::clipper)
if(ENABLE_ARCUS)
    target_link_libraries(engine_tests PRIVATE arcus::libarcus protobuf::libprotobuf)
endif()

include(GoogleTest)
gtest_discover_tests(engine_tests)
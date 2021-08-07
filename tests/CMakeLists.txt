include(CTest)
message(STATUS "Building tests...")
find_package(GTest CONFIG REQUIRED)

# List of tests. For each test there must be a file tests/${NAME}.cpp.
set(engine_TEST
        GCodeExportTest
        InfillTest
        LayerPlanTest
        MergeInfillLinesTest
        PathOrderMonotonicTest
        TimeEstimateCalculatorTest
        )
set(engine_TEST_INTEGRATION
        SlicePhaseTest
        )
set(engine_TEST_SETTINGS
        SettingsTest
        )
set(engine_TEST_ARCUS
        ArcusCommunicationTest
        ArcusCommunicationPrivateTest
        )
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
        arcus/MockSocket.cpp
        )
set(engine_TEST_HELPERS
        ReadTestPolygons.cpp
        )

target_link_libraries(_CuraEngine PUBLIC GTest::gtest GTest::gmock)

#To make sure that the tests are built before running them, add the building of these tests as an additional test.
add_custom_target(build_all_tests)
add_test(BuildTests "${CMAKE_COMMAND}" --build "${CMAKE_CURRENT_BINARY_DIR}" --target build_all_tests)

foreach (test ${engine_TEST})
    add_executable(${test} main.cpp ${engine_TEST_HELPERS} ${test}.cpp)
    target_link_libraries(${test} PRIVATE _CuraEngine GTest::gtest GTest::gmock polyclipping::polyclipping)
    add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/tests")
    add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
    message(STATUS "Added test: ${test}")
endforeach()
foreach (test ${engine_TEST_INFILL})
    add_executable(${test} main.cpp infill/${test}.cpp)
    target_link_libraries(${test} PRIVATE _CuraEngine GTest::gtest GTest::gmock)
    add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/tests/")
    add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
    message(STATUS "Added test: ${test}")
endforeach()
foreach (test ${engine_TEST_INTEGRATION})
    add_executable(${test} main.cpp integration/${test}.cpp)
    target_link_libraries(${test} PRIVATE _CuraEngine GTest::gtest GTest::gmock)
    add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/tests/")
    add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
    message(STATUS "Added test: ${test}")
endforeach()
foreach (test ${engine_TEST_SETTINGS})
    add_executable(${test} main.cpp settings/${test}.cpp)
    target_link_libraries(${test} PRIVATE _CuraEngine GTest::gtest GTest::gmock polyclipping::polyclipping)
    add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/tests/")
    add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
    message(STATUS "Added test: ${test}")
endforeach()
if (ENABLE_ARCUS)
    foreach (test ${engine_TEST_ARCUS})
        add_executable(${test} main.cpp ${engine_TEST_ARCUS_HELPERS} arcus/${test}.cpp)
        target_link_libraries(${test} PRIVATE _CuraEngine GTest::gtest GTest::gmock Arcus::Arcus)
        add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/tests/")
        add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
        message(STATUS "Added test: ${test}")
    endforeach()
endif ()
foreach (test ${engine_TEST_UTILS})
    add_executable(${test} main.cpp utils/${test}.cpp)
    target_link_libraries(${test} PUBLIC _CuraEngine GTest::gtest GTest::gmock polyclipping::polyclipping)
    add_test(NAME ${test} COMMAND "${test}" WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/tests/")
    add_dependencies(build_all_tests ${test}) #Make sure that this gets built as part of the build_all_tests target.
    message(STATUS "Added test: ${test}")
endforeach()
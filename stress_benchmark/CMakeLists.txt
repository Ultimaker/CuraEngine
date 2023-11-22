# Copyright (c) 2023 Ultimaker B.V.
# CuraEngine is released under the terms of the AGPLv3 or higher.

message(STATUS "Building stress benchmarks...")


add_executable(stress_bm_voronoi stress_bm_voronoi.cpp)
target_link_libraries(stress_bm_voronoi PRIVATE _CuraEngine test_helpers range-v3::range-v3 fmt::fmt spdlog::spdlog boost::boost)
target_include_directories(stress_bm_voronoi PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/generated)
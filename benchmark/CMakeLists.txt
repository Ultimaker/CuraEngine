# Copyright (c) 2022 Ultimaker B.V.
# CuraEngine is released under the terms of the AGPLv3 or higher.

message(STATUS "Building benchmarks...")

find_package(benchmark REQUIRED)


add_executable(benchmarks main.cpp)
target_link_libraries(benchmarks PRIVATE _CuraEngine benchmark::benchmark test_helpers)
target_include_directories(benchmarks PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/generated)
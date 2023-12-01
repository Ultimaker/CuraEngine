# Copyright (c) 2023 Ultimaker B.V.
# CuraEngine is released under the terms of the AGPLv3 or higher.

message(STATUS "Building stress benchmarks...")

find_package(docopt REQUIRED)

add_executable(stress_benchmark stress_benchmark.cpp)
target_link_libraries(stress_benchmark PRIVATE _CuraEngine test_helpers spdlog::spdlog boost::boost rapidjson docopt_s)
target_include_directories(stress_benchmark PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/generated)
# Copyright (c) 2022 Ultimaker B.V.
# CuraEngine is released under the terms of the AGPLv3 or higher.

cmake_policy(SET CMP0091 NEW)  # For MSVC flags, will be ignored on non-Windows OS's
cmake_minimum_required(VERSION 3.20)
project(PackageTest)

message(STATUS "Building package-test...")

find_package(curaengine REQUIRED)

add_executable(PackageTest package_test.cpp)
target_include_directories(PackageTest PUBLIC "../include")
target_link_libraries(PackageTest PRIVATE _CuraEngine clipper::clipper)

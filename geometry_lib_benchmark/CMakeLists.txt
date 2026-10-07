cmake_minimum_required(VERSION 3.20)
project(geometry_lib_benchmark CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

include(FetchContent)

# Clipper 6.4.2: sourceforge.net is not reachable from the sandbox, so the pristine upstream
# tarball is taken from the Ubuntu archive (same files as clipper_ver6.4.2.zip).
FetchContent_Declare(clipper1
    URL http://archive.ubuntu.com/ubuntu/pool/universe/libp/libpolyclipping/libpolyclipping_6.4.2.orig.tar.bz2
    URL_HASH SHA256=69a2fadc3c0b6c06c3f77e92067b65e28e398a75d1260e9114c283a04e76c463
    SOURCE_SUBDIR do_not_add_subdirectory)

# Clipper2: latest release at the time of writing
FetchContent_Declare(clipper2
    URL https://github.com/AngusJohnson/Clipper2/archive/refs/tags/Clipper2_2.0.1.tar.gz
    SOURCE_SUBDIR do_not_add_subdirectory)

FetchContent_MakeAvailable(clipper1 clipper2)

# Boost.Geometry is header-only (apt install libboost-dev)
find_package(Boost REQUIRED)

set(C2 ${clipper2_SOURCE_DIR}/CPP/Clipper2Lib)
add_executable(geobench
    main.cpp
    ${clipper1_SOURCE_DIR}/cpp/clipper.cpp
    ${C2}/src/clipper.engine.cpp
    ${C2}/src/clipper.offset.cpp
    ${C2}/src/clipper.rectclip.cpp)
target_include_directories(geobench PRIVATE ${clipper1_SOURCE_DIR}/cpp ${C2}/include)
target_link_libraries(geobench PRIVATE Boost::headers)
target_compile_definitions(geobench PRIVATE DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")

project(CuraEngine)
cmake_minimum_required(VERSION 3.4.3)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# CMake useful variables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

set(VERSION_MAJOR "15")
set(VERSION_MINOR "04")
set(VERSION_PATCH "6")
set(CURA_ENGINE_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

include(Common)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_DEBUG_INIT})
else()
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_RELEASE_INIT})
endif()

if(APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")
endif()

if(NOT DEFINED LIB_SUFFIX)
    set(LIB_SUFFIX "")
endif()
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}")

# Add a compiler flag to check the output for insane values if we are in debug mode.
if(CMAKE_BUILD_TYPE MATCHES DEBUG OR CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    message(STATUS "Building debug release of CuraEngine.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra -O0 -g -fno-omit-frame-pointer")
    add_definitions(-DASSERT_INSANE_OUTPUT)
    add_definitions(-DUSE_CPU_TIME)
    add_definitions(-DDEBUG)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall") # Add warnings

option (ENABLE_MORE_COMPILER_OPTIMIZATION_FLAGS
    "Enable more optimization flags" ON)
if (ENABLE_MORE_COMPILER_OPTIMIZATION_FLAGS)
    message (STATUS "Compile with more optimization flags")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Ofast -funroll-loops")
endif ()

if(NOT APPLE AND NOT WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libstdc++")
endif()

option (ENABLE_OPENMP
    "Use OpenMP for parallel code" ON)

if (ENABLE_OPENMP)
    FIND_PACKAGE( OpenMP )
    if( OPENMP_FOUND )
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}" )
    endif()
endif()

option (ENABLE_G3LOG "Enable G3log for logging" OFF)
if (ENABLE_G3LOG)
    message (STATUS "Compile with G3log for logging")
    add_definitions(-DUSE_G3LOG)
    # G3log for logging
    find_package(G3LOG REQUIRED)
    include_directories(${G3LOG_INCLUDE_DIRS})
endif()

# Mac needed variables (adapt for your needs - http://www.cmake.org/Wiki/CMake_RPATH_handling#Mac_OS_X_and_the_RPATH)
set(CMAKE_MACOSX_RPATH ON)
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


###########################################################################
# build clipper library
###########################################################################
include_directories(${CMAKE_CURRENT_BINARY_DIR} libs libs/clipper)

add_library(clipper STATIC libs/clipper/clipper.cpp)


###########################################################################
# build main program
###########################################################################

set(APPLICATION_NAME CuraEngine)

set(engine_SRCS # Except main.cpp.
    src/layerPart.cpp
    src/skirt.cpp
    src/skin.cpp
    src/inset.cpp
    src/slicer.cpp
    src/utils/logoutput.cpp
    src/utils/socket.cpp
    src/utils/gettime.cpp
    src/polygonHelper.cpp
    src/pathOrderOptimizer.cpp
    src/comb.cpp
    src/settings.cpp
    src/raft.cpp
    src/modelFile/modelFile.cpp
    src/gcodeExport.cpp
    src/support.cpp
    src/bridge.cpp
    src/infill.cpp
    src/polygonOptimizer.cpp
    src/timeEstimate.cpp
    src/optimizedModel.cpp
    )

# Compiling CuraEngine itself.
add_library(_CuraEngine ${engine_SRCS})

# Link to clipper static library.
target_link_libraries(_CuraEngine clipper)

if (UNIX)
    target_link_libraries(_CuraEngine pthread)
endif()

if (ENABLE_G3LOG)
    target_link_libraries(_CuraEngine ${G3LOG_LIBRARIES})
endif()

add_executable(CuraEngine src/main.cpp) # Then compile main.cpp as separate executable, and link the library to it.

target_link_libraries(CuraEngine _CuraEngine)
set_target_properties(CuraEngine PROPERTIES COMPILE_DEFINITIONS "VERSION=\"${CURA_ENGINE_VERSION}\"")

add_custom_command(TARGET CuraEngine POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                       ${CMAKE_SOURCE_DIR}/resources $<TARGET_FILE_DIR:CuraEngine>)

# Installing CuraEngine.
include(GNUInstallDirs)
install(TARGETS CuraEngine DESTINATION ${CMAKE_INSTALL_BINDIR})
include(CPackConfig.cmake)

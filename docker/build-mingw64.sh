#!/bin/sh
#
# This scripts uses CMake to build CuraEngine with static libraries using MinGW
# W64 targeting Windows x64. It also creates a debian package with cpack. The
# contents of the package are installed under "/usr/x86_64-w64-mingw32".
#

# Need protoc here. Add binaries to PATH in the cura development environment.
CURA_DEV_ENV_ROOT=/opt/cura-dev
export PATH="${CURA_DEV_ENV_ROOT}/bin:${PATH}"

TOOLCHAIN_PREFIX=x86_64-w64-mingw32

# NOTES:
#
# 1. CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES
#
# It is needed to make it work. Without it, CMake will generate include search
# paths with /usr/x86_64-w64-mingw32, but for some reason, it won't search
# /usr/x86_64-w64-mingw32/include and generates an error "stdlib.h no such file
# or directory". The difference I saw with and without
# CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES is that, without
# CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES, it will add "-isystem
# /usr/x86_64-w64-mingw32/include".
#
# 2. Linking against Ws2_32 CMake automatically generates link flags with
# "-lWs2_32", which is supposed to link against "libWs2_32.a". On Linux, this is
# case-senstive, and MinGW w64 only installs a "libws2_32.a", so it ends up with
# an error. The symlink commands make sure that "libWs2_32.a" presents.
#

# Create a symlink "libWs2_32.a" pointing to "libws2_32.a"
_old_pwd="$(pwd)"
cd /usr/${TOOLCHAIN_PREFIX}/lib
ln -s libws2_32.a libWs2_32.a
cd "${_old_pwd}"

mkdir build
cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_SYSTEM_NAME=Windows \
    -DCMAKE_FIND_ROOT_PATH=/usr/${TOOLCHAIN_PREFIX} \
    -DCMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES=/usr/${TOOLCHAIN_PREFIX}/include \
    -DCMAKE_PREFIX_PATH=/usr/${TOOLCHAIN_PREFIX} \
    -DCMAKE_INSTALL_PREFIX=/usr/${TOOLCHAIN_PREFIX} \
    -DCMAKE_C_COMPILER=${TOOLCHAIN_PREFIX}-gcc \
    -DCMAKE_CXX_COMPILER=${TOOLCHAIN_PREFIX}-g++ \
    -DCMAKE_RC_COMPILER=${TOOLCHAIN_PREFIX}-windres \
    -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER \
    -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=ONLY \
    -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ONLY \
    -DBUILD_TESTS=OFF \
    -DCMAKE_VERBOSE_MAKEFILE=ON \
    ..
make
# Create DEB
cpack \
    --config ../cmake/cpack_config_deb_mingw64.cmake \
    -D CPACK_INSTALL_CMAKE_PROJECTS="$(pwd);CuraEngine;ALL;/"
# Create ZIP
cpack \
    --config ../cmake/cpack_config_zip_mingw64.cmake \
    -D CPACK_INSTALL_CMAKE_PROJECTS="$(pwd);CuraEngine;ALL;/"

#!/usr/bin/env bash

# Abort at the first error.
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PROJECT_DIR="$( cd "${SCRIPT_DIR}/.." && pwd )"

cd "${PROJECT_DIR}"

python -m pip install conan
conan profile new default --detect
conan config install https://github.com/ultimaker/conan-config.git
conan install . -if build -pr:b cura_build.jinja -pr:h cura_release.jinja --build=missing

cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake \
    -DBUILD_TESTS=OFF \
    ..
make -j $(nproc)
#ctest --output-on-failure -T Test FIXME: use proper Conan targets for the GTest

# TODO upload to conan server so build artifacts only need to be build once. But I would rather not do this to my private server
# We need to inject the cura user and password in this script then
# conan upload "*" -r ultimaker --all --check
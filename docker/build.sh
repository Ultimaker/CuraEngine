#!/usr/bin/env bash

# Abort at the first error.
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PROJECT_DIR="$( cd "${SCRIPT_DIR}/.." && pwd )"

# Make sure that environment variables are set properly
export PATH="${CURA_BUILD_ENV_PATH}/bin:${PATH}"
export PKG_CONFIG_PATH="${CURA_BUILD_ENV_PATH}/lib/pkgconfig:${PKG_CONFIG_PATH}"
export LD_LIBRARY_PATH="${CURA_BUILD_ENV_PATH}/lib:${LD_LIBRARY_PATH}"

if [ "${TESTS}" = "" ]; then
    TESTS="ON"
fi

cd "${PROJECT_DIR}"

mkdir build
cd build
cmake \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_PREFIX_PATH="${CURA_BUILD_ENV_PATH}" \
    -DBUILD_TESTS=${TESTS} \
    ..
make

if [ "${TESTS}" = "ON" ]; then
    ctest --output-on-failure -T Test
fi

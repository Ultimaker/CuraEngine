#!/bin/bash
set euxo pipefail

conan install . --build=missing --update -s build_type=Release -o curaengine:enable_testing=True
cmake --preset release -DWITH_TEST_FUZZ=ON
cmake --build --preset release -j$(nproc)

find . -name FuzzGcodeExport
#cp build/Release/tests/fuzz/Fuzz* $OUT
#!/bin/bash
set euxo pipefail

conan install . --build=missing --update -s build_type=Release -o curaengine:enable_testing=True
cmake --preset release -DWITH_TEST_FUZZ=ON
cmake --build --preset release -j$(nproc)

cp build/Release/tests/fuzz/Fuzz* $OUT

mkdir -p $OUT/lib
# Move all dynamic deps into output directory.
find ~/.conan/data -name '*.so*' -exec cp {} $OUT/lib/ \;

# Rewrite dynamic linker paths to point to output directory
for fuzzer in $OUT/Fuzz*; do
    chrpath -r '$ORIGIN/lib' $fuzzer
done

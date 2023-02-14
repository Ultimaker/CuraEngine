#!/usr/bin/env bash
root_dir=$(dirname $(realpath -- "$BASH_SOURCE"))

cd $root_dir

if [ $1 = debug ]; then
    rm -r build | echo "no build dir to remove, installing..."
    echo "building debug version..."
    conan install . --build=missing --update -s build_type=Debug
    cmake --preset debug
    cmake --build --preset debug
    source build/generators/conanrun.sh
elif [ $1 = release ]; then
    rm -r build | echo "no build dir to remove, installing..."
    echo "building release version..."
    conan install . --build=missing --update
    conan build .
else
    echo "invalid option. options are 'debug' or 'release'"
fi
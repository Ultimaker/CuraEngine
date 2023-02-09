#!/usr/bin/env bash
root_dir=$(dirname $(realpath -- "$BASH_SOURCE"))

cd $root_dir

echo $(pwd)
rm -r build | echo "no build dir to remove, installing..."
conan install . --build=missing --update
conan build .
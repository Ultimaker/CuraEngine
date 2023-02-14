#!/usr/bin/env bash
root_dir=$(dirname $(realpath -- "$BASH_SOURCE"))

cd $root_dir

if [ -f "./build/Debug/CuraEngine" ]; then
    ./build/Debug/CuraEngine connect 127.0.0.1:49674
elif [ -f "./build/Release/CuraEngine" ]; then
    ./build/Release/CuraEngine connect 127.0.0.1:49674
else
    echo "you must build using buildEngine.sh first"
fi
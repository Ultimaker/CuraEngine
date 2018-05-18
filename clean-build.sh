clear

echo "Deleting build/ folder"
rm -rf build/
echo "Creating build/ folder"
mkdir build
cd build
echo "Entering build/ folder"

if [ "$1" == "debug" ];
    then
        cmake -DCMAKE_BUILD_TYPE=Debug ..
        make -j 8
else
        cmake -DCMAKE_BUILD_TYPE=Release ..
        make VERBOSE=1 -j 8
fi

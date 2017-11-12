#!/bin/sh

if which cmake > /dev/null; then
    echo "cmake found"
else
    echo "cmake not found. Make sure it's in your PATH."
    exit 1
fi

cd ..
if [ -d ./build ]; then
    echo "build directory exists and will be reused"
else
    mkdir ./build
    echo "creating build directory"
fi

echo "Entering build directory"
cd ./build

echo $@

echo "generate cmake config"
cmake $@ -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=RelWithDebInfo ..

echo "build ..."
cmake --build . --target install --config RelWithDebInfo -- -j4

cd ../scripts

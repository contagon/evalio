#!/bin/bash

if test -d deps/Sophus; then
    exit 0
fi

mkdir -p deps
cd deps

git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.24.6
git apply ../../sophus.patch

mkdir build
cd build
cmake -G Ninja .. \
    -DBUILD_SOPHUS_TESTS=OFF \
ninja
ninja install
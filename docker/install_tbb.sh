#!/bin/bash

if test -d deps/oneTBB; then
    exit 0
fi

mkdir -p deps
cd deps

git clone https://github.com/oneapi-src/oneTBB
cd oneTBB
git checkout 2021.10.0

mkdir build
cd build
cmake -G Ninja .. -DTBB_TEST=OFF
ninja
ninja install
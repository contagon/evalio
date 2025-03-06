#!/bin/bash

if test -d deps/robin-map; then
    exit 0
fi

mkdir -p deps
cd deps

git clone https://github.com/Tessil/robin-map.git
cd robin-map
git checkout v1.3.0

mkdir build
cd build
cmake -G Ninja .. 
ninja
ninja install
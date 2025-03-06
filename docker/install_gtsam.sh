#!/bin/bash

if test -d deps/gtsam; then
    exit 0
fi

mkdir -p deps
cd deps
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.2

mkdir build
cd build
cmake -G Ninja .. \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_WITH_TBB=ON \
    -DGTSAM_USE_SYSTEM_METIS=ON \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_DOCS=FALSE \
    -DGTSAM_BUILD_PYTHON=FALSE \
    -DGTSAM_INSTALL_MATLAB_TOOLBOX=FALSE \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=FALSE \
    -DGTSAM_BUILD_EXAMPLES=FALSE \
ninja
ninja install
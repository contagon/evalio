#!/bin/bash
# call this script from the root of evalio

topdir=$(pwd)
# ----------------- Pipelines ----------------- #
mkdir -p cpp/bindings/pipelines-src
cd cpp/bindings/pipelines-src

# kiss
if [ ! -d "kiss-icp" ]; then
    git clone https://github.com/PRBonn/kiss-icp.git
fi
cd kiss-icp
git stash
git switch --detach v1.2.2
git apply ../../pipelines/kiss_icp.patch
cd ..

# lio-sam
if [ ! -d "LIO-SAM" ]; then
    git clone https://github.com/contagon/LIO-SAM.git
fi
cd LIO-SAM
git stash
git checkout master

# LOAM
if [ ! -d "loam" ]; then
    git clone https://github.com/DanMcGann/loam.git
fi

# ------------------------- Dependencies ------------------------- #
cd $topdir
if [ ! -d ".vcpkg/" ]; then
    git clone https://github.com/microsoft/vcpkg.git .vcpkg/
fi
cd .vcpkg
git switch --detach 2025.03.19
cd ..
./.vcpkg/bootstrap-vcpkg.sh
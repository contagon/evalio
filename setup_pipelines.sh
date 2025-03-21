#!/bin/bash
# call this script from the root of evalio

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
if [ ! -d "kiss-icp" ]; then
    git clone https://github.com/contagon/LIO-SAM.git
fi
cd LIO-SAM
git stash
git checkout master

# ------------------------- Dependencies ------------------------- #
if [ ! -d ".vcpkg/" ]; then
    git clone --depth 1 https://github.com/microsoft/vcpkg.git .vcpkg/
fi
./.vcpkg/bootstrap-vcpkg.sh
./.vcpkg/vcpkg install --x-install-root=.vcpkg_installed/
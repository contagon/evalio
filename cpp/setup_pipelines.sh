#!/bin/bash
# call this script from the root of evalio OR from cpp/

# Detect if we're in cpp/ or root and adjust
if [[ $(basename $(pwd)) == "cpp" ]]; then
    cd ..
fi

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
cd ..

# LOAM
if [ ! -d "loam" ]; then
    git clone https://github.com/DanMcGann/loam.git
fi
cd loam
git stash
git checkout main
git apply ../../pipelines/loam.patch
cd ..

# GenZ-ICP
if [ ! -d "genz-icp" ]; then
    git clone https://github.com/cocel-postech/genz-icp.git
fi
cd genz-icp
git stash
git switch --detach v0.2.0
git apply ../../pipelines/genz_icp.patch
cd ..

# MAD-ICP
if [ ! -d "mad-icp" ]; then
    git clone https://github.com/rvp-group/mad-icp.git
fi
cd mad-icp
git stash
git switch --detach 0.0.10
git apply ../../pipelines/mad_icp.patch
cd ..

if [ ! -d "ct_icp" ]; then
    git clone https://github.com/jedeschaud/ct_icp.git
fi
cd ct_icp
git stash
git switch --detach icra_2022
git apply ../../pipelines/ct_icp.patch
cd ..

# DLIO
if [ ! -d "direct_lidar_inertial_odometry" ]; then
    git clone https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git
fi
cd direct_lidar_inertial_odometry
git stash
git switch --detach v1.1.1
git apply ../../pipelines/dlio.patch
cd ..

# Fast-LIO2
if [ ! -d "FAST_LIO" ]; then
    git clone https://github.com/hku-mars/FAST_LIO.git
fi
cd FAST_LIO
git stash
git checkout main
git apply ../../pipelines/fast_lio2.patch
cd ..

# ------------------------- Dependencies ------------------------- #
cd $topdir
if [ ! -d ".vcpkg/" ]; then
    git clone https://github.com/microsoft/vcpkg.git .vcpkg/
fi
cd .vcpkg
git switch --detach 2025.08.27
cd ..
./.vcpkg/bootstrap-vcpkg.sh
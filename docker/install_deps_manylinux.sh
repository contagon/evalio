#!/bin/bash

# Base dependencies
yum install -y eigen3-devel \
                ninja-build \
                python3.11-pybind11-devel \
# I'm sure the above will change eventually

# Be careful with source tbb install, may conflict with system tbb installed with opencv
# Unfortunately, system tbb is too old for some of the projects
                
# Kiss-ICP
yum install -y eigen3-devel \
./install_tbb.sh
yum install -y fmt-devel # for sophus
./install_sophus.sh
# default robin-map is old.. build from source
./install_robin.sh

# LIO-SAM 
yum install -y opencv-devel \
                pcl-devel \
                tbb-devel \
# for gtsam
yum install -y metis-devel
./install_tbb.sh
./install_gtsam.sh

# clean up afterwards
rm -rf deps
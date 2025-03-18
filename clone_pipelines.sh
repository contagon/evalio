#!/bin/bash
# call this script from the root of evalio

mkdir -p cpp/bindings/pipelines-src
cd cpp/bindings/pipelines-src

# TODO: Add patch
git clone https://github.com/PRBonn/kiss-icp.git
cd kiss-icp
git switch --detach v1.2.2
git apply ../../pipelines/kiss_icp.patch
cd ..

git clone https://github.com/contagon/LIO-SAM.git


#!/bin/bash

archs=(
    "x86_64"
)

glibcs=(
    "2_34"
    "2_28"
)

for arch in "${archs[@]}"; do
    for glibc in "${glibcs[@]}"; do
        echo "Building for arch: $arch, glibc: $glibc"
        
        docker build -f Dockerfile_manylinux -t "ghcr.io/contagon/evalio_manylinux_${glibc}_${arch}" --build-arg ARCH=$arch --build-arg GLIBC=$glibc .
        docker push ghcr.io/contagon/evalio_manylinux_${glibc}_$arch
    done
done


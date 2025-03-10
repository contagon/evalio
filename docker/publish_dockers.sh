#!/bin/bash

archs=(
    "x86_64"
)

versions=(
    "latest"
)

for arch in "${archs[@]}"; do
    for version in "${versions[@]}"; do
        echo "Building for arch: $arch, version: $version"
        # Add your build commands here
        docker build -f Dockerfile_manylinux -t "ghcr.io/contagon/evalio_manylinux_2_28_$arch:$version" --build-arg ARCH=$arch --build-arg VERSION=$version .

        docker push ghcr.io/contagon/evalio_manylinux_2_28_$arch:$version
    done
done


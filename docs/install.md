For the majority of use cases, we simply recommend installing via your favorite python package manager,
```bash
uv add evalio      # uv
pip install evalio # pip
```

## Building from Source
Building all the available pipelines together is difficult, so we've done our best to separate a build option for just the core types of evalio, and then pipelines can be optionally added in.

### Base
We've attempted to make building from source as easy as possible. We generally build through [scikit-core-build](https://scikit-build-core.readthedocs.io/) which provides a simple wrapper for building CMake projects as python packages. `uv` is our frontend of choice for this process, but it is also possible via pip
```bash
uv sync --verbose  # uv version
pip install -e .   # pip version
```

Of course, building via the usual `CMake` way is also possible, with the only default dependency being `Eigen3`,
```bash
mkdir build
cd build
cmake ..
make
```

### Pipelines
By default, pipelines are not included due to their large dependencies. We use vpckg to handle a reliable build of these dependencies and pipelines. vcpkg and the pipelines can be setup via running
```bash
./cpp/setup_pipelines.sh
```

This will clone and setup vcpkg in the `.vcpkg` directory, and the pipelines (including some minor patches) to `cpp/bindings/pipelines-src`. vcpkg will automatically be picked up by CMake in this directory, so the build process then continues as in the base section.

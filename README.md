## Evalio

evalio is a tool for **Eval**uating **L**idar-**I**nertial **O**dometry.

Specifically, it provides a common interface for connecting LIO datasets and LIO pipelines. This allows for easy addition of new datasets and pipelines, as well as a common location to evaluate them.

## Usage

TODO! Fill this out

## Installation

Python installation can be done via `pypi`. Simply run
```bash
pip install evalio
```

If you are looking to add a custom C++ pipeline, the header-only C++ library can be added via `CMake` fetch_content.
```cmake
include(FetchContent)
FetchContent_Declare(
  evalio
  GIT_REPOSITORY https://github.com/contagon/evalio.git
)
FetchContent_MakeAvailable(evalio)
...
target_link_libraries(my_target PRIVATE evalio)
```

## Building

While we recommend simply installing the python package using your preferred python package manager, we've attempted to make building from source as easy as possible.

The only default dependency is `Eigen3`, thus building can simply be done via `CMake`:
```bash
mkdir build
cd build
cmake ..
make
```

By default, all pipelines are not included due to their large dependencies. CMake will look for them in the `cpp/evalio/pipeslines-src` directory. If you'd like to add them, simply run `clone_pipelines.sh` that will clone and patch them appropriately. When these pipelines are included, the number of dependencies increases significantly, so have provided a [docker image](https://github.com/contagon/evalio/pkgs/container/evalio_manylinux_2_28_x86_64) that includes all dependencies for building as well as a VSCode devcontainer configuration. When opening in VSCode, you'll automatically be prompted to open in this container.

To build the python package, the python bindings can be enabled by passing `-DEVALIO_BUILD_PYTHON=ON` to cmake. Alternatively (and what we recommend), the package can be built using a python frontend with `scikit-build-core` wrapping the cmake process. We prefer `uv` as our frontend, which allows the python package to be built via `uv build` and installed in the uv venv using `uv sync`.

## Contributing

### Datasets

### Pipelines
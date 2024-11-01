## Evalio

evalio is a tool for **Eval**uating **L**idar-**I**nertial **O**dometry.

Specifically, it provides a common interface for connecting LIO datasets and LIO pipelines. This allows for easy addition of new datasets and pipelines, as well as a common location to evaluate them.

## Building

While we recommend simply installing the python package using your preferred python package manager, we've attempted to make building from source as easy as possible.

You'll need,
- [vcpkg](https://vcpkg.io/en/) installed and the `VCPKG_ROOT` environment variable set to the root of your vcpkg installation. `vcpkg` handles all of the C++ dependencies.
- [uv](https://github.com/astral-sh/uv) installed and on your `$PATH`

Building the C++ portion can be done as follows,
```bash
cmake -B build -DCMAKE_TOOLCHAIN_FILE="${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
cmake --build build 
```

Alternatively, you can set `CMAKE_TOOLCHAIN_FILE` as an env variable, which I usually do using [.envrc](.envrc) and [direnv](https://github.com/direnv/direnv).

To build the python portion, simply set the `CMAKE_TOOLCHAIN_FILE` environment variable and run 
```bash
uv sync
```
which will build evalio from scratch and install it into the uv virtual env. Evalio can then be run with `uv run evalio <command>`. `uv` will not automatically notice changes and recompile, but will if  you run `touch pyproject.toml`. 

If you'd prefer an editable (aka incremental) build, you can run
```bash
uv run pip install --no-build-isolation --config-settings=editable.rebuild=true -Cbuild-dir=build_pip -ve .
```
to install it in the current uv virtual env. If this is done, before each time you run `evalio`, `cmake --build build_pip` will be ran to compile any changes that may have occurred. See [scikit-build-core](https://scikit-build-core.readthedocs.io/en/latest/configuration.html#editable-installs) for more info.
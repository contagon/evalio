## evalio

evalio is a tool for **Eval**uating **L**idar-**I**nertial **O**dometry.

Specifically, it provides a common interface for connecting LIO datasets and LIO pipelines. This allows for easy addition of new datasets and pipelines, as well as a common location to evaluate them making benchmarks significantly easier to run. It features,
- No ROS dependency! (though it can still load rosbag datasets using the wonderful [rosbags](https://ternaris.gitlab.io/rosbags/) package)
- Easy to add new datasets and pipelines, see the [example](https://github.com/contagon/evalio-example)
- Download and manage datasets via the CLI interface
- Simply to use API for friction-free access to data
- Run pipelines via the CLI interface and yml config files
- Compute statistics for resulting trajectory runs

## Usage

Once installed, datasets can be listed and downloaded via the CLI interface. For example, to list all datasets and then download a sequence from the newer-college 2020 dataset,
```bash
evalio ls datasets
evalio download newer_college_2020/short_experiment
```
evalio downloads data to the `EVALIO_DATA` environment variable, or if unset to the local folder `./evalio_data`. Once downloaded, a trajectory can then be easily used in python,
```python
from evalio.datasets import NewerCollege2020

# for all data
for mm in NewerCollege2020.short_experiment:
    print(mm)

# for lidars
for scan in NewerCollege2020.short_experiment.lidar():
    print(scan)

# for imu
for imu in NewerCollege2020.short_experiment.imu():
    print(imu)

# get a single scan
scan = NewerCollege2020.short_experiment.get_one_lidar(100)
```
We recommend checking out the [base dataset class](python/evalio/datasets/base.py) for more information on how to interact with datasets.

Once downloaded, pipelines can then be run on the dataset. All pipelines and their parameters can be shown via,
```bash
evalio ls pipelines
```
For example, to run KissICP on the dataset,
```bash
evalio run -o results -d newer_college_2020/short_experiment -p kiss
```
This will run the pipeline on the dataset and save the results to the `results` folder. The results can then be used to compute statistics on the trajectory,
```bash
evalio stats results
```
More complex experiments can be run, including varying pipeline parameters, via specifying a config file,
```yaml
output_dir: ./results/

datasets:
  # Run on all of newer college trajectories
  - newer_college_2020/*
  # Run on first 1000 scans of multi campus
  - name: multi_campus/ntu_day_01
    length: 1000

pipelines:
  # Run vanilla kiss with default parameters
  - kiss
  # Tweak kiss parameters
  - name: kiss_tweaked
    deskew: true
    # Sweep over parameters is available as well
    sweep:
      min_motion_th: [0.01, 0.1, 1.0]
      
```
This can then be run via
```bash
evalio run -c config.yml
```
That's about the jist of it! Try playing around the CLI interface to see what else is possible. Feel free to open an issue if you have any questions, suggestions, or problems.

It should also be mentioned, autocomplete can be installed via [argcomplete](https://github.com/kislyuk/argcomplete),
```bash
evalio "$(register-python-argcomplete evalio)"
```
This is extra useful for specifying the datasets when downloading or running, as they can get particularly long.

## Installation

Python packages are available via PyPi, so simply install via pip,
```bash
pip install evalio
```
or for usage in `uv`,
```bash
uv add evalio
```

If you are looking to add a custom C++ pipeline, the header-only C++ library is included in the python package, and can be found via,
```cmake
find_package(evalio REQUIRED)
target_link_libraries(my_target PRIVATE evalio)
```
Alternatively, the library can be pulled via CMake's `FetchContent` module. See our [examples](https://github.com/contagon/evalio-example) for more information.

## Custom Datasets & Pipelines
We understand that using an internal or work-in-progress datasets and pipelines will be a requirement, thus evalio has full support for this. As mentioned above, we recommend checking out our [example](https://github.com/contagon/evalio-example) for more information how to to do this (it's pretty easy!). 

The TL;DR version, a custom dataset can be made via inheriting from the `Dataset` class in python only, and a custom pipeline from inheriting the `Pipeline` class in either C++ or python. These can then be made available to evalio via the `EVALIO_CUSTOM` env variable point to the python module that contains them.

We **highly** recommend making a PR to merge your custom datasets or pipelines into evalio once they are ready. This will make it more likely the community will use and cite your work, as well as increase the usefulness of evalio for everyone.

## Building from Source

While we recommend simply installing the python package using your preferred python package manager (our is `uv`), we've attempted to make building from source as easy as possible. We generally build through [scikit-core-build](https://scikit-build-core.readthedocs.io/) which provides a simple wrapper for building CMake projects as python packages. `uv` is our frontend of choice for this process, but it is also possible via pip
```bash
uv sync          # uv version
pip install -e . # pip version
```

Of course, building via the usual `CMake` way is also possible, with the only default dependency being `Eigen3`,
```bash
mkdir build
cd build
cmake ..
make
```

By default, all pipelines are not included due to their large dependencies. CMake will look for them in the `cpp/bindings/pipeslines-src` directory. If you'd like to add them, simply run the `clone_pipelines.sh` script that will clone and patch them appropriately. 

When these pipelines are included, the number of dependencies increases significantly, so have provided a [docker image](https://github.com/contagon/evalio/pkgs/container/evalio_manylinux_2_28_x86_64) that includes all dependencies for building as well as a VSCode devcontainer configuration. When opening in VSCode, you'll automatically be prompted to open in this container.
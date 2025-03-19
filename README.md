## evalio

evalio is a tool for **Eval**uating **L**idar-**I**nertial **O**dometry.

Specifically, it provides a common interface for connecting LIO datasets and LIO pipelines. This allows for easy addition of new datasets and pipelines, as well as a common location to evaluate them making benchmarks significantly easier to run. It features,
- No ROS dependency! (though it can still load rosbag datasets using the wonderful [rosbags](https://ternaris.gitlab.io/rosbags/) package)
- Easy to add new datasets and pipelines, see the [example](https://github.com/contagon/evalio-example)
- Unified representation of lidar scan, e.g. row (scan-line) major order, stamped at the start of the scan, point stamps are relative from the start of the scan.
- Download and manage datasets via the CLI interface
- Simple to use API for friction-free access to data
- Run pipelines via the CLI interface and yaml config files
- Compute statistics for resulting trajectory runs

## Installation

evalio is available on PyPi, so simply install via your favorite python package manager,
```bash
uv add evalio      # uv
pip install evalio # pip
```

## Usage

evalio can be used both as a python library and as a CLI for both datasets and pipelines.

### Datasets

Once evalio is installed, datasets can be listed and downloaded via the CLI interface. For example, to list all datasets and then download a sequence from the newer-college 2020 dataset,
```bash
evalio ls datasets
evalio download hilti_2022/basement_2
```
evalio downloads data to the `EVALIO_DATA` environment variable, or if unset to the local folder `./evalio_data`. 

> [!NOTE]
> Many datasets use [gdown](https://github.com/wkentaro/gdown) to download datasets from google drive. Unfortunately, this can occasionally be finicky due to google's download limits, [downloading cookies from your browser](https://github.com/wkentaro/gdown?tab=readme-ov-file#i-set-the-permission-anyone-with-link-but-i-still-cant-download) can often help.

Once downloaded, a trajectory can then be easily used in python,
```python
from evalio.datasets import Hilti2022

# for all data
for mm in Hilti2022.basement_2:
    print(mm)

# for lidars
for scan in Hilti2022.basement_2.lidar():
    print(scan)

# for imu
for imu in Hilti2022.basement_2.imu():
    print(imu)
```

For example, you can easily get a single scan to plot a bird-eye view,
```python
import matplotlib.pyplot as plt
import numpy as np
# get the 10th scan
scan = Hilti2022.basement_2.get_one_lidar(10)
# always in row-major order, with stamp at start of scan
x = np.array([p.x for p in scan.points])
y = np.array([p.y for p in scan.points])
z = np.array([p.z for p in scan.points])
plt.scatter(x, y, c=z, s=1)
plt.axis('equal')
plt.show()
```
evalio also comes with a built wrapper for converting to [rerun](rerun.io) types,
```python
import rerun as rr
from evalio.rerun import convert
rr.init("evalio")
rr.connect_tcp()
for scan in Hilti2022.basement_2.lidar():
    rr.set_time_seconds("timeline", seconds=scan.stamp.to_sec())
    rr.log("lidar", convert(scan, color=[255, 0, 255]))
```

> [!NOTE]  
> To run the rerun visualization, rerun must be installed. This can be done by installing `rerun-sdk` or `evalio[vis]` from PyPi.

We recommend checking out the [base dataset class](python/evalio/datasets/base.py) for more information on how to interact with datasets.

### Pipelines

The other half of evalio is the pipelines that can be run on various datasets. All pipelines and their parameters can be shown via,
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
That's about the gist of it! Try playing around the CLI interface to see what else is possible. Feel free to open an issue if you have any questions, suggestions, or problems.

It should also be mentioned, autocomplete can be installed via [argcomplete](https://github.com/kislyuk/argcomplete),
```bash
evalio "$(register-python-argcomplete evalio)"
```
This is extra useful for specifying the datasets when downloading or running, as they can get particularly long.

## Custom Datasets & Pipelines
We understand that using an internal or work-in-progress datasets and pipelines will often be needed, thus evalio has full support for this. As mentioned above, we recommend checking out our [example](https://github.com/contagon/evalio-example) for more information how to to do this (it's pretty easy!). 

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

By default, all pipelines are not included due to their large dependencies. CMake will look for them in the `cpp/bindings/pipelines-src` directory. If you'd like to add them, simply run the `clone_pipelines.sh` script that will clone and patch them appropriately. 

When these pipelines are included, the number of dependencies increases significantly, so have provided a [docker image](https://github.com/contagon/evalio/pkgs/container/evalio_manylinux_2_28_x86_64) that includes all dependencies for building as well as a VSCode devcontainer configuration. When opening in VSCode, you'll automatically be prompted to open in this container.
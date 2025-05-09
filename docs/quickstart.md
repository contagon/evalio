
evalio is available on PyPi, so simply install via your favorite python package manager,
```bash
uv add evalio      # uv
pip install evalio # pip
```

evalio can be used both as a python library and as a CLI for both datasets and pipelines.

## Datasets

Once evalio is installed, datasets can be listed and downloaded via the CLI interface. For example, to list all datasets and then download a sequence from the hilti-2022 dataset,
```bash
evalio ls datasets
evalio download hilti_2022/basement_2
```
evalio downloads data to the path given by `-D`, `EVALIO_DATA` environment variable, or if both are unset to the local folder `./evalio_data`. All the trajectories in a dataset can also be downloaded by using the wildcard `hilti_2022/*`, making sure to escape the asterisk as needed.

!!! tip

    evalio also comes with autocomplete, which makes typing the long dataset and pipeline names much easier. To install, do one of the following,
    ```bash
    eval "$(evalio --show-completion)" # install for the current session
    evalio --install-completion        # install for all future sessions
    ```

!!! note

    Many datasets use [gdown](https://github.com/wkentaro/gdown) to download datasets from google drive. Unfortunately, this can occasionally be finicky due to google's download limits, however [downloading cookies from your browser](https://github.com/wkentaro/gdown?tab=readme-ov-file#i-set-the-permission-anyone-with-link-but-i-still-cant-download) can often help.


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
evalio also comes with a built wrapper for converting to [rerun](https://rerun.io) types,
```python
import rerun as rr
from evalio.rerun import convert

rr.init("evalio")
rr.connect_tcp()
for scan in Hilti2022.basement_2.lidar():
    rr.set_time_seconds("timeline", seconds=scan.stamp.to_sec())
    rr.log("lidar", convert(scan, color=[255, 0, 255]))
```

!!! note
    
    To run the rerun visualization, rerun must be installed. This can be done by installing `rerun-sdk` or `evalio[vis]` from PyPi.

We recommend checking out the [API reference][evalio.datasets.Dataset] for more information on how to interact with datasets, and the [example](examples/dataset.md) for an example of how to create your own dataset.

## Pipelines

The other half of evalio is the pipelines that can be run on various datasets. All pipelines and their parameters can be shown via,
```bash
evalio ls pipelines
```
For example, to run KissICP on a dataset,
```bash
evalio run -o results -d hilti_2022/basement_2 -p kiss
```
This will run the pipeline on the dataset and save the results to the `results` folder. The results can then be used to compute statistics on the trajectory,
```bash
evalio stats results
```
!!! note

    KissICP does poorly by default on hilti_2022/basement_2, due to the close range and large default voxel size. You can visualize this by adding `-s ms` to the `run` command to visualize the map and scan in rerun.

More complex experiments can be run, including varying pipeline parameters, via specifying a config file,
```yaml
output_dir: ./results/

datasets:
  # Run on all of newer college trajectories
  - hilti_2022/*
  # Run on first 1000 scans of multi campus
  - name: multi_campus/ntu_day_01
    length: 1000

pipelines:
  # Run vanilla kiss with default parameters
  - kiss
  # Tweak kiss parameters
  - name: kiss_tweaked
    pipeline: kiss
    deskew: true
    # Some of these datasets need smaller voxel sizes
    sweep:
      voxel_size: [0.1, 0.5, 1.0]
      
```
This can then be run via
```bash
evalio run -c config.yml
```
That's about the gist of it! Try playing around the CLI interface to see what else is possible, such as a number of visualization options using rerun. Feel free to open an issue if you have any questions, suggestions, or problems. 

Additionally, we recommend checking out the usage section to see some more examples for use cases for evalio.
<div align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/assets/logo-wordmark-dark.svg">
    <source media="(prefers-color-scheme: light)" srcset="docs/assets/logo-wordmark.svg">
    <img alt="evalio" src="docs/assets/logo-wordmark.svg">
  </picture>
  <br/>
  <a href="https://github.com/contagon/evalio/releases"><img src="https://img.shields.io/github/v/release/contagon/evalio" /></a>
  <a href="https://github.com/contagon/evalio/actions/workflows/on_main.yml"><img src="https://img.shields.io/github/actions/workflow/status/contagon/evalio/on_main.yml" /></a>
  <a href="https://github.com/contagon/evalio/blob/master/LICENSE"><img src="https://img.shields.io/github/license/rpl-cmu/form" /></a>
  <a href="https://arxiv.org/abs/2507.16000"><img src="https://img.shields.io/badge/arXiv-2507.16000-b31b1b.svg" /></a>
  <p>evalio is a tool for <strong>Eval</strong>uating <strong>L</strong>idar-<strong>I</strong>nertial <strong>O</strong>dometry.</p>
  <br/>
</div>

Specifically, it provides a common interface for connecting LIO datasets and LIO pipelines. This allows for easy addition of new datasets and pipelines, as well as a common location to evaluate them making benchmarks significantly easier to run. It features,
- No ROS dependency! (though it can still load rosbag datasets using the wonderful [rosbags](https://ternaris.gitlab.io/rosbags/) package)
- 8 compiled in LIO pipelines and 12 configured datasets.
- Easy to add new datasets and pipelines, see the [example](https://github.com/contagon/evalio-example)
- Unified representation of lidar scan, e.g. row (scan-line) major order, stamped at the start of the scan, point stamps are relative from the start of the scan.
- Download and manage datasets via the CLI interface
- Simple to use API for friction-free access to data
- Run pipelines via the CLI interface and yaml config files
- Compute statistics for resulting trajectory runs

## Installation

evalio is available on PyPi (with all pipelines compiled in!), so simply install via your favorite python package manager,
```bash
uv add evalio      # uv
pip install evalio # pip
```

## Basic Usage

evalio can be used both as a python library and as a CLI for both datasets and pipelines. We cover just the tip of the iceberg here, so please check out the [docs](https://contagon.github.io/evalio/) for more information.

### Datasets

Once evalio is installed, datasets can be listed and downloaded via the CLI interface. For example, to list all datasets and then download a sequence from the hilti-2022 dataset,
```bash
evalio ls datasets
evalio dl hilti_2022/basement_2
```

Once downloaded, a trajectory can then be easily used in python,
```python
from evalio import datasets as ds

# for all data
for mm in ds.Hilti2022.basement_2:
    print(mm)

# for lidars
for scan in ds.Hilti2022.basement_2.lidar():
    print(scan)

# for imu
for imu in ds.Hilti2022.basement_2.imu():
    print(imu)
```

### Pipelines

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

More complex experiments can be run, including varying pipeline parameters, via specifying a config file,
```yaml
output_dir: ./results/

datasets:
  # Run on all of hilti trajectories
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
    # Sweep over voxel size parameter
    sweep:
      voxel_size: [0.1, 0.5, 1.0]
      
```
This can then be run via
```bash
evalio run -c config.yml
```

## Contributing

Contributions are always welcome! Feel free to open an issue, pull request, etc. The documentation has a more details on developing new datasets and pipelines.

## Citation

If you use evalio in your research, please cite the following paper,
```bibtex
@misc{potokar2025_evaluation_lidar_odometry,
      title={A Comprehensive Evaluation of LiDAR Odometry Techniques}, 
      author={Easton Potokar and Michael Kaess},
      year={2025},
      eprint={2507.16000},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2507.16000}, 
}
```

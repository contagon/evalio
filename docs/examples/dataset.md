While evalio comes with a number of built-in datasets, you can also easily create your own dataset without having to build any of evalio from source. In addition to this guide, we also provide [evalio-example](https://github.com/contagon/evalio-example) with examples.

One simply has to inherit from the `Dataset` class,

```python
from evalio.datasets import Dataset, DatasetIterator
from evalio.types import Trajectory, SE3, ImuParams, LidarParams

from typing import Sequence, Optional
from pathlib import Path
from enum import auto

class MyDataset(Dataset):
    sequence_1 = auto()
    sequence_2 = auto()
    sequence_3 = auto()

    def data_iter(self) -> DatasetIterator: ...

    def ground_truth_raw(self) -> Trajectory: ...

    def files(self) -> Sequence[str | Path]: ...

    def imu_T_lidar(self) -> SE3: ...

    def imu_T_gt(self) -> SE3: ...

    def imu_params(self) -> ImuParams: ...

    def lidar_params(self) -> LidarParams: ...
```
The most obvious thing to note is the list of sequences; as each Dataset class is an enum, we list all of the sequences as enum members.

After that are the methods. The last four methods are fairly self-explanatory, but we'll elaborate more on the first three.

## data_iter

Arguably the most important method, as is the main interface to the actual data. It returns a [`DatasetIterator`][evalio.datasets.DatasetIterator] object, which provides iterators over the data.

While you are welcome to provide a custom [`DatasetIterator`][evalio.datasets.DatasetIterator], evalio provides some for the most common use cases in [`RosbagIter`][evalio.datasets.RosbagIter] that iterates over topics found in a rosbag, and [`RawDataIter`][evalio.datasets.RawDataIter] takes in iterators for imu and lidar data to return.

An example of [`RosbagIter`][evalio.datasets.RosbagIter] can be found in the [Hilti2022 source code](https://github.com/contagon/evalio/blob/master/python/evalio/datasets/hilti_2022.py) and [`RawDataIter`][evalio.datasets.RawDataIter] in the [Helipr source code](https://github.com/contagon/evalio/blob/master/python/evalio/datasets/helipr.py). There is also examples of each in [evalio-example](https://github.com/contagon/evalio-example).

## ground_truth_raw

This method provides the ground truth trajectory in an arbitrary frame (often a GPS or prism frame). Many times this is provided in a csv file, which can be loaded via [`Trajectory.from_tum`][evalio.types.Trajectory.from_tum] or[`Trajectory.from_csv`][evalio.types.Trajectory.from_csv]. Naturally, [`Trajectory`][evalio.types.Trajectory] can also be created manually from a list of [`Stamp`][evalio.types.Stamp] and [`SE3`][evalio.types.SE3] objects.

It will be transformed into the IMU frame using transform given by the `imu_T_gt` method.

## files

This provides a list of files that are required to run the dataset. evalio will internally use them to check to make sure the dataset is complete before running anything.

If a returned file is a `str` it is assumed to be a path relative to the sequence directory in `EVALIO_DATA`, which is given by [`folder`][evalio.datasets.Dataset.folder]. If it is a `Path` object, it is assumed to be an absolute path stored wherever you desire.

## optionals

Additionally, there is a number of optional methods that you can implement to add more functionality and information to the `evalio ls` command. Each of these already have default implementations. These methods are (continuing from the above example):

```python
    @classmethod
    def dataset_name(cls) -> str: ...

    @staticmethod
    def url() -> str: ...

    def environment(self) -> str: ...

    def vehicle(self) -> str: ...

    def download(self) -> str: ...

    def quick_len(self) -> Optional[int]: ...
```
The first one provides a base dataset name to be used. By default this is the class name converted to snake case, but it can be overridden. This will be the name used when running CLI commands.

The next three are again self-explanatory, all of which provide information for the `evalio ls` command.

`download` does exactly what it says - downloads the datasets. See the [newer college 2020](https://github.com/contagon/evalio/blob/master/python/evalio/datasets/newer_college_2020.py#L157) for an example downloading from google drive, and [hilti 2022](https://github.com/contagon/evalio/blob/master/python/evalio/datasets/hilti_2022.py#L144) for an example downloading directly from a url.

`quick_len` returns a hardcoded number of scans in a dataset, used for `evalio ls` and for computing time estimates in `evalio run`. If not set, evalio will load the data to compute the length.

That's all there is to it! Datasets are fairly simple - mostly just parameter setting and easy-to-use iterator wrappers.
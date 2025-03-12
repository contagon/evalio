from evalio.cli.parser import DatasetBuilder
from evalio.datasets.base import Dataset
import pytest
from pathlib import Path
import pickle
import time

data_dir = Path("tests/data")
dataset_classes = DatasetBuilder._all_datasets()
datasets = [cls(cls.sequences()[0]) for cls in dataset_classes.values()]


@pytest.mark.parametrize("dataset", datasets)
def test_load_imu(dataset: Dataset):
    imu = dataset.first_n_imu_measurement(100)
    with open(data_dir / f"imu_{dataset.name()}.pkl", "rb") as f:
        imu_cached = pickle.load(f)

    assert imu == imu_cached, f"IMU measurements do not match for {dataset.name()}"


@pytest.mark.parametrize("dataset", datasets)
def test_load_lidar(dataset: Dataset):
    lidar = dataset.first_n_lidar_scans(10)
    with open(data_dir / f"lidar_{dataset.name()}.pkl", "rb") as f:
        lidar_cached = pickle.load(f)

    assert lidar == lidar_cached, f"Lidar scans do not match for {dataset.name()}"


# @pytest.mark.parametrize("dataset", datasets)
# def test_speed(dataset):
#     # do it once to make it fresh
#     for _ in dataset.lidar_iter():
#         pass

#     # time the second one
#     start = time.time()
#     for _ in dataset.lidar_iter():
#         pass
#     end = time.time()
#     new = end - start

#     # load the old one
#     with open(data_dir / f"lidar_time_{dataset.name()}.txt", "r") as f:
#         time_cached = float(f.read())

#     # Make sure it opens within 2 seconds of the cached time
#     # Times are stored from loading from SSD so they should be mostly consistent
#     assert new <= time_cached + 2, f"Timing data does not match for {dataset.name()}"

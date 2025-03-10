from evalio.cli.parser import DatasetBuilder
import pytest
from pathlib import Path
import pickle

data_dir = Path("tests/data")
dataset_classes = DatasetBuilder._all_datasets()
datasets = [cls(cls.sequences()[0]) for cls in dataset_classes.values()]

@pytest.mark.parametrize("dataset", datasets)
def test_load_imu(dataset):
    imu = dataset.first_n_imu_measurement(100)
    with open(data_dir / f"imu_{dataset.name()}.pkl", "rb") as f:
        imu_cached = pickle.load(f)

    assert imu == imu_cached, f"IMU measurements do not match for {dataset.name()}"

@pytest.mark.parametrize("dataset", datasets)
def test_load_lidar(dataset):
    lidar = dataset.first_n_lidar_scans(10)
    with open(data_dir / f"lidar_{dataset.name()}.pkl", "rb") as f:
        lidar_cached = pickle.load(f)

    assert lidar == lidar_cached, f"Lidar scans do not match for {dataset.name()}"
from evalio.types import ImuMeasurement, LidarMeasurement
from evalio.cli.parser import DatasetBuilder
from evalio.datasets.base import Dataset
import pytest
from pathlib import Path
import pickle

from utils import check_lidar_eq

data_dir = Path("tests/data")
dataset_classes = DatasetBuilder._all_datasets()
datasets = [
    cls(cls.sequences()[0])
    for cls in dataset_classes.values()
    if (
        cls.check_download(cls.sequences()[0])
        and (data_dir / f"imu_{cls.name()}.pkl").exists()
        and (data_dir / f"lidar_{cls.name()}.pkl").exists()
    )
]


@pytest.mark.parametrize("dataset", datasets)
def test_load_imu(dataset: Dataset):
    imu = dataset.get_one_imu()
    with open(data_dir / f"imu_{dataset.name()}.pkl", "rb") as f:
        imu_cached: ImuMeasurement = pickle.load(f)

    assert imu == imu_cached, f"IMU measurements do not match for {dataset.name()}"


@pytest.mark.parametrize("dataset", datasets)
def test_load_lidar(dataset: Dataset):
    lidar = dataset.get_one_lidar()
    with open(data_dir / f"lidar_{dataset.name()}.pkl", "rb") as f:
        lidar_cached: LidarMeasurement = pickle.load(f)

    check_lidar_eq(lidar, lidar_cached)

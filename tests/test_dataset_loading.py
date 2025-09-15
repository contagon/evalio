import pickle
from enum import Enum, auto
from pathlib import Path

import numpy as np
import pytest
from evalio.cli.parser import DatasetBuilder
from evalio.datasets.base import Dataset
from evalio.types import SE3, ImuMeasurement, LidarMeasurement, Stamp, Trajectory
from utils import check_lidar_eq, isclose_se3, rand_se3

# ------------------------- Loading imu & lidar ------------------------- #
data_dir = Path("tests/data")
dataset_classes = DatasetBuilder.all_datasets()
datasets = [
    cls.sequences()[0]
    for cls in dataset_classes.values()
    if (
        cls.sequences()[0].is_downloaded()
        and (data_dir / f"imu_{cls.dataset_name()}.pkl").exists()
        and (data_dir / f"lidar_{cls.dataset_name()}.pkl").exists()
    )
]


@pytest.mark.parametrize("dataset", datasets)
def test_load_imu(dataset: Dataset):
    imu = dataset.get_one_imu()
    with open(data_dir / f"imu_{dataset.dataset_name()}.pkl", "rb") as f:
        imu_cached: ImuMeasurement = pickle.load(f)

    assert imu == imu_cached, (
        f"IMU measurements do not match for {dataset.dataset_name()}"
    )


@pytest.mark.parametrize("dataset", datasets)
def test_load_lidar(dataset: Dataset):
    lidar = dataset.get_one_lidar()
    with open(data_dir / f"lidar_{dataset.dataset_name()}.pkl", "rb") as f:
        lidar_cached: LidarMeasurement = pickle.load(f)

    check_lidar_eq(lidar_cached, lidar)


# ------------------------- Loading ground truth ------------------------- #
class StampStyle(Enum):
    Seconds = auto()
    Nanoseconds = auto()
    Split = auto()

    def attributes(self) -> list[str]:
        return {
            StampStyle.Seconds: ["sec"],
            StampStyle.Nanoseconds: ["nsec"],
            StampStyle.Split: ["sec", "nsec"],
        }[self]

    def serialize_stamp(self, stamp: Stamp) -> str:
        match self:
            case StampStyle.Seconds:
                return f"{stamp.sec}.{stamp.nsec:09}"
            case StampStyle.Nanoseconds:
                return f"{stamp.to_nsec()}"
            case StampStyle.Split:
                return f"{stamp.sec}, {stamp.nsec}"


def fake_groundtruth() -> Trajectory:
    stamps = [
        Stamp.from_nsec(i + np.random.randint(-500, 500))
        for i in range(1_000, 10_000, 1_000)
    ]
    poses = [rand_se3() for _ in range(len(stamps))]
    return Trajectory(metadata={}, stamps=stamps, poses=poses)


def serialize_gt(gt: Trajectory, style: StampStyle) -> list[str]:
    def serialize_se3(se3: SE3) -> str:
        return f"{se3.trans[0]}, {se3.trans[1]}, {se3.trans[2]}, {se3.rot.qx}, {se3.rot.qy}, {se3.rot.qz}, {se3.rot.qw}"

    return [
        f"{style.serialize_stamp(stamp)}, {serialize_se3(se3)}"
        for stamp, se3 in zip(gt.stamps, gt.poses)
    ]


@pytest.mark.parametrize("style", list(StampStyle))
def test_load_groundtruth(style: StampStyle, tmp_path: Path):
    gt = fake_groundtruth()
    gt_str = serialize_gt(gt, style)

    gt_file = tmp_path / "gt.csv"
    with open(gt_file, "w") as f:
        f.write("\n".join(gt_str))

    gt_returned = Trajectory.from_csv(
        gt_file, fieldnames=style.attributes() + ["x", "y", "z", "qx", "qy", "qz", "qw"]
    )

    for i, (exp, got) in enumerate(zip(gt.poses, gt_returned.poses)):
        assert isclose_se3(exp, got), f"Pose {i} does not match"

    for i, (exp, got) in enumerate(zip(gt.stamps, gt_returned.stamps)):
        assert exp == got, f"Stamp {i} does not match"

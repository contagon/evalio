from pathlib import Path

import numpy as np
from evalio.datasets.loaders import RawDataIter, RosbagIter
from evalio.types import ImuMeasurement, LidarMeasurement, LidarParams, Point, Stamp
from utils import rosbag_saver


def make_imus(num: int = 200) -> list[ImuMeasurement]:
    imus: list[ImuMeasurement] = []
    for i in range(num):
        imus.append(
            ImuMeasurement(
                Stamp.from_sec((i + 1e-6) / num),
                np.random.rand(3),
                np.random.rand(3),
            )
        )

    return imus


def make_lidars(num: int = 10) -> list[LidarMeasurement]:
    lidars: list[LidarMeasurement] = []
    pt = Point()
    for i in range(num):
        # offset a smidge from the imus
        lidars.append(LidarMeasurement(Stamp.from_sec((i + 1e-2) / num), [pt, pt]))

    return lidars


# ------------------------- Make sure imu/lidars are in order and got all of them ------------------------- #
def test_rawdata():
    imus = make_imus()
    lidars = make_lidars()
    iterator = RawDataIter(iter(lidars), iter(imus), len(lidars))

    last_stamp = Stamp.from_sec(0.0)

    count = 0
    exp_count = len(imus) + len(lidars)
    for m in iterator:
        count += 1
        assert last_stamp < m.stamp

    assert count == exp_count


def test_rosbag(tmp_path: Path):
    imus = make_imus()
    lidars = make_lidars()
    bag = tmp_path / "bags"
    bag.mkdir(parents=True, exist_ok=True)
    bag = bag / "test.bag"
    rosbag_saver(bag, imus, lidars)

    iterator = RosbagIter(
        bag,
        "/lidar",
        "/imu",
        lidar_params=LidarParams(
            num_rows=2, num_columns=1, min_range=1.0, max_range=200.0
        ),
    )

    last_stamp = Stamp.from_sec(0.0)

    count = 0
    exp_count = len(imus) + len(lidars)
    for m in iterator:
        count += 1
        assert last_stamp < m.stamp

    assert count == exp_count

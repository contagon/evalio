from dataclasses import dataclass
from pathlib import Path
from evalio.types import ImuMeasurement, LidarMeasurement, Stamp, Point, LidarParams
from evalio.datasets.iterators import RosbagIter
import numpy as np
import pytest

from utils import rosbag_saver

# Create a row major point cloud
WIDTH = 256
HEIGHT = 32

PARAMS = LidarParams(
    num_rows=HEIGHT,
    num_columns=WIDTH,
    min_range=0.1,
    max_range=100.0,
)

np.random.seed(0)


# ------------------------- Helpers for making/saving ------------------------- #
def make_imu():
    return ImuMeasurement(
        Stamp.from_sec(1.0),
        np.random.rand(3),
        np.random.rand(3),
    )


def make_lidar() -> LidarMeasurement:
    all_points = []
    for i in range(HEIGHT):
        for j in range(WIDTH):
            t = Stamp.from_sec(j / (WIDTH * 10))
            x, y, z, intensity = np.random.rand(4)
            r = np.random.randint(0, 1000)
            point = Point(
                x=x,
                y=y,
                z=z,
                intensity=intensity,
                t=t,
                range=r,
                row=i,
                col=j,
            )
            all_points.append(point)

    return LidarMeasurement(Stamp.from_sec(0.1), all_points)


@dataclass
class Fixture:
    imu: ImuMeasurement
    lidar: LidarMeasurement
    bag: Path


def get_mm(iterator: RosbagIter):
    imu = next(iterator.imu_iter())
    lidar = next(iterator.lidar_iter())
    return imu, lidar


# ------------------------- The tests ------------------------- #
# all points row major
@pytest.fixture(scope="session")
def all_row_major(tmp_path_factory):
    bag = tmp_path_factory.mktemp("bag") / "all_row_major.bag"

    imu = make_imu()
    lidar = make_lidar()

    rosbag_saver(bag, [imu], [lidar])
    return Fixture(imu, lidar, bag)


def test_all_row_major(all_row_major: Fixture):
    iterator = RosbagIter(
        all_row_major.bag,
        lidar_topic="/lidar",
        imu_topic="/imu",
        lidar_params=PARAMS,
    )

    imu, lidar = get_mm(iterator)
    assert imu == all_row_major.imu
    if lidar != all_row_major.lidar:
        assert lidar.stamp == all_row_major.lidar.stamp, "stamps do not match"
        for i, (got, exp) in enumerate(zip(lidar.points, all_row_major.lidar.points)):
            assert got == exp, f"p{i}, {got} != {exp}"


# TODO: Test for column major, and all/onlyvalid points
# TODO: Also test relative begin/endstamps??


# all points col major
@pytest.fixture(scope="session")
def all_col_major(tmp_path_factory):
    bag = tmp_path_factory.mktemp("bag") / "all_col_major.bag"

    imu = make_imu()
    lidar_expected = make_lidar()

    # Convert the row major lidar to column major
    # pull out the points from C++, otherwise its sloowww
    original_points = lidar_expected.points
    points = []
    for i in range(WIDTH):
        for j in range(HEIGHT):
            points.append(original_points[j * WIDTH + i])
    lidar_col_major = LidarMeasurement(lidar_expected.stamp, points)

    rosbag_saver(bag, [imu], [lidar_col_major])
    return Fixture(imu, lidar_expected, bag)


def test_all_col_major(all_col_major: Fixture):
    iterator = RosbagIter(
        all_col_major.bag,
        lidar_topic="/lidar",
        imu_topic="/imu",
        lidar_params=PARAMS,
    )

    imu, lidar = get_mm(iterator)
    assert imu == all_col_major.imu
    if lidar != all_col_major.lidar:
        assert lidar.stamp == all_col_major.lidar.stamp, "stamps do not match"
        for i, (got, exp) in enumerate(zip(lidar.points, all_col_major.lidar.points)):
            assert got == exp, f"p{i}, {got} != {exp}"

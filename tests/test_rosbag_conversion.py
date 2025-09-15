from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest
from evalio.datasets.loaders import (
    LidarDensity,
    LidarMajor,
    LidarPointStamp,
    RosbagIter,
)
from evalio.types import (
    Duration,
    ImuMeasurement,
    LidarMeasurement,
    LidarParams,
    Point,
    Stamp,
)
from utils import check_lidar_eq, rosbag_saver

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
    all_points: list[Point] = []
    for i in range(HEIGHT):
        for j in range(WIDTH):
            t = Duration.from_sec(j / (WIDTH * 10))
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
def all_row_major(tmp_path_factory: pytest.TempPathFactory):
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
    check_lidar_eq(all_row_major.lidar, lidar)

    assert iterator.lidar_format.major == LidarMajor.Row
    assert iterator.lidar_format.density == LidarDensity.AllPoints
    assert iterator.lidar_format.point_stamp == LidarPointStamp.Start


# all points col major
@pytest.fixture(scope="session")
def all_col_major(tmp_path_factory: pytest.TempPathFactory):
    bag = tmp_path_factory.mktemp("bag") / "all_col_major.bag"

    imu = make_imu()
    lidar_expected = make_lidar()

    # Convert the row major lidar to column major
    # pull out the points from C++, otherwise its sloowww
    original_points = lidar_expected.points
    points: list[Point] = []
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
    check_lidar_eq(all_col_major.lidar, lidar)

    assert iterator.lidar_format.major == LidarMajor.Column
    assert iterator.lidar_format.density == LidarDensity.AllPoints
    assert iterator.lidar_format.point_stamp == LidarPointStamp.Start


# only valid row major
@pytest.fixture(scope="session")
def only_valid_row_major(tmp_path_factory: pytest.TempPathFactory):
    bag = tmp_path_factory.mktemp("bag") / "only_valid_row_major.bag"

    imu = make_imu()
    lidar = make_lidar()

    # sort into rows
    points_to_save = lidar.points
    points_expected = [
        deepcopy(points_to_save[i : i + WIDTH])
        for i in range(0, len(points_to_save), WIDTH)
    ]
    # Get points to randomly drop
    drop_points = [
        (r * WIDTH + i, r, i)
        for r in range(HEIGHT)
        for i in np.random.choice(range(WIDTH), WIDTH // 8, replace=False)
    ]
    drop_points.sort(reverse=True)

    for idx, r, i in drop_points:
        # Move an empty point to end of the row for expected
        points_expected[r].pop(i)
        points_expected[r].append(Point(row=r))
        # drop the point from the list to save
        points_to_save.pop(idx)

    # fill out col guesses - it's what we expect to do
    for r in range(HEIGHT):
        for i in range(WIDTH):
            points_expected[r][i].col = i

    # Save the bag with missing points
    rosbag_saver(bag, [imu], [LidarMeasurement(lidar.stamp, points_to_save)])
    # return the others
    empty: list[Point] = []
    points_expected = sum(points_expected, empty)
    return Fixture(imu, LidarMeasurement(lidar.stamp, points_expected), bag)


def test_only_valid_row_major(
    capsys: pytest.CaptureFixture[str], only_valid_row_major: Fixture
):
    iterator = RosbagIter(
        only_valid_row_major.bag,
        lidar_topic="/lidar",
        imu_topic="/imu",
        lidar_params=PARAMS,
    )

    imu, lidar = get_mm(iterator)

    assert imu == only_valid_row_major.imu
    check_lidar_eq(only_valid_row_major.lidar, lidar)

    assert iterator.lidar_format.major == LidarMajor.Row
    assert iterator.lidar_format.density == LidarDensity.OnlyValidPoints
    assert iterator.lidar_format.point_stamp == LidarPointStamp.Start

    # also check that we were warned this is undefined
    captured = capsys.readouterr()
    exp_err = "Warning: Loading row major scan with only valid points. Can't identify where missing points should go, putting at end of scanline"
    assert exp_err == captured.out.strip()


# only valid col major
@pytest.fixture(scope="session")
def only_valid_col_major(tmp_path_factory: pytest.TempPathFactory):
    bag = tmp_path_factory.mktemp("bag") / "only_valid_col_major.bag"

    imu = make_imu()
    lidar = make_lidar()

    # Switch points to column major
    to_return = lidar.points
    to_save: list[Point] = []
    for i in range(WIDTH):
        for j in range(HEIGHT):
            to_save.append(deepcopy(to_return[j * WIDTH + i]))

    # Get points to randomly drop
    drop_points = [
        (c * HEIGHT + r, r, c)
        for r in range(HEIGHT)
        for c in np.random.choice(range(WIDTH), WIDTH // 8, replace=False)
    ]
    # sort in reverse order so we can pop from the end to the start
    drop_points.sort(reverse=True)

    for idx, r, c in drop_points:
        # Set the empty point to zero in our expected return
        to_return[r * WIDTH + c] = Point(row=r, col=c)
        # remove it from the saved return
        to_save.pop(idx)

    # Save the bag with missing points
    rosbag_saver(bag, [imu], [LidarMeasurement(lidar.stamp, to_save)])
    return Fixture(imu, LidarMeasurement(lidar.stamp, to_return), bag)


def test_only_valid_col_major(only_valid_col_major: Fixture):
    iterator = RosbagIter(
        only_valid_col_major.bag,
        lidar_topic="/lidar",
        imu_topic="/imu",
        lidar_params=PARAMS,
    )

    imu, lidar = get_mm(iterator)

    assert imu == only_valid_col_major.imu
    check_lidar_eq(only_valid_col_major.lidar, lidar)

    assert iterator.lidar_format.major == LidarMajor.Column
    assert iterator.lidar_format.density == LidarDensity.OnlyValidPoints
    assert iterator.lidar_format.point_stamp == LidarPointStamp.Start


# time stamps relative to the end
@pytest.fixture(scope="session")
def point_stamp_relative_end(tmp_path_factory: pytest.TempPathFactory):
    bag = tmp_path_factory.mktemp("bag") / "point_stamp_relative_end.bag"

    imu = make_imu()
    lidar = make_lidar()

    # Move the lidar points to be relative to the end
    original_points = lidar.points
    points: list[Point] = []
    for i in range(len(original_points)):
        points.append(deepcopy(original_points[i]))
        points[i].t = points[i].t - PARAMS.delta_time()

    rosbag_saver(bag, [imu], [LidarMeasurement(lidar.stamp, points)])
    return Fixture(imu, lidar, bag)


def test_point_stamp_relative_end(point_stamp_relative_end: Fixture):
    iterator = RosbagIter(
        point_stamp_relative_end.bag,
        lidar_topic="/lidar",
        imu_topic="/imu",
        lidar_params=PARAMS,
    )

    imu, lidar = get_mm(iterator)
    assert imu == point_stamp_relative_end.imu
    check_lidar_eq(point_stamp_relative_end.lidar, lidar)

    assert iterator.lidar_format.point_stamp == LidarPointStamp.End

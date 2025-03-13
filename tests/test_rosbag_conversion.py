from dataclasses import dataclass
from pathlib import Path
from evalio.types import ImuMeasurement, LidarMeasurement, Stamp, Point, LidarParams
from evalio.datasets.iterators import RosbagIter
import numpy as np
import pytest

# We arbitrarily choose to use ROS1 Noetic for this example
# but the rosbags abstraction to make it ROS version agnostic
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.stores.ros1_noetic import (
    builtin_interfaces__msg__Time as RosTime,
    std_msgs__msg__Header as Header,
    sensor_msgs__msg__PointCloud2 as RosPointCloud2,
    sensor_msgs__msg__PointField as RosPointField,
    sensor_msgs__msg__Imu as RosImu,
    geometry_msgs__msg__Vector3 as RosVector3,
    geometry_msgs__msg__Quaternion as RosQuaternion,
)

# Create a row major point cloud
WIDTH = 1024
HEIGHT = 64

PARAMS = LidarParams(
    num_rows=HEIGHT,
    num_columns=WIDTH,
    min_range=0.1,
    max_range=100.0,
)

np.random.seed(0)


# ------------------------- Helpers for serializing ------------------------- #
def point_to_bytes(point: Point) -> bytes:
    """Convert a point to a byte array."""
    return (
        np.float64(point.x).tobytes()
        + np.float64(point.y).tobytes()
        + np.float64(point.z).tobytes()
        + np.float64(point.intensity).tobytes()
        + np.uint32(point.t.to_nsec()).tobytes()
        + np.uint32(point.range).tobytes()
        + np.uint16(point.col).tobytes()
        + np.uint8(point.row).tobytes()
    )


def scan_to_bytes(scan: LidarMeasurement) -> bytes:
    """Convert a scan to a byte array."""
    return b"".join([point_to_bytes(point) for point in scan.points])


def rosbag_saver(filename: Path, imu: ImuMeasurement, lidar: LidarMeasurement):
    """Save an imu and lidar measurement to the output bag."""
    typestore = get_typestore(Stores.ROS1_NOETIC)
    with Writer(filename) as writer:
        # Write IMU measurement
        conn = writer.add_connection("/imu", RosImu.__msgtype__, typestore=typestore)
        timestamp = imu.stamp.to_nsec()
        msg = RosImu(
            header=Header(
                seq=0,
                stamp=RosTime(sec=imu.stamp.sec, nanosec=imu.stamp.nsec),
                frame_id="imu",
            ),
            angular_velocity=RosVector3(x=imu.gyro[0], y=imu.gyro[1], z=imu.gyro[2]),
            linear_acceleration=RosVector3(
                x=imu.accel[0], y=imu.accel[1], z=imu.accel[2]
            ),
            orientation=RosQuaternion(x=0, y=0, z=0, w=1),
            orientation_covariance=np.zeros(9),
            angular_velocity_covariance=np.zeros(9),
            linear_acceleration_covariance=np.zeros(9),
        )
        writer.write(conn, timestamp, typestore.serialize_ros1(msg, RosImu.__msgtype__))

        # Write LIDAR measurement
        conn = writer.add_connection(
            "/lidar", RosPointCloud2.__msgtype__, typestore=typestore
        )
        timestamp = lidar.stamp.to_nsec()
        msg = RosPointCloud2(
            header=Header(
                seq=0,
                stamp=RosTime(sec=lidar.stamp.sec, nanosec=lidar.stamp.nsec),
                frame_id="lidar",
            ),
            is_dense=len(lidar.points) == WIDTH * HEIGHT,
            height=1,
            width=len(lidar.points),
            fields=[
                RosPointField(name="x", offset=0, datatype=8, count=1),  # float64
                RosPointField(name="y", offset=8, datatype=8, count=1),  # float64
                RosPointField(name="z", offset=16, datatype=8, count=1),  # float64
                RosPointField(
                    name="intensity", offset=24, datatype=8, count=1
                ),  # float64
                RosPointField(name="time", offset=32, datatype=6, count=1),  # uint32
                RosPointField(name="range", offset=36, datatype=6, count=1),  # uint32
                RosPointField(name="col", offset=40, datatype=4, count=1),  # uint16
                RosPointField(name="row", offset=42, datatype=2, count=1),  # uint8
            ],
            is_bigendian=False,
            point_step=43,
            row_step=43 * len(lidar.points),
            data=np.frombuffer(scan_to_bytes(lidar), dtype=np.uint8),
        )
        writer.write(
            conn, timestamp, typestore.serialize_ros1(msg, RosPointCloud2.__msgtype__)
        )


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

    rosbag_saver(bag, imu, lidar)
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

    rosbag_saver(bag, imu, lidar_col_major)
    return Fixture(imu, lidar_expected, bag)


def test_all_col_major(all_col_major: Fixture):
    print(all_col_major.bag)
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

from pathlib import Path

# We arbitrarily choose to use ROS1 Noetic for this example
# but the rosbags abstraction should make it ROS version agnostic
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

from evalio.types import ImuMeasurement, LidarMeasurement, Point

import numpy as np


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


def rosbag_saver(
    filename: Path, list_imu: list[ImuMeasurement], list_lidar: list[LidarMeasurement]
):
    """Save an imu and lidar measurement to the output bag."""
    typestore = get_typestore(Stores.ROS1_NOETIC)
    with Writer(filename) as writer:
        # Write IMU measurement
        conn = writer.add_connection("/imu", RosImu.__msgtype__, typestore=typestore)
        for imu in list_imu:
            timestamp = imu.stamp.to_nsec()
            msg = RosImu(
                header=Header(
                    seq=0,
                    stamp=RosTime(sec=imu.stamp.sec, nanosec=imu.stamp.nsec),
                    frame_id="imu",
                ),
                angular_velocity=RosVector3(
                    x=imu.gyro[0], y=imu.gyro[1], z=imu.gyro[2]
                ),
                linear_acceleration=RosVector3(
                    x=imu.accel[0], y=imu.accel[1], z=imu.accel[2]
                ),
                orientation=RosQuaternion(x=0, y=0, z=0, w=1),
                orientation_covariance=np.zeros(9),
                angular_velocity_covariance=np.zeros(9),
                linear_acceleration_covariance=np.zeros(9),
            )
            writer.write(
                conn, timestamp, typestore.serialize_ros1(msg, RosImu.__msgtype__)
            )

        # Write LIDAR measurement
        conn = writer.add_connection(
            "/lidar", RosPointCloud2.__msgtype__, typestore=typestore
        )
        for lidar in list_lidar:
            timestamp = lidar.stamp.to_nsec()
            msg = RosPointCloud2(
                header=Header(
                    seq=0,
                    stamp=RosTime(sec=lidar.stamp.sec, nanosec=lidar.stamp.nsec),
                    frame_id="lidar",
                ),
                is_dense=False,  # We don't use this
                height=1,
                width=len(lidar.points),
                fields=[
                    RosPointField(name="x", offset=0, datatype=8, count=1),  # float64
                    RosPointField(name="y", offset=8, datatype=8, count=1),  # float64
                    RosPointField(name="z", offset=16, datatype=8, count=1),  # float64
                    RosPointField(
                        name="intensity", offset=24, datatype=8, count=1
                    ),  # float64
                    RosPointField(
                        name="time", offset=32, datatype=6, count=1
                    ),  # uint32
                    RosPointField(
                        name="range", offset=36, datatype=6, count=1
                    ),  # uint32
                    RosPointField(name="col", offset=40, datatype=4, count=1),  # uint16
                    RosPointField(name="row", offset=42, datatype=2, count=1),  # uint8
                ],
                is_bigendian=False,
                point_step=43,
                row_step=43 * len(lidar.points),
                data=np.frombuffer(scan_to_bytes(lidar), dtype=np.uint8),
            )
            writer.write(
                conn,
                timestamp,
                typestore.serialize_ros1(msg, RosPointCloud2.__msgtype__),
            )

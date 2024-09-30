from typing import Protocol, Union
from rosbags.highlevel import AnyReader

import numpy as np
import os
import csv
from pathlib import Path

from evalio._cpp import (  # type: ignore
    SO3,
    SE3,
    Stamp,
    ImuMeasurement,
    LidarMeasurement,
    LidarParams,
    ImuParams,
    Field,
    DataType,
    PointCloudMetadata,
    ros_pc2_to_evalio,
)

Measurement = Union[ImuMeasurement, LidarMeasurement]

EVALIO_DATA = Path(os.getenv("EVALIO_DATA", "./"))


class Dataset(Protocol):
    def __init__(self, seq: str): ...

    def __iter__(self): ...

    def download(self, seq: str):
        raise NotImplementedError("Download not implemented")

    # TODO: Does these need to be stamped?
    def ground_truth(self) -> list[SE3]: ...

    @staticmethod
    def name() -> str: ...

    @staticmethod
    def sequences() -> list[str]: ...

    @staticmethod
    def imu_T_lidar() -> SE3: ...

    @staticmethod
    def imu_T_gt() -> SE3: ...

    @staticmethod
    def imu_params() -> ImuParams: ...

    @staticmethod
    def lidar_params() -> LidarParams: ...


# ------------------------- Helpers ------------------------- #


def pointcloud2_to_evalio(msg) -> LidarMeasurement:
    # Convert to C++ types
    fields = []
    for f in msg.fields:
        fields.append(
            Field(name=f.name, datatype=DataType(f.datatype), offset=f.offset)
        )

    stamp = Stamp(sec=msg.header.stamp.sec, nsec=msg.header.stamp.nanosec)

    cloud = PointCloudMetadata(
        stamp=stamp,
        height=msg.height,
        width=msg.width,
        row_step=msg.row_step,
        point_step=msg.point_step,
        is_bigendian=msg.is_bigendian,
        is_dense=msg.is_dense,
    )

    return ros_pc2_to_evalio(cloud, fields, bytes(msg.data))


def imu_to_evalio(msg) -> ImuMeasurement:
    acc = msg.linear_acceleration
    acc = np.array([acc.x, acc.y, acc.z])
    gyro = msg.angular_velocity
    gyro = np.array([gyro.x, gyro.y, gyro.z])

    stamp = Stamp(sec=msg.header.stamp.sec, nsec=msg.header.stamp.nanosec)
    return ImuMeasurement(stamp, gyro, acc)


# These are helpers to help with common dataset types
class RosbagIter:
    def __init__(self, path: Path, lidar_topic: str, imu_topic: str):
        self.lidar_topic = lidar_topic
        self.imu_topic = imu_topic

        # Glob to get all .bag files in the directory
        if path.is_dir():
            self.path = list(path.glob("*.bag"))
            if not self.path:
                raise FileNotFoundError(f"No .bag files found in directory {path}")
        else:
            self.path = [path]

        # Open the bag file
        self.reader = AnyReader(self.path)
        self.reader.open()
        connections = [
            x
            for x in self.reader.connections
            if x.topic in [self.lidar_topic, self.imu_topic]
        ]
        self.iterator = self.reader.messages(connections=connections)

    def __iter__(self):
        return self

    def __next__(self) -> Measurement:
        # TODO: Ending somehow
        connection, timestamp, rawdata = next(self.iterator)

        msg = self.reader.deserialize(rawdata, connection.msgtype)

        if connection.msgtype == "sensor_msgs/msg/PointCloud2":
            return pointcloud2_to_evalio(msg)
        elif connection.msgtype == "sensor_msgs/msg/Imu":
            return imu_to_evalio(msg)
        else:
            raise ValueError(f"Unknown message type {connection.msgtype}")


def load_pose_csv(path: str) -> dict[Stamp, SE3]:
    poses = {}

    with open(path) as csvfile:
        reader = csv.DictReader(csvfile)
        for line in reader:
            r = SO3(
                qw=float(line["qw"]),
                qx=float(line["qx"]),
                qy=float(line["qy"]),
                qz=float(line["qz"]),
            )
            t = np.array([float(line["x"]), float(line["y"]), float(line["z"])])
            pose = SE3(r, t)
            stamp = Stamp(sec=int(line["#sec"]), nsec=int(line["nsec"]))
            poses[stamp] = pose

    return poses

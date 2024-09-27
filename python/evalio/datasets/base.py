from typing import Protocol, Union
from rosbags.typesys import Stores
from rosbags.rosbag1 import Reader as Reader1
from rosbags.rosbag2 import Reader as Reader2
import rosbags

import numpy as np
import os
import csv
from pathlib import Path

from evalio._cpp import SE3, Stamp, ImuMeasurement, LidarMeasurement, ImuParams  # type: ignore


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


# ------------------------- Helpers ------------------------- #
# These are helpers to help with common dataset types
class RosbagIter:
    def __init__(self, path: str, kind: Stores, lidar_topic: str, imu_topic: str):
        self.path = path
        if kind == Stores.ROS1_NOETIC:
            self.version = 1
            self.reader = Reader1(path)
        else:
            self.version = 1
            self.reader = Reader2(path)

        self.typestore = rosbags.typesys.get_typestore(kind)

        self.lidar_topic = lidar_topic
        self.imu_topic = imu_topic

    def __iter__(self):
        self.reader.open()
        connections = [
            x for x in self.reader.connections if x.topic in [self.imu_topic]
        ]
        print(connections)
        self.iterator = self.reader.messages(connections=connections)
        return self

    def __next__(self) -> Measurement:
        # TODO: Parse point cloud
        # TODO: Switch to new ImuMeasurement class
        # TODO: Ending somehow
        connection, timestamp, rawdata = next(self.iterator)

        if self.version == 1:
            msg = self.typestore.deserialize_ros1(rawdata, connection.msgtype)
        else:
            msg = self.typestore.deserialize_cdr(rawdata, connection.msgtype)

        acc = msg.linear_acceleration
        acc = np.array([acc.x, acc.y, acc.z])
        gyro = msg.angular_velocity
        gyro = np.array([gyro.x, gyro.y, gyro.z])

        stamp = Stamp(msg.header.stamp.sec, msg.header.stamp.nanosec)
        mm = ImuMeasurement(stamp, gyro, acc)

        return mm


def load_pose_csv(path: str) -> dict[Stamp, SE3]:
    poses = {}

    with open(path) as csvfile:
        reader = csv.DictReader(csvfile)
        for line in reader:
            # TODO: Redo this w/ SE3
            pass
            # r = gtsam.Rot3.Quaternion(
            #     float(line["qw"]),
            #     float(line["qx"]),
            #     float(line["qy"]),
            #     float(line["qz"]),
            # )
            # t = np.array([float(line["x"]), float(line["y"]), float(line["z"])])
            # pose = SE3(r, t)
            # stamp = Stamp(int(line["#sec"]), int(line["nsec"]))
            # poses[stamp] = pose

    return poses

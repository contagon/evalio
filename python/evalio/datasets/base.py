from typing import Protocol, Union
from rosbags.highlevel import AnyReader

import numpy as np
import os
import csv
from pathlib import Path

from dataclasses import dataclass

from evalio._cpp.types import (  # type: ignore
    SO3,
    SE3,
    Stamp,
    ImuMeasurement,
    LidarMeasurement,
    LidarParams,
    ImuParams,
)

from evalio._cpp._helpers import (  # type: ignore
    Field,
    DataType,
    PointCloudMetadata,
    ros_pc2_to_evalio,
)

Measurement = Union[ImuMeasurement, LidarMeasurement]

EVALIO_DATA = Path(os.getenv("EVALIO_DATA", "./"))

# TODO: Ponder: https://stackoverflow.com/a/17496524


@dataclass
class Dataset(Protocol):
    seq: str

    # ------------------------- For loading data ------------------------- #
    def __iter__(self): ...

    def ground_truth(self) -> list[(Stamp, SE3)]: ...

    # ------------------------- For loading params ------------------------- #
    @staticmethod
    def name() -> str: ...

    @staticmethod
    def nickname() -> str: ...

    @staticmethod
    def sequences() -> list[str]: ...

    @staticmethod
    def nicksequences() -> list[str]: ...

    @staticmethod
    def imu_T_lidar() -> SE3: ...

    @staticmethod
    def imu_T_gt() -> SE3: ...

    @staticmethod
    def imu_params() -> ImuParams: ...

    @staticmethod
    def lidar_params() -> LidarParams: ...

    # ------------------------- For downloading ------------------------- #
    @staticmethod
    def check_download(self, seq: str) -> bool:
        return True

    @staticmethod
    def download(self, seq: str) -> None:
        raise NotImplementedError("Download not implemented")

    # ------------------------- Helpers ------------------------- #
    def __post_init__(self):
        self.seq = self.process_seq(self.seq)

        if not self.check_download(self.seq):
            raise ValueError(
                f"Data for {self.seq} not found, please use `evalio download {self.name()}/{self.seq}` to download"
            )

    @classmethod
    def process_seq(cls, seq: str):
        if seq in cls.sequences():
            return seq
        elif seq in cls.nicksequences():
            idx = cls.nicksequences().index(seq)
            return cls.sequences()[idx]
        else:
            raise ValueError(f"Sequence {seq} not in {cls.name()}")

    def ground_truth_corrected(self, imu_o_T_imu_0: SE3) -> list[SE3]:
        # Load all transforms
        gt_poses = self.ground_truth()
        stamp, gt_o_T_gt_0 = gt_poses[0]
        gt_T_imu = self.imu_T_gt().inverse()

        # compute imu_o_T_gt_o
        gt_o_T_imu_0 = gt_o_T_gt_0 * gt_T_imu
        imu_o_T_gt_o = imu_o_T_imu_0 * gt_o_T_imu_0.inverse()

        # Clean all poses
        gt_poses_corrected = []
        for stamp, gt_o_T_gt_i in gt_poses:
            gt_poses_corrected.append(imu_o_T_gt_o * gt_o_T_gt_i * gt_T_imu)

        return gt_poses_corrected


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

        self.lidar_count = sum(
            [x.msgcount for x in connections if x.topic == self.lidar_topic]
        )

        self.iterator = self.reader.messages(connections=connections)

    def __len__(self):
        return self.lidar_count

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


def load_pose_csv(
    path: str, fieldnames: list[str], delimiter=","
) -> list[(Stamp, SE3)]:
    poses = []

    with open(path) as csvfile:
        csvfile = filter(lambda row: row[0] != "#", csvfile)
        reader = csv.DictReader(csvfile, fieldnames=fieldnames, delimiter=delimiter)
        for line in reader:
            r = SO3(
                qw=float(line["qw"]),
                qx=float(line["qx"]),
                qy=float(line["qy"]),
                qz=float(line["qz"]),
            )
            t = np.array([float(line["x"]), float(line["y"]), float(line["z"])])
            pose = SE3(r, t)

            if "nsec" not in fieldnames:
                stamp = Stamp.from_sec(float(line["sec"]))
            elif "sec" not in fieldnames:
                stamp = Stamp.from_nsec(float(line["nsec"]))
            else:
                stamp = Stamp(sec=int(line["sec"]), nsec=int(line["nsec"]))
            poses.append((stamp, pose))

    return poses

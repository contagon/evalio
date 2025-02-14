import csv
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Optional, Protocol, Union

import numpy as np
from rosbags.highlevel import AnyReader
from tabulate import tabulate

from evalio._cpp._helpers import (  # type: ignore
    DataType,
    Field,
    PointCloudMetadata,
    ros_pc2_to_evalio,
    helipr_bin_to_evalio,
)
from evalio.types import (
    SE3,
    SO3,
    ImuMeasurement,
    ImuParams,
    LidarMeasurement,
    LidarParams,
    Stamp,
    Trajectory,
)

Measurement = Union[ImuMeasurement, LidarMeasurement]

EVALIO_DATA = Path(os.getenv("EVALIO_DATA", "./"))


@dataclass
class Dataset(Protocol):
    seq: str
    length: Optional[int] = None

    # ------------------------- For loading data ------------------------- #
    def __iter__(self) -> Iterator[Measurement]: ...

    def ground_truth_raw(self) -> Trajectory: ...

    # ------------------------- For loading params ------------------------- #
    @staticmethod
    def url() -> str: ...

    @staticmethod
    def name() -> str: ...

    @staticmethod
    def sequences() -> list[str]: ...

    def imu_T_lidar(self) -> SE3: ...

    def imu_T_gt(self) -> SE3: ...

    def imu_params(self) -> ImuParams: ...

    def lidar_params(self) -> LidarParams: ...

    # ------------------------- For downloading ------------------------- #
    @staticmethod
    def check_download(seq: str) -> bool:
        return True

    @staticmethod
    def download(seq: str) -> None:
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
        if seq not in cls.sequences():
            raise ValueError(f"Sequence {seq} not in {cls.name()}")

        return seq

    def ground_truth(self) -> Trajectory:
        gt_traj = self.ground_truth_raw()
        gt_T_imu = self.imu_T_gt().inverse()

        # Convert to IMU frame
        for i in range(len(gt_traj)):
            gt_o_T_gt_i = gt_traj.poses[i]
            gt_traj.poses[i] = gt_o_T_gt_i * gt_T_imu

        return gt_traj

    def __str__(self):
        return f"{self.name()}/{self.seq}"


# ------------------------- Helpers ------------------------- #


def pointcloud2_to_evalio(msg, params: LidarParams) -> LidarMeasurement:
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

    return ros_pc2_to_evalio(cloud, fields, bytes(msg.data), params)  # type: ignore


def imu_to_evalio(msg) -> ImuMeasurement:
    acc = msg.linear_acceleration
    acc = np.array([acc.x, acc.y, acc.z])
    gyro = msg.angular_velocity
    gyro = np.array([gyro.x, gyro.y, gyro.z])

    stamp = Stamp(sec=msg.header.stamp.sec, nsec=msg.header.stamp.nanosec)
    return ImuMeasurement(stamp, gyro, acc)


# These are helpers to help with common dataset types
class RosbagIter:
    def __init__(
        self,
        path: Path,
        lidar_topic: str,
        imu_topic: str,
        params,
        is_mcap: bool = False,
    ):
        self.lidar_topic = lidar_topic
        self.imu_topic = imu_topic
        self.params = params

        # Glob to get all .bag files in the directory
        if path.is_dir() and is_mcap is False:
            self.path = [p for p in path.glob("*.bag") if "orig" not in str(p)]
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
        if len(connections) == 0:
            connections_all = [[c.topic, c.msgtype] for c in self.reader.connections]
            print(
                tabulate(
                    connections_all, headers=["Topic", "MsgType"], tablefmt="fancy_grid"
                )
            )
            raise ValueError(
                f"Could not find topics {self.lidar_topic} or {self.imu_topic}"
            )

        self.lidar_count = sum(
            [x.msgcount for x in connections if x.topic == self.lidar_topic]
        )

        self.iterator = self.reader.messages(connections=connections)

    def __len__(self):
        return self.lidar_count

    def __iter__(self):
        return self

    def __next__(self) -> Measurement:
        connection, timestamp, rawdata = next(self.iterator)

        msg = self.reader.deserialize(rawdata, connection.msgtype)

        if connection.msgtype == "sensor_msgs/msg/PointCloud2":
            return pointcloud2_to_evalio(msg, self.params)
        elif connection.msgtype == "sensor_msgs/msg/Imu":
            return imu_to_evalio(msg)
        else:
            raise ValueError(f"Unknown message type {connection.msgtype}")


class RawDataIter:
    def __init__(self, lidar_path: Path, imu_file: Path, lidar_params: LidarParams):
        self.lidar_path = lidar_path
        self.imu_file = imu_file

        # Load all IMU data
        imu_stamps = np.loadtxt(imu_file, usecols=0, dtype=np.int64, delimiter=",")
        self.imu_stamps = [Stamp.from_nsec(x) for x in imu_stamps]
        imu_data = np.loadtxt(imu_file, usecols=(11, 12, 13, 14, 15, 16), delimiter=",")
        self.imu_gyro = imu_data[:, :3]
        self.imu_acc = imu_data[:, 3:]

        self.lidar_params = lidar_params
        self.lidar_files = sorted(list(lidar_path.glob("*")))
        self.lidar_stamps = [Stamp.from_nsec(int(x.stem)) for x in self.lidar_files]

        self.idx_imu = 0
        self.idx_lidar = 0

    def __len__(self):
        return len(self.lidar_files)

    def __iter__(self):
        return self

    def __next__(self) -> Measurement:
        if self.idx_imu >= len(self.imu_stamps) or self.idx_lidar >= len(
            self.lidar_stamps
        ):
            raise StopIteration

        if self.imu_stamps[self.idx_imu] < self.lidar_stamps[self.idx_lidar]:
            mm = ImuMeasurement(
                self.imu_stamps[self.idx_imu],
                self.imu_gyro[self.idx_imu],
                self.imu_acc[self.idx_imu],
            )
            self.idx_imu += 1
            return mm
        else:
            file = self.lidar_files[self.idx_lidar]
            stamp = self.lidar_stamps[self.idx_lidar]
            self.idx_lidar += 1
            return helipr_bin_to_evalio(str(file), stamp, self.lidar_params)


def load_pose_csv(
    path: Path, fieldnames: list[str], delimiter=",", skip_lines: Optional[int] = None
) -> Trajectory:
    poses = []
    stamps = []

    with open(path) as f:
        csvfile = list(filter(lambda row: row[0] != "#", f))
        if skip_lines is not None:
            csvfile = csvfile[skip_lines:]
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

            if "t" in fieldnames:
                line["sec"] = line["t"]

            if "nsec" not in fieldnames:
                s, ns = line["sec"].split(".")  # parse separately to get exact stamp
                ns = ns.ljust(9, "0")  # pad to 9 digits for nanoseconds
                stamp = Stamp(sec=int(s), nsec=int(ns))
            elif "sec" not in fieldnames:
                stamp = Stamp.from_nsec(int(line["nsec"]))
            else:
                stamp = Stamp(sec=int(line["sec"]), nsec=int(line["nsec"]))
            poses.append(pose)
            stamps.append(stamp)

    return Trajectory(metadata={}, stamps=stamps, poses=poses)


def load_tum(path: Path) -> Trajectory:
    return load_pose_csv(path, ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"])

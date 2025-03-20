from collections.abc import Iterator
from typing import Callable, Any, Optional, TypeVar
from pathlib import Path

from evalio._cpp._helpers import (  # type: ignore
    DataType,
    Field,
    PointCloudMetadata,
    pc2_to_evalio,
    fill_col_col_major,
    fill_col_row_major,
    reorder_points,
    shift_point_stamps,
)
from evalio.types import (
    ImuMeasurement,
    LidarMeasurement,
    LidarParams,
    Stamp,
    Duration,
)
from evalio.datasets.base import DatasetIterator, Measurement
from rosbags.highlevel import AnyReader
from tabulate import tabulate
import numpy as np
from dataclasses import dataclass
from enum import StrEnum, auto
from evalio.types import Trajectory, SE3, SO3
import csv


# ------------------------- Simple csv loaders ------------------------- #
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


# ------------------------- Iterator over a rosbag ------------------------- #
# Various properties that a pointcloud may have - we iterate over them
class LidarStamp(StrEnum):
    Start = auto()
    End = auto()


class LidarPointStamp(StrEnum):
    Guess = auto()
    Start = auto()
    End = auto()


class LidarMajor(StrEnum):
    Guess = auto()
    Row = auto()
    Column = auto()


class LidarDensity(StrEnum):
    Guess = auto()
    AllPoints = auto()
    OnlyValidPoints = auto()


@dataclass
class LidarFormatParams:
    stamp: LidarStamp = LidarStamp.Start
    point_stamp: LidarPointStamp = LidarPointStamp.Guess
    major: LidarMajor = LidarMajor.Guess
    density: LidarDensity = LidarDensity.Guess


class RosbagIter(DatasetIterator):
    def __init__(
        self,
        path: Path,
        lidar_topic: str,
        imu_topic: str,
        lidar_params: LidarParams,
        # for mcap files, we point at the directory, not the file
        is_mcap: bool = False,
        # Reduce compute by telling the iterator how to format the pointcloud
        lidar_format: Optional[LidarFormatParams] = None,
        custom_col_func: Optional[Callable[[LidarMeasurement], None]] = None,
    ):
        self.lidar_topic = lidar_topic
        self.imu_topic = imu_topic
        self.lidar_params = lidar_params
        if lidar_format is None:
            self.lidar_format = LidarFormatParams()
        else:
            self.lidar_format = lidar_format
        self.custom_col_func = custom_col_func

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
        self.connections_lidar = [
            x for x in self.reader.connections if x.topic == self.lidar_topic
        ]
        self.connections_imu = [
            x for x in self.reader.connections if x.topic == self.imu_topic
        ]

        if len(self.connections_imu) == 0 or len(self.connections_lidar) == 0:
            connections_all = [[c.topic, c.msgtype] for c in self.reader.connections]
            print(
                tabulate(
                    connections_all, headers=["Topic", "MsgType"], tablefmt="fancy_grid"
                )
            )
            if len(self.connections_imu) == 0:
                raise ValueError(f"Could not find topic {self.imu_topic}")
            if len(self.connections_lidar) == 0:
                raise ValueError(f"Could not find topic {self.lidar_topic}")

        self.lidar_count = sum(
            [x.msgcount for x in self.connections_lidar if x.topic == self.lidar_topic]
        )

    def __len__(self):
        return self.lidar_count

    # ------------------------- Iterators ------------------------- #
    def __iter__(self):
        iterator = self.reader.messages(
            connections=self.connections_lidar + self.connections_imu
        )

        for connection, timestamp, rawdata in iterator:
            msg = self.reader.deserialize(rawdata, connection.msgtype)
            if connection.msgtype == "sensor_msgs/msg/PointCloud2":
                yield self._lidar_conversion(msg)
            elif connection.msgtype == "sensor_msgs/msg/Imu":
                yield self._imu_conversion(msg)
            else:
                raise ValueError(f"Unknown message type {connection.msgtype}")

    def imu_iter(self) -> Iterator[ImuMeasurement]:
        iterator = self.reader.messages(connections=self.connections_imu)

        for connection, timestamp, rawdata in iterator:
            msg = self.reader.deserialize(rawdata, connection.msgtype)
            yield self._imu_conversion(msg)

    def lidar_iter(self) -> Iterator[LidarMeasurement]:
        iterator = self.reader.messages(connections=self.connections_lidar)

        for connection, timestamp, rawdata in iterator:
            msg = self.reader.deserialize(rawdata, connection.msgtype)
            yield self._lidar_conversion(msg)

    # ------------------------- Convertors ------------------------- #
    def _imu_conversion(self, msg: Any) -> ImuMeasurement:
        acc = msg.linear_acceleration
        acc = np.array([acc.x, acc.y, acc.z])
        gyro = msg.angular_velocity
        gyro = np.array([gyro.x, gyro.y, gyro.z])

        stamp = Stamp(sec=msg.header.stamp.sec, nsec=msg.header.stamp.nanosec)
        return ImuMeasurement(stamp, gyro, acc)

    def _lidar_conversion(self, msg: Any) -> LidarMeasurement:
        # ------------------------- Convert to our type ------------------------- #
        fields = []
        for f in msg.fields:
            fields.append(
                Field(name=f.name, datatype=DataType(f.datatype), offset=f.offset)
            )

        stamp = Stamp(sec=msg.header.stamp.sec, nsec=msg.header.stamp.nanosec)

        # Adjust the stamp to the start of the scan
        # Do this early so we can use the stamp for the rest of the conversion
        match self.lidar_format.stamp:
            case LidarStamp.Start:
                pass
            case LidarStamp.End:
                stamp = stamp - self.lidar_params.delta_time()

        cloud = PointCloudMetadata(
            stamp=stamp,
            height=msg.height,
            width=msg.width,
            row_step=msg.row_step,
            point_step=msg.point_step,
            is_bigendian=msg.is_bigendian,
            is_dense=msg.is_dense,
        )
        scan: LidarMeasurement = pc2_to_evalio(cloud, fields, bytes(msg.data))  # type:ignore

        # ------------------------- Handle formatting properly ------------------------- #
        # For the ones that have been guessed, use heuristics to figure out format
        # Will only be ran on the first cloud, afterwords it will be set
        # row major
        if self.lidar_format.major == LidarMajor.Guess:
            if scan.points[0].row == scan.points[1].row:
                self.lidar_format.major = LidarMajor.Row
            else:
                self.lidar_format.major = LidarMajor.Column
        # density
        if self.lidar_format.density == LidarDensity.Guess:
            if (
                len(scan.points)
                == self.lidar_params.num_rows * self.lidar_params.num_columns
            ):
                self.lidar_format.density = LidarDensity.AllPoints
            else:
                self.lidar_format.density = LidarDensity.OnlyValidPoints
        # point stamp
        if self.lidar_format.point_stamp == LidarPointStamp.Guess:
            # Leave a little fudge room just in case
            # 2000ns = 0.002ms
            min_time = min(scan.points, key=lambda x: x.t).t
            if min_time < Duration.from_nsec(-2000):
                self.lidar_format.point_stamp = LidarPointStamp.End
            else:
                self.lidar_format.point_stamp = LidarPointStamp.Start

        if (
            self.lidar_format.major == LidarMajor.Row
            and self.lidar_format.density == LidarDensity.OnlyValidPoints
        ):
            print(
                "WARNING: Loading row major scan with only valid points. Can't identify where missing points should go, putting at end of scanline"
            )

        # Begin standardizing the pointcloud

        # Make point stamps relative to the start of the scan
        match self.lidar_format.point_stamp:
            case LidarPointStamp.Start:
                pass
            case LidarPointStamp.End:
                shift_point_stamps(scan, self.lidar_params.delta_time())

        # Add column indices
        if self.custom_col_func is not None:
            self.custom_col_func(scan)
        else:
            match self.lidar_format.major:
                case LidarMajor.Row:
                    fill_col_row_major(scan)
                case LidarMajor.Column:
                    fill_col_col_major(scan)

        # Reorder the points into row major with invalid points in the correct spots
        if (
            self.lidar_format.major == LidarMajor.Row
            and self.lidar_format.density == LidarDensity.AllPoints
        ):
            pass
        else:
            reorder_points(
                scan, self.lidar_params.num_rows, self.lidar_params.num_columns
            )

        return scan


# ------------------------- Flexible Iterator for Anything ------------------------- #
class RawDataIter(DatasetIterator):
    T = TypeVar("T", ImuMeasurement, LidarMeasurement)

    def __init__(
        self,
        iter_lidar: Iterator[LidarMeasurement],
        iter_imu: Iterator[ImuMeasurement],
        num_lidar: int,
    ):
        self.iter_lidar = iter_lidar
        self.iter_imu = iter_imu
        self.num_lidar = num_lidar
        # These hold the current values for iteration to compare stamps on what should be returned
        self.next_lidar = None
        self.next_imu = None

    def imu_iter(self) -> Iterator[ImuMeasurement]:
        return self.iter_imu

    def lidar_iter(self) -> Iterator[LidarMeasurement]:
        return self.iter_lidar

    def __len__(self) -> int:
        return self.num_lidar

    @staticmethod
    def _step(iter: Iterator[T]) -> Optional[T]:
        try:
            return next(iter)
        except StopIteration:
            return None

    def __iter__(self) -> Iterator[Measurement]:
        self.next_imu = next(self.iter_imu)
        self.next_lidar = next(self.iter_lidar)
        return self

    def __next__(self) -> Measurement:
        # fmt: off
        match (self.next_imu, self.next_lidar):
            case (None, None):
                raise StopIteration
            case (None, _):
                to_return, self.next_lidar = self.next_lidar, self._step(self.iter_lidar)
                return to_return
            case (_, None):
                to_return, self.next_imu = self.next_imu, self._step(self.iter_imu)
                return to_return
            case (imu, lidar):
                if imu.stamp < lidar.stamp:
                    to_return, self.next_imu = self.next_imu, self._step(self.iter_imu)
                    return to_return
                else:
                    to_return, self.next_lidar = self.next_lidar, self._step(self.iter_lidar)
                    return to_return

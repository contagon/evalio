from collections.abc import Iterator
from typing import Callable, Any, Optional
from pathlib import Path

from evalio._cpp._helpers import (  # type: ignore
    DataType,
    Field,
    PointCloudMetadata,
    pc2_to_evalio,
    fill_col_col_major,
    fill_col_row_major,
    reorder_points,
)
from evalio.types import (
    ImuMeasurement,
    LidarMeasurement,
    LidarParams,
    Stamp,
)
from evalio.datasets.base import DatasetIterator, Measurement
from rosbags.highlevel import AnyReader
from tabulate import tabulate
import numpy as np
from dataclasses import dataclass
from enum import StrEnum, auto


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
        lidar_format: LidarFormatParams = LidarFormatParams(),
        custom_col_func: Optional[Callable[[LidarMeasurement], None]] = None,
    ):
        self.lidar_topic = lidar_topic
        self.imu_topic = imu_topic
        self.lidar_params = lidar_params
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

    # TODO: Formalize the lengths somehow?
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
        if self.lidar_format.major == LidarMajor.Guess:
            if scan.points[0].row == scan.points[1].row:
                self.lidar_format.major = LidarMajor.Row
            else:
                self.lidar_format.major = LidarMajor.Column

        if self.lidar_format.density == LidarDensity.Guess:
            if (
                len(scan.points)
                == self.lidar_params.num_rows * self.lidar_params.num_columns
            ):
                self.lidar_format.density = LidarDensity.AllPoints
            else:
                self.lidar_format.density = LidarDensity.OnlyValidPoints

        if self.lidar_format.point_stamp == LidarPointStamp.Guess:
            pass

        # Adjust the stamp to the start of the scan
        match self.lidar_format.stamp:
            case LidarStamp.Start:
                pass
            case LidarStamp.End:
                scan.stamp = Stamp.from_sec(
                    scan.stamp.to_sec() - 1.0 / self.lidar_params.rate
                )

        if (
            self.lidar_format.major == LidarMajor.Row
            and self.lidar_format.density == LidarDensity.OnlyValidPoints
        ):
            print(
                "WARNING: Loading row major scan with only valid points. Can't identify where missing points should go, putting at end of scanline"
            )

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

        # match self.lidar_format.point_stamp:

        return scan


# ------------------------- Flexible Iterator for Anything ------------------------- #
class RawDataIter(DatasetIterator):
    def __init__(
        self,
        stamps_lidar: list[Stamp],
        iterator_lidar: Iterator[LidarMeasurement],
        stamps_imu: list[Stamp],
        iterator_imu: Iterator[ImuMeasurement],
    ):
        # TODO: Probably a clever way to do this that doesn't require the stamps or indices
        # Need to make the iterators peekable
        self.stamps_lidar = stamps_lidar
        self.stamps_imu = stamps_imu
        self.idx_lidar = 0
        self.idx_imu = 0
        self.iterator_lidar = iterator_lidar
        self.iterator_imu = iterator_imu

    def imu_iter(self) -> Iterator[ImuMeasurement]:
        return self.iterator_imu

    def lidar_iter(self) -> Iterator[LidarMeasurement]:
        return self.iterator_lidar

    def __iter__(self) -> Iterator[Measurement]:
        if self.stamps_imu[self.idx_imu] < self.stamps_lidar[self.idx_lidar]:
            self.idx_imu += 1
            yield next(self.iterator_imu)
        else:
            self.idx_lidar += 1
            yield next(self.iterator_lidar)

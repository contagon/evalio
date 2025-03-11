from collections.abc import Iterator
from typing import Iterable, Protocol, Union, Any
from pathlib import Path

from evalio._cpp._helpers import (  # type: ignore
    DataType,
    Field,
    PointCloudMetadata,
    general_pc2_to_evalio,
)
from evalio.types import (
    ImuMeasurement,
    LidarMeasurement,
    LidarParams,
    Stamp,
)
from rosbags.highlevel import AnyReader
from tabulate import tabulate
import numpy as np

Measurement = Union[ImuMeasurement, LidarMeasurement]


class DatasetIterator(Iterable[Measurement], Protocol):
    def imu_iter(self) -> Iterator[ImuMeasurement]: ...

    def lidar_iter(self) -> Iterator[LidarMeasurement]: ...

    def __iter__(self) -> Iterator[Measurement]: ...


# ------------------------- Iterator over a rosbag ------------------------- #
def pointcloud2_to_evalio(
    msg, params: LidarParams, cpp_point_loader=general_pc2_to_evalio
) -> LidarMeasurement:
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

    return cpp_point_loader(cloud, fields, bytes(msg.data), params)  # type: ignore


def imu_to_evalio(msg) -> ImuMeasurement:
    acc = msg.linear_acceleration
    acc = np.array([acc.x, acc.y, acc.z])
    gyro = msg.angular_velocity
    gyro = np.array([gyro.x, gyro.y, gyro.z])

    stamp = Stamp(sec=msg.header.stamp.sec, nsec=msg.header.stamp.nanosec)
    return ImuMeasurement(stamp, gyro, acc)


class RosbagIter(DatasetIterator):
    def __init__(
        self,
        path: Path,
        lidar_topic: str,
        imu_topic: str,
        params: LidarParams,
        # for mcap files, we point at the directory, not the file
        is_mcap: bool = False,
        cpp_point_loader=general_pc2_to_evalio,
    ):
        self.lidar_topic = lidar_topic
        self.imu_topic = imu_topic
        self.params = params
        self.cpp_point_loader = cpp_point_loader

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

    def __iter__(self):
        iterator = self.reader.messages(
            connections=self.connections_lidar + self.connections_imu
        )

        for connection, timestamp, rawdata in iterator:
            msg = self.reader.deserialize(rawdata, connection.msgtype)
            if connection.msgtype == "sensor_msgs/msg/PointCloud2":
                yield pointcloud2_to_evalio(msg, self.params, self.cpp_point_loader)
            elif connection.msgtype == "sensor_msgs/msg/Imu":
                yield imu_to_evalio(msg)
            else:
                raise ValueError(f"Unknown message type {connection.msgtype}")

    def imu_iter(self) -> Iterator[ImuMeasurement]:
        iterator = self.reader.messages(connections=self.connections_imu)

        for connection, timestamp, rawdata in iterator:
            msg = self.reader.deserialize(rawdata, connection.msgtype)
            yield imu_to_evalio(msg)

    def lidar_iter(self) -> Iterator[LidarMeasurement]:
        iterator = self.reader.messages(connections=self.connections_lidar)

        for connection, timestamp, rawdata in iterator:
            msg = self.reader.deserialize(rawdata, connection.msgtype)
            yield pointcloud2_to_evalio(msg, self.params, self.cpp_point_loader)

    @staticmethod
    def _imu_converstion(msg: Any) -> ImuMeasurement:
        acc = msg.linear_acceleration
        acc = np.array([acc.x, acc.y, acc.z])
        gyro = msg.angular_velocity
        gyro = np.array([gyro.x, gyro.y, gyro.z])

        stamp = Stamp(sec=msg.header.stamp.sec, nsec=msg.header.stamp.nanosec)
        return ImuMeasurement(stamp, gyro, acc)


# ------------------------- Flexible Iterator for Anything ------------------------- #
class RawDataIter(DatasetIterator):
    def __init__(
        self,
        stamps_lidar: list[Stamp],
        iterator_lidar: Iterator[LidarMeasurement],
        stamps_imu: list[Stamp],
        iterator_imu: Iterator[ImuMeasurement],
    ):
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

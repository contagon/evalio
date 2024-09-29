from typing import Protocol, Union
from rosbags.highlevel import AnyReader
from dataclasses import dataclass

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
    Point,
    Field as FieldCpp,
    DataType,
    PointCloud2,
    pointcloud2_to_evalio as pointcloud2_to_evalio_cpp,
)

import time

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
@dataclass
class Field:
    name: str
    offset: int
    data_width: int
    data_type: type


WIDTHS = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}


def pointcloud2_to_evalio(msg) -> LidarMeasurement:
    # Convert to C++ types
    # help(DataType)
    fields = []
    for f in msg.fields:
        fields.append(
            FieldCpp(name=f.name, datatype=DataType(f.datatype), offset=f.offset)
        )

    stamp = Stamp(sec=msg.header.stamp.sec, nsec=msg.header.stamp.nanosec)

    t0 = time.time()
    cloud = PointCloud2(
        fields=fields,
        data=msg.data,
        stamp=stamp,
        height=msg.height,
        width=msg.width,
        row_step=msg.row_step,
        point_step=msg.point_step,
        is_bigendian=msg.is_bigendian,
        is_dense=msg.is_dense,
    )
    t1 = time.time()
    print("Conversion Time:", t1 - t0)

    return pointcloud2_to_evalio_cpp(cloud)


def pointcloud2_to_evalio_py(msg) -> LidarMeasurement:
    # Parse fields to get the correct fields
    fields = []
    for f in msg.fields:
        name = f.name
        if name in ["x", "y", "z", "intensity", "t", "range", "ring"]:
            if name == "ring":
                name = "row"
            dtype = WIDTHS[f.datatype]
            fields.append(Field(name, f.offset, dtype().nbytes, dtype))

    # Parse the data
    # TODO: Endianess??
    # TODO: This is REALLY slow, need to speed it up
    points = []
    for row in range(0, msg.height):  # over each column
        i = row * msg.row_step
        for col in range(0, msg.width):  # then each row
            point = Point(col=col)
            j = i + col * msg.point_step
            for f in fields:
                d = np.frombuffer(
                    msg.data[j + f.offset : j + f.offset + f.data_width],
                    dtype=f.data_type,
                )[0]
                setattr(point, f.name, d)
            points.append(point)

    stamp = Stamp(sec=msg.header.stamp.sec, nsec=msg.header.stamp.nanosec)
    return LidarMeasurement(stamp, points)


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
            # TODO: Remove this
            self.path = [self.path[0]]
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
            import time

            t0 = time.time()
            res = pointcloud2_to_evalio(msg)
            t1 = time.time()
            print("Lidar Time:", t1 - t0)
            return res
        elif connection.msgtype == "sensor_msgs/msg/Imu":
            import time

            t0 = time.time()
            res = imu_to_evalio(msg)
            t1 = time.time()
            print("Imu Time:", t1 - t0)
            return res
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

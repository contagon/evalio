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
from rosbags.typesys import Stores, get_typestore
import numpy as np
from dataclasses import dataclass
from enum import StrEnum, auto
from evalio.utils import print_warning

from rich.table import Table
from rich.console import Console
from rich import box


# ------------------------- Iterator over a rosbag ------------------------- #
# Various properties that a pointcloud may have - we iterate over them
# TODO: Nest these into LidarFormatParams or something
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
    """An iterator for loading from rosbag files.

    This is a wrapper around the rosbags library, with some niceties for converting ros PointCloud2 messages to a standardized format.
    Has identical methods to [DatasetIterator][evalio.datasets.DatasetIterator].
    """

    def __init__(
        self,
        path: Path,
        lidar_topic: str,
        imu_topic: str,
        lidar_params: LidarParams,
        type_store: Optional[Stores] = None,
        # Reduce compute by telling the iterator how to format the pointcloud
        lidar_format: Optional[LidarFormatParams] = None,
        custom_col_func: Optional[Callable[[LidarMeasurement], None]] = None,
    ):
        """
        Args:
            path (Path): Location of rosbag file(s) or dir(s). If a directory is passed containing multiple bags (ros1 or ros2), all will be loaded.
            lidar_topic (str): Name of lidar topic.
            imu_topic (str): Name of imu topic.
            lidar_params (LidarParams): Lidar parameters, can be gotten from [lidar_params][evalio.datasets.Dataset.lidar_params].
            type_store (Optional[Stores], optional): Additional type store to be loaded into rosbags. Defaults to None.
            lidar_format (Optional[LidarFormatParams], optional): Various parameters for how lidar data is stored. If not specified, most will try to be inferred. We strongly recommend setting this to ensure data is standardized properly. Defaults to None.
            custom_col_func (Optional[Callable[[LidarMeasurement], None]], optional): Function to put the point cloud in row major format. Will generally not be needed, except for strange default orderings. Defaults to None.

        Raises:
            FileNotFoundError: If the path is a directory and no .bag files are found.
            ValueError: If the lidar or imu topic is not found in the bag file.
        """
        self.lidar_topic = lidar_topic
        self.imu_topic = imu_topic
        self.lidar_params = lidar_params
        if lidar_format is None:
            self.lidar_format = LidarFormatParams()
        else:
            self.lidar_format = lidar_format
        self.custom_col_func = custom_col_func

        # Find all bags (may be either ros1 .bag files or ros2 bag/ dirs)
        if path.is_file():
            # Provide path is a ros1 .bag file
            self.path = [path]
        else:

            def is_ros2_bag(d):
                return bool(list(d.glob("*.mcap")) + list(d.glob("*.db3")))

            # Path provided is a directory may be ros2 bag/ dir or contain multiple bags
            ros1_bag_file_list = [p for p in path.glob("*.bag")]
            ros2_bag_dir_list = [d for d in path.glob("*/") if is_ros2_bag(d)]

            if ros1_bag_file_list:  # path contains ros1 .bag files
                self.path = ros1_bag_file_list
            elif ros2_bag_dir_list:  # path contains ros2 bag/ directories
                self.path = ros2_bag_dir_list
            elif is_ros2_bag(path):  # path is a ros2 bag/ (contains mcap or db3 file)
                self.path = [path]
            else:
                raise ValueError(
                    f"Invalid rosbag path: {path}\nExpected path to be one of:\na) ros1 .bag \nb) ros2 bag/ dir or \nc) directory multiple a or b"
                )

        # Open the bag file
        self.reader = AnyReader(self.path)
        self.reader.open()

        # force load passed in type store
        # there is a default_typestore parameter in AnyReader, but it won't be used if one of bags has msgdefs
        # this works around that, but may be unstable
        # https://gitlab.com/ternaris/rosbags/-/blob/master/src/rosbags/highlevel/anyreader.py?ref_type=heads#L125-140
        if type_store is not None:
            self.reader.typestore.register(get_typestore(type_store).fielddefs)

        self.connections_lidar = [
            x for x in self.reader.connections if x.topic == self.lidar_topic
        ]
        self.connections_imu = [
            x for x in self.reader.connections if x.topic == self.imu_topic
        ]

        if len(self.connections_imu) == 0 or len(self.connections_lidar) == 0:
            table = Table(title="Rosbag Connections", highlight=True, box=box.ROUNDED)
            table.add_column("Topic", justify="right")
            table.add_column("MsgType", justify="left")
            for c in self.reader.connections:
                table.add_row(c.topic, c.msgtype)
            Console().print(table)

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
            print_warning(
                "Loading row major scan with only valid points. Can't identify where missing points should go, putting at end of scanline"
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
    """An iterator for loading from python iterables.

    Interleaves imu and lidar iterables. Allows for arbitrary data to be loaded and presented in a consistent manner for the base [Dataset][evalio.datasets.Dataset] class.
    Has identical methods to [DatasetIterator][evalio.datasets.DatasetIterator].
    """

    T = TypeVar("T", ImuMeasurement, LidarMeasurement)

    def __init__(
        self,
        iter_lidar: Iterator[LidarMeasurement],
        iter_imu: Iterator[ImuMeasurement],
        num_lidar: int,
    ):
        """
        Args:
            iter_lidar (Iterator[LidarMeasurement]): An iterator over LidarMeasurement
            iter_imu (Iterator[ImuMeasurement]): An iterator over ImuMeasurement
            num_lidar (int): The number of lidar measurements

        ``` py
        from evalio.datasets.loaders import RawDataIter
        from evalio.types import ImuMeasurement, LidarMeasurement, Stamp
        import numpy as np

        # Create some fake data
        imu_iter = (
            ImuMeasurement(Stamp.from_sec(i), np.zeros(3), np.zeros(3))
            for i in range(10)
        )
        lidar_iter = (LidarMeasurement(Stamp.from_sec(i + 0.1)) for i in range(10))

        # Create the iterator
        rawdata = RawDataIter(imu_iter, lidar_iter, 10)
        ```
        """
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

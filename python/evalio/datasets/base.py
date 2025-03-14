import csv
import os
from pathlib import Path
from typing import Iterable, Iterator, Optional, Union
from itertools import islice

from enum import StrEnum, auto, Enum

import numpy as np
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

if os.getenv("EVALIO_DATA") is None:
    print(
        "Warning: EVALIO_DATA environment variable is not set. Using default './data'"
    )
EVALIO_DATA = Path(os.getenv("EVALIO_DATA", "./data"))

Measurement = Union[ImuMeasurement, LidarMeasurement]


class DatasetIterator(Iterable[Measurement]):
    def imu_iter(self) -> Iterator[ImuMeasurement]: ...

    def lidar_iter(self) -> Iterator[LidarMeasurement]: ...

    def __iter__(self) -> Iterator[Measurement]: ...


class Dataset(StrEnum):
    # ------------------------- For loading data ------------------------- #
    def data_iter(self) -> DatasetIterator: ...

    # Return the ground truth in the ground truth frame
    def ground_truth_raw(self) -> Trajectory: ...

    # ------------------------- For loading params ------------------------- #
    @staticmethod
    def url() -> str: ...

    def imu_T_lidar(self) -> SE3: ...

    def imu_T_gt(self) -> SE3: ...

    def imu_params(self) -> ImuParams: ...

    def lidar_params(self) -> LidarParams: ...

    def files(self) -> list[str]: ...

    # ------------------------- Optional overrides ------------------------- #
    # Optional method
    def download(self) -> None:
        raise NotImplementedError("Download not implemented")

    # TODO: This would match better as a "classproperty", but not will involve some work
    @classmethod
    def dataset_name(cls) -> str:
        return pascal_to_snake(cls.__name__)

    # ------------------------- Helpers that wrap the above ------------------------- #
    def is_downloaded(self) -> bool:
        for f in self.files():
            if not (EVALIO_DATA / self / f).exists():
                return False

        return True

    def ground_truth(self) -> Trajectory:
        gt_traj = self.ground_truth_raw()
        gt_T_imu = self.imu_T_gt().inverse()

        # Convert to IMU frame
        for i in range(len(gt_traj)):
            gt_o_T_gt_i = gt_traj.poses[i]
            gt_traj.poses[i] = gt_o_T_gt_i * gt_T_imu

        return gt_traj

    def get_one_lidar(self, idx: int = 0) -> LidarMeasurement:
        return next(islice(self.lidar(), idx, idx + 1))

    def get_one_imu(self, idx: int = 0) -> ImuMeasurement:
        return next(islice(self.imu(), idx, idx + 1))

    def _fail_not_downloaded(self):
        if not self.is_downloaded():
            raise ValueError(
                f"Data for {self} not found, please use `evalio download {self}` to download"
            )

    def __iter__(self) -> Iterator[Measurement]:  # type: ignore
        self._fail_not_downloaded()
        return self.data_iter().__iter__()

    def imu(self) -> Iterable[ImuMeasurement]:
        self._fail_not_downloaded()
        return self.data_iter().imu_iter()

    def lidar(self) -> Iterable[LidarMeasurement]:
        self._fail_not_downloaded()
        return self.data_iter().lidar_iter()

    # ------------------------- Misc name helpers ------------------------- #
    def __str__(self):
        return self.full_name

    @property
    def seq_name(self) -> str:
        return self.value

    @property
    def full_name(self) -> str:
        return f"{self.dataset_name()}/{self.seq_name}"

    @classmethod
    def sequences(cls) -> list["Dataset"]:
        return list(cls.__members__.values())

    @property
    def folder(self) -> Path:
        return EVALIO_DATA / self.full_name


class CharKinds(Enum):
    LOWER = auto()
    UPPER = auto()
    DIGIT = auto()
    OTHER = auto()

    @staticmethod
    def from_char(char: str):
        if char.islower():
            return CharKinds.LOWER
        if char.isupper():
            return CharKinds.UPPER
        if char.isdigit():
            return CharKinds.DIGIT
        return CharKinds.OTHER


def pascal_to_snake(identifier):
    # only split when going from lower to something else
    splits = []
    last_kind = CharKinds.from_char(identifier[0])
    for i, char in enumerate(identifier[1:], start=1):
        kind = CharKinds.from_char(char)
        if last_kind == CharKinds.LOWER and kind != CharKinds.LOWER:
            splits.append(i)
        last_kind = kind

    parts = [identifier[i:j] for i, j in zip([0] + splits, splits + [None])]
    return "_".join(parts).lower()


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

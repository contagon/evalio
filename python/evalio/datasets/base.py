import csv
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, Optional, Protocol, Union

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


class DatasetIterator(Iterable[Measurement], Protocol):
    def imu_iter(self) -> Iterator[ImuMeasurement]: ...

    def lidar_iter(self) -> Iterator[LidarMeasurement]: ...

    def __iter__(self) -> Iterator[Measurement]: ...


@dataclass
class Dataset(Protocol):
    seq: str
    length: Optional[int] = None

    # ------------------------- For loading data ------------------------- #
    def data_iter(self) -> DatasetIterator: ...

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

    def first_n_lidar_scans(self, n: int = 1) -> list[LidarMeasurement]:
        scans = []
        for m in self.lidar_iter():
            scans.append(m)
            if len(scans) == n:
                return scans

        raise ValueError("No lidar scans found")

    def first_n_imu_measurement(self, n: int = 1) -> list[ImuMeasurement]:
        imu = []
        for m in self.imu_iter():
            imu.append(m)
            if len(imu) == n:
                return imu

        raise ValueError("No imu mm found")

    def __str__(self):
        return f"{self.name()}/{self.seq}"

    def __iter__(self) -> Iterator[Measurement]:
        return self.data_iter().__iter__()

    def imu_iter(self) -> Iterable[ImuMeasurement]:
        return self.data_iter().imu_iter()

    def lidar_iter(self) -> Iterable[LidarMeasurement]:
        return self.data_iter().lidar_iter()


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

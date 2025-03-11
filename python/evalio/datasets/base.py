import csv
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, Optional, Protocol

import numpy as np
from evalio._cpp._helpers import (  # type: ignore
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
from evalio.datasets.iterators import DatasetIterator, Measurement, RosbagIter  # noqa: F401

if os.getenv("EVALIO_DATA") is None:
    print(
        "Warning: EVALIO_DATA environment variable is not set. Using default './data'"
    )
EVALIO_DATA = Path(os.getenv("EVALIO_DATA", "./data"))


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


# ------------------------- Helpers ------------------------- #
class RawDataIter:
    # def __init__(
    #     self,
    #     lidar_stamps: list[Stamp],
    #     lidar_iterator: Generator[LidarMeasurement, None, None],
    #     imu_stamps: list[Stamp],
    #     imu_iterator: Generator[ImuMeasurement, None, None],
    # ):
    #     self.lidar_stamps = lidar_stamps
    #     self.imu_stamps = imu_stamps
    #     self.idx_lidar = 0
    #     self.idx_imu = 0

    def __init__(self, lidar_path: Path, imu_file: Path, lidar_params: LidarParams):
        self.lidar_path = lidar_path
        self.imu_file = imu_file

        # Load all IMU data
        imu_stamps = np.loadtxt(imu_file, usecols=0, dtype=np.int64, delimiter=",")
        self.imu_stamps = [Stamp.from_nsec(x) for x in imu_stamps]
        imu_data = np.loadtxt(imu_file, usecols=(11, 12, 13, 14, 15, 16), delimiter=",")
        self.imu_gyro = imu_data[:, 3:]
        self.imu_acc = imu_data[:, :3]

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
            # short circuit if we don't need lidar scans (ie for bias generation)
            # return LidarMeasurement(stamp, [])
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

from pathlib import Path
from typing import Iterable, Iterator, Union
from itertools import islice

import os
from enum import StrEnum, auto, Enum

from evalio.types import (
    SE3,
    ImuMeasurement,
    ImuParams,
    LidarMeasurement,
    LidarParams,
    Trajectory,
)

from rich import print

Measurement = Union[ImuMeasurement, LidarMeasurement]

_DATA_DIR = Path(os.environ.get("EVALIO_DATA", "evalio_data"))
_WARNED = False


class DatasetIterator(Iterable[Measurement]):
    def imu_iter(self) -> Iterator[ImuMeasurement]: ...

    def lidar_iter(self) -> Iterator[LidarMeasurement]: ...

    def __iter__(self) -> Iterator[Measurement]: ...

    # Return the number of lidar scans
    def __len__(self) -> int: ...


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
        self._warn_default_dir()
        for f in self.files():
            if not (self.folder / f).exists():
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

    def _fail_not_downloaded(self):
        if not self.is_downloaded():
            # TODO: Make this print with rich?
            raise ValueError(
                f"Data for {self} not found, please use `evalio download {self}` to download"
            )

    @classmethod
    def _warn_default_dir(cls):
        global _DATA_DIR, _WARNED
        if not _WARNED and _DATA_DIR == Path("./evalio_data"):
            print(
                "[bold red]Warning[/bold red]: Data directory has not been set. Using default './evalio_data'"
            )
            _WARNED = True

    # ------------------------- Helpers that leverage from the iterator ------------------------- #

    def __len__(self) -> int:
        return self.data_iter().__len__()

    def __iter__(self) -> Iterator[Measurement]:  # type: ignore
        self._fail_not_downloaded()
        return self.data_iter().__iter__()

    def imu(self) -> Iterable[ImuMeasurement]:
        self._fail_not_downloaded()
        return self.data_iter().imu_iter()

    def lidar(self) -> Iterable[LidarMeasurement]:
        self._fail_not_downloaded()
        return self.data_iter().lidar_iter()

    def get_one_lidar(self, idx: int = 0) -> LidarMeasurement:
        return next(islice(self.lidar(), idx, idx + 1))

    def get_one_imu(self, idx: int = 0) -> ImuMeasurement:
        return next(islice(self.imu(), idx, idx + 1))

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
        return _DATA_DIR / self.full_name


# For converting dataset names to snake case
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


# ------------------------- Helpers ------------------------- #
def set_data_dir(directory: Path):
    """
    Set the location where datasets are stored. This will be used to store the downloaded data.
    """
    global _DATA_DIR, _WARNED
    _DATA_DIR = directory
    _WARNED = True


def get_data_dir() -> Path:
    """
    Get the data directory for the dataset. This will be used to store the downloaded data.
    """
    global _DATA_DIR
    return _DATA_DIR

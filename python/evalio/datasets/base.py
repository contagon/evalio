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

from evalio.utils import print_warning

Measurement = Union[ImuMeasurement, LidarMeasurement]

_DATA_DIR = Path(os.environ.get("EVALIO_DATA", "evalio_data"))
_WARNED = False


class DatasetIterator(Iterable[Measurement]):
    def imu_iter(self) -> Iterator[ImuMeasurement]:
        """Main interface for iterating over IMU measurements.

        Yields:
            Iterator[ImuMeasurement]: Iterator of IMU measurements.
        """
        ...

    def lidar_iter(self) -> Iterator[LidarMeasurement]:
        """Main interface for iterating over Lidar measurements.

        Yields:
            Iterator[LidarMeasurement]: Iterator of Lidar measurements.
        """
        ...

    def __iter__(self) -> Iterator[Measurement]:
        """Main interface for iterating over all measurements.

        Yields:
            Iterator[Measurement]: Iterator of all measurements (IMU and Lidar).
        """

        ...

    # Return the number of lidar scans
    def __len__(self) -> int: ...


class Dataset(StrEnum):
    # ------------------------- For loading data ------------------------- #
    def data_iter(self) -> DatasetIterator:
        """
        Provides an iterator over the dataset's measurements.

        Returns:
            DatasetIterator: An iterator that yields measurements from the dataset.
        """
        ...

    # Return the ground truth in the ground truth frame
    def ground_truth_raw(self) -> Trajectory:
        """
        Retrieves the raw ground truth trajectory, as represented in the ground truth frame.

        Returns:
            Trajectory: The raw ground truth trajectory data.
        """
        ...

    # ------------------------- For loading params ------------------------- #
    @staticmethod
    def url() -> str:
        """Webpage with the dataset information.

        Returns:
            str: URL of the dataset webpage.
        """
        ...

    def imu_T_lidar(self) -> SE3:
        """Returns the transformation from IMU to Lidar frame.

        Returns:
            SE3: Transformation from IMU to Lidar frame.
        """
        ...

    def imu_T_gt(self) -> SE3:
        """Retrieves the transformation from IMU to ground truth frame.

        Returns:
            SE3: Transformation from IMU to ground truth frame.
        """
        ...

    def imu_params(self) -> ImuParams:
        """Specifies the parameters of the IMU.

        Returns:
            ImuParams: Parameters of the IMU.
        """
        ...

    def lidar_params(self) -> LidarParams:
        """Specifies the parameters of the Lidar.

        Returns:
            LidarParams: Parameters of the Lidar.
        """
        ...

    def files(self) -> list[str | Path]:
        """Return list of files required to run this dataset.

        If a returned type is a Path, it will be checked as is. If it is a string, it will be prepended with [folder][evalio.datasets.Dataset.folder].

        Returns:
            list[str]: _description_
        """
        ...

    # ------------------------- Optional overrides ------------------------- #
    # Optional method
    def download(self) -> None:
        """Method to download the dataset.

        Completely optional to implement, although most datasets do.

        Raises:
            NotImplementedError: If not implemented.
        """
        raise NotImplementedError("Download not implemented")

    # TODO: This would match better as a "classproperty", but not will involve some work
    @classmethod
    def dataset_name(cls) -> str:
        """Name of the dataset, in snake case.

        This is the name that will be used when parsing directly from a string. Currently is automatically generated from the class name, but can be overridden.

        Returns:
            str: _description_
        """
        return pascal_to_snake(cls.__name__)

    # ------------------------- Helpers that wrap the above ------------------------- #
    def is_downloaded(self) -> bool:
        """Verify if the dataset is downloaded.

        Returns:
            bool: True if the dataset is downloaded, False otherwise.
        """
        self._warn_default_dir()
        for f in self.files():
            if isinstance(f, str):
                if not (self.folder / f).exists():
                    return False
            else:
                if not f.exists():
                    return False

        return True

    def ground_truth(self) -> Trajectory:
        """Get the ground truth trajectory in the **IMU** frame, rather than the ground truth frame as returned in [ground_truth_raw][evalio.datasets.Dataset.ground_truth_raw].

        Returns:
            Trajectory: The ground truth trajectory in the IMU frame.
        """
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
            print_warning(
                "Using default './evalio_data' for base data directory. Override by setting [magenta]EVALIO_DATA[/magenta], [magenta]evalio.set_data_dir(path)[/magenta] in python, or [magenta]-D[/magenta] in the CLI."
            )
            _WARNED = True

    # ------------------------- Helpers that leverage from the iterator ------------------------- #

    def __len__(self) -> int:
        """Return the number of lidar scans.

        Returns:
            int: Number of lidar scans.
        """
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
        """The folder in the global dataset directory where this dataset is stored.

        Returns:
            Path: Path to the dataset folder.
        """
        global _DATA_DIR
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
    Set the global location where datasets are stored. This will be used to store the downloaded data.
    """
    global _DATA_DIR, _WARNED
    _DATA_DIR = directory
    _WARNED = True


def get_data_dir() -> Path:
    """
    Get the global data directory. This will be used to store the downloaded data.
    """
    global _DATA_DIR
    return _DATA_DIR

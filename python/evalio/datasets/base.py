import os
from enum import StrEnum
from itertools import islice
from pathlib import Path
from typing import Iterable, Iterator, Optional, Sequence, Union, cast

from evalio._cpp.types import (  # type: ignore
    SE3,
    ImuMeasurement,
    ImuParams,
    LidarMeasurement,
    LidarParams,
)

from evalio.types import GroundTruth, Trajectory

from evalio.utils import print_warning, pascal_to_snake

Measurement = Union[ImuMeasurement, LidarMeasurement]

_DATA_DIR = Path(os.environ.get("EVALIO_DATA", "evalio_data"))
_WARNED = False


class DatasetIterator(Iterable[Measurement]):
    """This is the base class for iterating over datasets.

    This class is the main interface used to iterate over the dataset's measurements.
    It provides an interface for iterating over IMU and Lidar measurements, as well as all measurements interleaved.
    This allows for standardizing access to loading data, while allowing for loading parameters in [Dataset][evalio.datasets.Dataset] without having to load the data.
    Generally, will be created by the [Dataset.data_iter][evalio.datasets.Dataset.data_iter] method.
    """

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
    def __len__(self) -> int:
        """Number of lidar scans.

        Returns:
            int: Number of lidar scans.
        """
        ...


class Dataset(StrEnum):
    """The base class for all datasets.

    This class provides an interface for loading datasets, including loading parameters and iterating over measurements.
    All datasets are string enums, where each enum member represents a trajectory sequence in the dataset.
    """

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

    def files(self) -> Sequence[str | Path]:
        """Return list of files required to run this dataset.

        If a returned type is a Path, it will be checked as is. If it is a string, it will be prepended with [folder][evalio.datasets.Dataset.folder].

        Returns:
            list[str]: _description_
        """
        ...

    # ------------------------- Optional dataset info ------------------------- #
    @staticmethod
    def url() -> str:
        """Webpage with the dataset information.

        Returns:
            str: URL of the dataset webpage.
        """
        return "-"

    def environment(self) -> str:
        """Environment where the dataset was collected.

        Returns:
            str: Environment where the dataset was collected.
        """
        return "-"

    def vehicle(self) -> str:
        """Vehicle used to collect the dataset.

        Returns:
            str: Vehicle used to collect the dataset.
        """
        return "-"

    def quick_len(self) -> Optional[int]:
        """Hardcoded number of lidar scans in the dataset, rather than computing by loading all the data (slow).

        Returns:
            Optional[int]: Number of lidar scans in the dataset. None if not available.
        """
        return None

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

    def ground_truth(self) -> Trajectory[GroundTruth]:
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

        gt_traj = cast(Trajectory[GroundTruth], gt_traj)
        gt_traj.metadata = GroundTruth(sequence=self.full_name)

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
                "Using default './evalio_data' for base data directory. Override by setting [magenta]EVALIO_DATA[/magenta], [magenta]evalio.set_DATA_DIR(path)[/magenta] in python, or [magenta]-D[/magenta] in the CLI."
            )
            _WARNED = True

    # ------------------------- Helpers that leverage from the iterator ------------------------- #

    def __len__(self) -> int:
        """Return the number of lidar scans.

        If quick_len is available, it will be used. Otherwise, it will load the entire dataset to get the length.

        Returns:
            int: Number of lidar scans.
        """
        if (length := self.quick_len()) is not None:
            return length

        self._fail_not_downloaded()
        return self.data_iter().__len__()

    def __iter__(self) -> Iterator[Measurement]:  # type: ignore
        """Main interface for iterating over measurements of all types.

        Returns:
            Iterator[Measurement]: Iterator of all measurements (IMU and Lidar).
        """
        self._fail_not_downloaded()
        return self.data_iter().__iter__()

    def imu(self) -> Iterable[ImuMeasurement]:
        """Iterate over just IMU measurements.

        Returns:
            Iterable[ImuMeasurement]: Iterator of IMU measurements.
        """
        self._fail_not_downloaded()
        return self.data_iter().imu_iter()

    def lidar(self) -> Iterable[LidarMeasurement]:
        """Iterate over just Lidar measurements.

        Returns:
            Iterable[LidarMeasurement]: Iterator of Lidar measurements.
        """
        self._fail_not_downloaded()
        return self.data_iter().lidar_iter()

    def get_one_lidar(self, idx: int = 0) -> LidarMeasurement:
        """Get a single Lidar measurement.

        Note, this can be expensive to compute, as it will iterate over the entire dataset until it finds the measurement.

        Args:
            idx (int, optional): Index of measurement to get. Defaults to 0.

        Returns:
            LidarMeasurement: The Lidar measurement at the given index.
        """
        return next(islice(self.lidar(), idx, idx + 1))

    def get_one_imu(self, idx: int = 0) -> ImuMeasurement:
        """Get a single IMU measurement.

        Note, this can be expensive to compute, as it will iterate over the entire dataset until it finds the measurement.

        Args:
            idx (int, optional): Index of measurement to get. Defaults to 0.

        Returns:
            ImuMeasurement: The IMU measurement at the given index.
        """
        return next(islice(self.imu(), idx, idx + 1))

    # ------------------------- Misc name helpers ------------------------- #
    def __str__(self):
        return self.full_name

    @property
    def seq_name(self) -> str:
        """Name of the sequence, in snake case.

        Returns:
            str: Name of the sequence.
        """
        return self.value

    @property
    def full_name(self) -> str:
        """Full name of the dataset, including the dataset name and sequence name.

        Example: "dataset_name/sequence_name"

        Returns:
            str: Full name of the dataset.
        """
        return f"{self.dataset_name()}/{self.seq_name}"

    @classmethod
    def sequences(cls) -> list["Dataset"]:
        """All sequences in the dataset.

        Returns:
            list[Dataset]: List of all sequences in the dataset.
        """
        return list(cls.__members__.values())

    @property
    def folder(self) -> Path:
        """The folder in the global dataset directory where this dataset is stored.

        Returns:
            Path: Path to the dataset folder.
        """
        global _DATA_DIR
        return _DATA_DIR / self.full_name

    def size_on_disk(self) -> Optional[float]:
        """Shows the size of the dataset on disk, in GB.

        Returns:
            Optional[float]: Size of the dataset on disk, in GB. None if the dataset is not downloaded.
        """

        if not self.is_downloaded():
            return None
        else:
            return sum(f.stat().st_size for f in self.folder.glob("**/*")) / 1e9


# ------------------------- Helpers ------------------------- #
def set_data_dir(directory: Path):
    """Set the global location where datasets are stored. This will be used to store the downloaded data.

    Args:
        directory (Path): Directory
    """
    global _DATA_DIR, _WARNED
    _DATA_DIR = directory
    _WARNED = True


def get_data_dir() -> Path:
    """Get the global data directory. This will be used to store the downloaded data.

    Returns:
        Path: Directory where datasets are stored.
    """
    global _DATA_DIR
    return _DATA_DIR

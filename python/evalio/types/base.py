"""
These are the base python-based types used throughout evalio.

They MUST not depend on anything else in evalio, or else circular imports will occur.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import csv
from _csv import Writer
from io import TextIOWrapper
from typing_extensions import TypeVar
from evalio.utils import print_warning
import yaml

from pathlib import Path
from typing import Any, ClassVar, Generic, Iterator, Optional, Self, cast

from evalio._cpp.types import (  # type: ignore
    SE3,
    Stamp,
)
from evalio._cpp.helpers import parse_csv_line  # type: ignore

from evalio.utils import pascal_to_snake

Param = bool | int | float | str
"""A parameter value for a pipeline, can be a bool, int, float, or str."""


class FailedMetadataParse(Exception):
    """Exception raised when metadata parsing fails."""

    def __init__(self, reason: str):
        super().__init__(f"Failed to parse metadata: {reason}")
        self.reason = reason


@dataclass(kw_only=True)
class Metadata:
    """Base class for metadata associated with a trajectory."""

    file: Optional[Path] = None
    """File where the metadata was loaded to and from, if any."""
    _registry: ClassVar[dict[str, type[Self]]] = {}

    def __init_subclass__(cls) -> None:
        cls._registry[cls.tag()] = cls

    @classmethod
    def tag(cls) -> str:
        """Get the tag for the metadata class. Will be used for serialization and deserialization.

        Returns:
            The tag for the metadata class.
        """
        return pascal_to_snake(cls.__name__)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        """Create an instance of the metadata class from a dictionary.

        Args:
            data (dict[str, Any]): The dictionary containing the metadata.

        Returns:
            An instance of the metadata class.
        """
        if "type" in data:
            del data["type"]
        return cls(**data)

    def to_dict(self) -> dict[str, Any]:
        """Convert the metadata instance to a dictionary.

        Returns:
            The dictionary representation of the metadata.
        """
        d = asdict(self)
        d["type"] = self.tag()  # add type tag for deserialization
        del d["file"]  # don't serialize the file path
        return d

    def to_yaml(self) -> str:
        """Convert the metadata instance to a YAML string.

        Returns:
            The YAML representation of the metadata.
        """
        data = self.to_dict()
        return yaml.safe_dump(data)

    @classmethod
    def from_yaml(cls, yaml_str: str) -> Metadata | FailedMetadataParse:
        """Create an instance of the metadata class from a YAML string.

        Will return the appropriate subclass based on the "type" field in the YAML.

        Args:
            yaml_str (str): The YAML string containing the metadata.

        Returns:
            An instance of the metadata class or an error.
        """
        data = yaml.safe_load(yaml_str)

        if "type" not in data:
            return FailedMetadataParse("No type field found in metadata.")

        for name, subclass in cls._registry.items():
            if data["type"] == name:
                try:
                    return subclass.from_dict(data)
                except Exception as e:
                    return FailedMetadataParse(f"Failed to parse {name}: {e}")

        return FailedMetadataParse(f"Unknown metadata type '{data['type']}'")


@dataclass(kw_only=True)
class GroundTruth(Metadata):
    """Metadata for ground truth trajectories."""

    sequence: str
    """Dataset used to run the experiment."""


M = TypeVar("M", bound=Metadata | None, default=None)


@dataclass(kw_only=True)
class Trajectory(Generic[M]):
    """A trajectory of poses with associated timestamps and metadata."""

    stamps: list[Stamp] = field(default_factory=list)
    """List of timestamps for each pose."""
    poses: list[SE3] = field(default_factory=list)
    """List of poses, in the same order as the timestamps."""
    metadata: M = None  # type: ignore
    """Metadata associated with the trajectory, such as the dataset name or other information."""
    _file: Optional[TextIOWrapper] = None
    _csv_writer: Optional[Writer] = None

    def __post_init__(self):
        if len(self.stamps) != len(self.poses):
            raise ValueError("Stamps and poses must have the same length.")

    def __getitem__(self, idx: int) -> tuple[Stamp, SE3]:
        """Get a (stamp, pose) pair by index.

        Args:
            idx (int): The index of the (stamp, pose) pair.

        Returns:
            The (stamp, pose) pair at the given index.
        """
        return self.stamps[idx], self.poses[idx]

    def __len__(self) -> int:
        """Get the length of the trajectory.

        Returns:
            The number of (stamp, pose) pairs in the trajectory.
        """
        return len(self.stamps)

    def __iter__(self) -> Iterator[tuple[Stamp, SE3]]:
        """Iterate over the trajectory.

        Returns:
            An iterator over the (stamp, pose) pairs.
        """
        return iter(zip(self.stamps, self.poses))

    def append(self, stamp: Stamp, pose: SE3):
        """Append a new pose to the trajectory.

        Will also write to file if the trajectory was opened with [open][evalio.types.Trajectory.open].

        Args:
            stamp (Stamp): The timestamp of the pose.
            pose (SE3): The pose to append.
        """
        self.stamps.append(stamp)
        self.poses.append(pose)

        if self._csv_writer is not None:
            self._csv_writer.writerow(self._serialize_pose(stamp, pose))

    def transform_in_place(self, T: SE3):
        """Apply a transformation to all poses in the trajectory.

        Args:
            T (SE3): The transformation to apply.
        """
        for i in range(len(self.poses)):
            self.poses[i] = self.poses[i] * T

    # ------------------------- Loading from file ------------------------- #
    @staticmethod
    def from_csv(
        path: Path,
        fieldnames: list[str],
        delimiter: str = ",",
        skip_lines: int = 0,
    ) -> Trajectory:
        """Flexible loader for stamped poses stored in csv files.

        Will automatically skip any lines that start with a #.

        ``` py
        from evalio.types import Trajectory

        fieldnames = ["sec", "nsec", "x", "y", "z", "qx", "qy", "qz", "qw"]
        trajectory = Trajectory.from_csv(path, fieldnames)
        ```

        Args:
            path (Path): Location of file.
            fieldnames (list[str]): List of field names to use, in their expected order. See above for an example.
            delimiter (str, optional): Delimiter between elements. Defaults to ",".
            skip_lines (int, optional): Number of lines to skip, useful for skipping headers. Defaults to 0.

        Returns:
            Stored trajectory
        """
        poses: list[SE3] = []
        stamps: list[Stamp] = []

        fields = {name: i for i, name in enumerate(fieldnames)}

        with open(path) as f:
            csvfile = filter(lambda row: row[0] != "#", f)
            for i, line in enumerate(csvfile):
                if i < skip_lines:
                    continue
                stamp, pose = parse_csv_line(line, delimiter, fields)

                poses.append(pose)
                stamps.append(stamp)

        return Trajectory(stamps=stamps, poses=poses)

    @staticmethod
    def from_tum(path: Path) -> Trajectory:
        """Load a TUM dataset pose file. Simple wrapper around [from_csv][evalio.types.Trajectory].

        Args:
            path (Path): Location of file.

        Returns:
            Stored trajectory
        """
        return Trajectory.from_csv(path, ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"])

    @staticmethod
    def from_file(
        path: Path,
    ) -> Trajectory[Metadata] | FailedMetadataParse | FileNotFoundError:
        """Load a saved evalio trajectory from file.

        Works identically to [from_tum][evalio.types.Trajectory.from_tum], but also loads metadata from the file.

        Args:
            path (Path): Location of trajectory results.

        Returns:
            Loaded trajectory with metadata, stamps, and poses.
        """
        if not path.exists():
            return FileNotFoundError(f"File {path} does not exist.")

        with open(path) as file:
            metadata_filter = filter(
                lambda row: row[0] == "#" and not row.startswith("# timestamp,"), file
            )
            metadata_list = [row[1:] for row in metadata_filter]
            metadata_str = "".join(metadata_list)

            metadata = Metadata.from_yaml(metadata_str)
            if isinstance(metadata, FailedMetadataParse):
                return metadata

            metadata.file = path

        trajectory = Trajectory.from_csv(
            path,
            fieldnames=["sec", "x", "y", "z", "qx", "qy", "qz", "qw"],
        )
        trajectory = cast(Trajectory[Metadata], trajectory)
        trajectory.metadata = metadata

        return trajectory

    # ------------------------- Saving to file ------------------------- #
    def _serialize_pose(self, stamp: Stamp, pose: SE3) -> list[str | float]:
        return [
            f"{stamp.sec}.{stamp.nsec:09}",
            pose.trans[0],
            pose.trans[1],
            pose.trans[2],
            pose.rot.qx,
            pose.rot.qy,
            pose.rot.qz,
            pose.rot.qw,
        ]

    def _serialize_metadata(self) -> str:
        if self.metadata is None:
            return ""

        metadata_str = self.metadata.to_yaml()
        metadata_str = metadata_str.replace("\n", "\n# ")
        return f"# {metadata_str}\n"

    def _write(self):
        if self._file is None or self._csv_writer is None:
            return

        # write everything we've got so far
        if self.metadata is not None:
            self._file.write(self._serialize_metadata())

        self._file.write("# timestamp, x, y, z, qx, qy, qz, qw\n")
        self._csv_writer.writerows(self._serialize_pose(s, p) for s, p in self)

    def open(self, path: Optional[Path] = None):
        """Open a CSV file for writing.

        This will overwrite any existing file. If no path is provided, will use the path in the metadata, if it exists.

        Args:
            path (Optional[Path], optional): Path to the CSV file. Defaults to None.
        """
        if path is not None:
            pass
        elif self.metadata is not None and self.metadata.file is not None:
            path = self.metadata.file
        else:
            print_warning(
                "Trajectory.open: No metadata or path provided, cannot set metadata file."
            )
            return

        path.parent.mkdir(parents=True, exist_ok=True)
        self._file = path.open("w")
        self._csv_writer = csv.writer(self._file)
        self._write()

    def close(self):
        """Close the CSV file if it was opened with [open][evalio.types.Trajectory.open]."""
        if self._file is not None:
            self._file.close()
            self._file = None
            self._csv_writer = None
        else:
            print_warning("Trajectory.close: No file to close.")

    def to_file(self, path: Optional[Path] = None):
        """Save the trajectory to a CSV file.

        Args:
            path (Optional[Path], optional): Path to the CSV file. If not specified, utilizes the path in the metadata, if it exists. Defaults to None.
        """
        self.open(path)
        self.close()

    def rewrite(self):
        """Update the contents of an open file."""
        if self._file is None or self._csv_writer is None:
            print_warning("Trajectory.rewrite: No file is open.")
            return

        if self.metadata is None:
            print_warning("Trajectory.rewrite: No metadata to update.")
            return

        # Go to start, empty, and rewrite
        self._file.seek(0)
        self._file.truncate()
        self._write()

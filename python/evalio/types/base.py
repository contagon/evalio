"""
These are the base python-based types used throughout evalio.

They MUST not depend on anything else in evalio, or else circular imports will occur.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import csv
from _csv import Writer
from enum import Enum
from io import TextIOWrapper
from typing_extensions import TypeVar
from evalio.utils import print_warning
import numpy as np
import yaml

from pathlib import Path
from typing import Any, ClassVar, Generic, Optional, Self, cast

from evalio._cpp.types import (  # type: ignore
    SE3,
    SO3,
    Stamp,
)

from evalio.utils import pascal_to_snake

Param = bool | int | float | str


class ExperimentStatus(Enum):
    Complete = "complete"
    Fail = "fail"
    Started = "started"


class FailedMetadataParse(Exception):
    def __init__(self, reason: str):
        super().__init__(f"Failed to parse metadata: {reason}")
        self.reason = reason


@dataclass(kw_only=True)
class Metadata:
    file: Optional[Path] = None
    """File where the metadata was loaded to and from, if any."""
    _registry: ClassVar[dict[str, type[Self]]] = {}

    def __init_subclass__(cls) -> None:
        cls._registry[cls.tag()] = cls

    @classmethod
    def tag(cls) -> str:
        return pascal_to_snake(cls.__name__)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        if "type" in data:
            del data["type"]
        return cls(**data)

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["type"] = self.tag()  # add type tag for deserialization
        del d["file"]  # don't serialize the file path
        return d

    def to_yaml(self) -> str:
        data = self.to_dict()
        return yaml.safe_dump(data)

    @classmethod
    def from_yaml(cls, yaml_str: str) -> Metadata | FailedMetadataParse:
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
    sequence: str
    """Dataset used to run the experiment."""


M = TypeVar("M", bound=Metadata | None, default=None)


@dataclass(kw_only=True)
class Trajectory(Generic[M]):
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
        return self.stamps[idx], self.poses[idx]

    def __len__(self) -> int:
        return len(self.stamps)

    def __iter__(self):
        return iter(zip(self.stamps, self.poses))

    def append(self, stamp: Stamp, pose: SE3):
        self.stamps.append(stamp)
        self.poses.append(pose)

        if self._csv_writer is not None:
            self._csv_writer.writerow(self._serialize_pose(stamp, pose))

    def transform_in_place(self, T: SE3):
        for i in range(len(self.poses)):
            self.poses[i] = self.poses[i] * T

    # ------------------------- Loading from file ------------------------- #
    @staticmethod
    def from_csv(
        path: Path,
        fieldnames: list[str],
        delimiter: str = ",",
        skip_lines: Optional[int] = None,
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
            Trajectory: Stored dataset
        """
        poses: list[SE3] = []
        stamps: list[Stamp] = []

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
                    s, ns = line["sec"].split(
                        "."
                    )  # parse separately to get exact stamp
                    ns = ns.ljust(9, "0")  # pad to 9 digits for nanoseconds
                    stamp = Stamp(sec=int(s), nsec=int(ns))
                elif "sec" not in fieldnames:
                    stamp = Stamp.from_nsec(int(line["nsec"]))
                else:
                    stamp = Stamp(sec=int(line["sec"]), nsec=int(line["nsec"]))
                poses.append(pose)
                stamps.append(stamp)

        return Trajectory(stamps=stamps, poses=poses)

    @staticmethod
    def from_tum(path: Path) -> "Trajectory":
        """Load a TUM dataset pose file. Simple wrapper around [from_csv][evalio.types.Trajectory].

        Args:
            path (Path): Location of file.

        Returns:
            Trajectory: Stored trajectory
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
            Trajectory: Loaded trajectory with metadata, stamps, and poses.
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
        """Helper to serialize a stamped pose for csv writing.

        Args:
            stamp (Stamp): Timestamp associated with the pose.
            pose (SE3): Pose to save.
        """
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
            print_warning("Trajectory.write_experiment: No file is open.")
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
        """Close the CSV file if it is open with [write_experiment][evalio.types.Trajectory.write_experiment] and incremental writing."""
        if self._file is not None:
            self._file.close()
            self._file = None
            self._csv_writer = None
        else:
            print_warning("Trajectory.close: No file to close.")

    def to_file(self, path: Optional[Path] = None):
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

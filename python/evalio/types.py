import csv
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Optional

from evalio.utils import print_warning
import numpy as np
import yaml

from ._cpp.types import (  # type: ignore
    SE3,
    SO3,
    Duration,
    ImuMeasurement,
    ImuParams,
    LidarMeasurement,
    LidarParams,
    Point,
    Stamp,
)

Param = bool | int | float | str


@dataclass(kw_only=True)
class GroundTruth:
    sequence: str
    """Dataset used to run the experiment."""

    @staticmethod
    def from_yaml(yaml_str: str) -> Optional["GroundTruth"]:
        """Create a GroundTruth object from a YAML string.

        Args:
            yaml_str (str): YAML string to parse.
        """
        data = yaml.safe_load(yaml_str)

        gt: Optional[bool] = data.pop("gt", None)
        sequence = data.pop("sequence", None)

        if gt is None or not gt or sequence is None:
            return None

        return GroundTruth(sequence=sequence)

    def to_yaml(self) -> str:
        """Convert the GroundTruth object to a YAML string.

        Returns:
            str: YAML string representation of the GroundTruth object.
        """
        data = {
            "gt": True,
            "sequence": self.sequence,
        }

        return yaml.dump(data)


class ExperimentStatus(Enum):
    Complete = "complete"
    Fail = "fail"
    Started = "started"
    NeverRan = "never_ran"


@dataclass(kw_only=True)
class Experiment:
    name: str
    """Name of the experiment."""
    status: ExperimentStatus
    """Status of the experiment, e.g. "success", "failure", etc."""
    sequence: str
    """Dataset used to run the experiment."""
    sequence_length: int
    """Length of the sequence, if set"""
    pipeline: str
    """Pipeline used to generate the trajectory."""
    pipeline_version: str
    """Version of the pipeline used."""
    pipeline_params: dict[str, Param] = field(default_factory=dict)
    """Parameters used for the pipeline."""
    total_elapsed: Optional[float] = None
    """Total time taken for the experiment, as a string."""
    max_step_elapsed: Optional[float] = None
    """Maximum time taken for a single step in the experiment, as a string."""

    @staticmethod
    def from_yaml(yaml_str: str) -> Optional["Experiment"]:
        """Create an Experiment object from a YAML string.

        Args:
            yaml_str (str): YAML string to parse.
        """
        data = yaml.safe_load(yaml_str)

        name = data.pop("name", None)
        pipeline = data.pop("pipeline", None)
        pipeline_version = data.pop("pipeline_version", None)
        pipeline_params = data.pop("pipeline_params", None)
        sequence = data.pop("sequence", None)
        sequence_length = data.pop("sequence_length", None)

        if (
            name is None
            or sequence is None
            or sequence_length is None
            or pipeline is None
            or pipeline_version is None
            or pipeline_params is None
        ):
            return None

        total_elapsed = data.pop("total_elapsed", None)
        max_step_elapsed = data.pop("max_step_elapsed", None)

        if "status" in data:
            status = ExperimentStatus(data.pop("status"))
        else:
            status = ExperimentStatus.Started

        if len(data) > 0:
            # Unknown fields
            print_warning(
                f"Experiment.from_yaml: Unknown fields in YAML: {list(data.keys())}"
            )

        return Experiment(
            name=name,
            sequence=sequence,
            sequence_length=sequence_length,
            pipeline=pipeline,
            pipeline_version=pipeline_version,
            pipeline_params=pipeline_params,
            status=status,
            total_elapsed=total_elapsed,
            max_step_elapsed=max_step_elapsed,
        )

    def to_yaml(self) -> str:
        """Convert the Experiment object to a YAML string.

        Returns:
            str: YAML string representation of the Experiment object.
        """
        data: dict[str, Param | dict[str, Param]] = {
            "name": self.name,
            "sequence": self.sequence,
            "sequence_length": self.sequence_length,
            "pipeline": self.pipeline,
            "pipeline_params": self.pipeline_params,
        }
        if self.status in [ExperimentStatus.Complete, ExperimentStatus.Fail]:
            data["status"] = self.status.value
        if self.total_elapsed is not None:
            data["total_elapsed"] = self.total_elapsed
        if self.max_step_elapsed is not None:
            data["max_step_elapsed"] = self.max_step_elapsed

        return yaml.dump(data)


def _parse_metadata(yaml_str: str) -> Optional[GroundTruth | Experiment]:
    if "gt:" in yaml_str:
        return GroundTruth.from_yaml(yaml_str)
    elif "name:" in yaml_str:
        return Experiment.from_yaml(yaml_str)
    else:
        return None


@dataclass(kw_only=True)
class Trajectory:
    stamps: list[Stamp]
    """List of timestamps for each pose."""
    poses: list[SE3]
    """List of poses, in the same order as the timestamps."""
    metadata: Optional[GroundTruth | Experiment] = None
    """Metadata associated with the trajectory, such as the dataset name or other information."""

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

    def transform_in_place(self, T: SE3):
        for i in range(len(self.poses)):
            self.poses[i] = self.poses[i] * T

    @staticmethod
    def from_csv(
        path: Path,
        fieldnames: list[str],
        delimiter: str = ",",
        skip_lines: Optional[int] = None,
    ) -> "Trajectory":
        """Flexible loader for stamped poses stored in csv files.

        Will automatically skip any lines that start with a #. Is most useful for loading ground truth data.

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
    def from_experiment(path: Path) -> Optional["Trajectory"]:
        """Load a saved experiment trajectory from file.

        Works identically to [from_tum][evalio.types.Trajectory.from_tum], but also loads metadata from the file.

        Args:
            path (Path): Location of trajectory results.

        Returns:
            Trajectory: Loaded trajectory with metadata, stamps, and poses.
        """
        with open(path) as file:
            metadata_filter = filter(
                lambda row: row[0] == "#" and not row.startswith("# timestamp,"), file
            )
            metadata_list = [row[1:].strip() for row in metadata_filter]
            if len(metadata_list) == 0:
                return None

            metadata_str = "\n".join(metadata_list)
            metadata = _parse_metadata(metadata_str)

        trajectory = Trajectory.from_csv(
            path,
            fieldnames=["sec", "x", "y", "z", "qx", "qy", "qz", "qw"],
        )
        trajectory.metadata = metadata

        return trajectory


# TODO: Some sort of incremental writer for experiments
# TODO: Some sort of batch writer for ground truth (can be the same as above)


__all__ = [
    "Experiment",
    "ExperimentStatus",
    "ImuMeasurement",
    "ImuParams",
    "LidarMeasurement",
    "LidarParams",
    "Duration",
    "Param",
    "Point",
    "SO3",
    "SE3",
    "Stamp",
    "Trajectory",
]

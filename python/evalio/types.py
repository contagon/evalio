from ._cpp.types import (  # type: ignore
    SE3,
    SO3,
    Duration,
    Stamp,
    Point,
    LidarMeasurement,
    LidarParams,
    ImuParams,
    ImuMeasurement,
)
from dataclasses import dataclass, field

from pathlib import Path
import yaml

from typing import Optional
import csv

import numpy as np


@dataclass(kw_only=True)
class Trajectory:
    stamps: list[Stamp]
    """List of timestamps for each pose."""
    poses: list[SE3]
    """List of poses, in the same order as the timestamps."""
    metadata: dict = field(default_factory=dict)
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
        delimiter=",",
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
    def from_experiment(path: Path) -> "Trajectory":
        """Load a saved experiment trajectory from file.

        Works identically to [from_tum][evalio.types.Trajectory.from_tum], but also loads metadata from the file.

        Args:
            path (Path): Location of trajectory results.

        Returns:
            Trajectory: Loaded trajectory with metadata, stamps, and poses.
        """
        with open(path) as file:
            metadata_filter = filter(lambda row: row[0] == "#", file)
            metadata_list = [row[1:].strip() for row in metadata_filter]
            # remove the header row
            metadata_list.pop(-1)
            metadata_str = "\n".join(metadata_list)
            metadata = yaml.safe_load(metadata_str)

        trajectory = Trajectory.from_csv(
            path,
            fieldnames=["sec", "x", "y", "z", "qx", "qy", "qz", "qw"],
        )
        trajectory.metadata = metadata

        return trajectory


__all__ = [
    "ImuMeasurement",
    "ImuParams",
    "LidarMeasurement",
    "LidarParams",
    "Duration",
    "Point",
    "SO3",
    "SE3",
    "Stamp",
    "Trajectory",
]

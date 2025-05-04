from copy import deepcopy
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


def _check_overstep(stamps: list[Stamp], s: Stamp, idx: int) -> bool:
    return abs((stamps[idx - 1] - s).to_sec()) < abs((stamps[idx] - s).to_sec())


@dataclass(kw_only=True)
class Trajectory:
    stamps: list[Stamp]
    poses: list[SE3]
    metadata: dict = field(default_factory=dict)

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

    @staticmethod
    def align(
        traj: "Trajectory", gt: "Trajectory", in_place: bool = False
    ) -> tuple["Trajectory", "Trajectory"]:
        """Align the trajectories both spatially and temporally.

        The resulting trajectories will be have the same origin as the second ("gt") trajectory.
        See [align_poses][evalio.types.Trajectory._align_poses] and [align_stamps][evalio.types.Trajectory._align_stamps] for more details.

        Args:
            traj (Trajectory): One of the trajectories to align.
            gt (Trajectory): The other trajectory to align to.
            in_place (bool, optional): If true, the original trajectory will be modified. Defaults to False.
        """
        if not in_place:
            traj = deepcopy(traj)
            gt = deepcopy(gt)

        Trajectory._align_stamps(traj, gt)
        Trajectory._align_poses(traj, gt)

        return traj, gt

    @staticmethod
    def _align_poses(traj: "Trajectory", other: "Trajectory"):
        """Align the trajectory in place to another trajectory. Operates in place.

        This results in the current trajectory having an identical first pose to the other trajectory.
        Assumes the first pose of both trajectories have the same stamp.

        Args:
            traj (Trajectory): The trajectory that will be modified
            other (Trajectory): The trajectory to align to.
        """
        this = traj.poses[0]
        oth = other.poses[0]
        delta = oth * this.inverse()

        for i in range(len(traj.poses)):
            traj.poses[i] = delta * traj.poses[i]

    @staticmethod
    def _align_stamps(traj1: "Trajectory", traj2: "Trajectory"):
        """Select the closest poses in traj1 and traj2. Operates in place.

        Does this by finding the higher frame rate trajectory and subsampling it to the closest poses of the other one.
        Additionally it checks the beginning of the trajectories to make sure they start at about the same stamp.

        Args:
            traj1 (Trajectory): One trajectory
            traj2 (Trajectory): Other trajectory

        Returns:
            tuple[Trajectory, Trajectory]: Sub-sampled trajectories
        """
        # Check if we need to skip poses in traj1
        first_pose_idx = 0
        while traj1.stamps[first_pose_idx] < traj2.stamps[0]:
            first_pose_idx += 1
        if _check_overstep(traj1.stamps, traj2.stamps[0], first_pose_idx):
            first_pose_idx -= 1
        traj1.stamps = traj1.stamps[first_pose_idx:]
        traj1.poses = traj1.poses[first_pose_idx:]

        # Check if we need to skip poses in traj2
        first_pose_idx = 0
        while traj2.stamps[first_pose_idx] < traj1.stamps[0]:
            first_pose_idx += 1
        if _check_overstep(traj2.stamps, traj1.stamps[0], first_pose_idx):
            first_pose_idx -= 1
        traj2.stamps = traj2.stamps[first_pose_idx:]
        traj2.poses = traj2.poses[first_pose_idx:]

        # Find the one that is at a higher frame rate
        # Leaves us with traj1 being the one with the higher frame rate
        swapped = False
        traj_1_dt = (traj1.stamps[-1] - traj1.stamps[0]).to_sec() / len(traj1.stamps)
        traj_2_dt = (traj2.stamps[-1] - traj2.stamps[0]).to_sec() / len(traj2.stamps)
        if traj_1_dt > traj_2_dt:
            traj1, traj2 = traj2, traj1
            swapped = True

        # Align the two trajectories by subsampling keeping traj1 stamps
        traj1_idx = 0
        traj1_stamps = []
        traj1_poses = []
        for i, stamp in enumerate(traj2.stamps):
            while traj1_idx < len(traj1) - 1 and traj1.stamps[traj1_idx] < stamp:
                traj1_idx += 1

            # go back one if we overshot
            if _check_overstep(traj1.stamps, stamp, traj1_idx):
                traj1_idx -= 1

            traj1_stamps.append(traj1.stamps[traj1_idx])
            traj1_poses.append(traj1.poses[traj1_idx])

            if traj1_idx >= len(traj1) - 1:
                traj2.stamps = traj2.stamps[: i + 1]
                traj2.poses = traj2.poses[: i + 1]
                break

        traj1.stamps = traj1_stamps
        traj1.poses = traj1_poses

        if swapped:
            traj1, traj2 = traj2, traj1


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

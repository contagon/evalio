from copy import deepcopy
from dataclasses import dataclass
from enum import StrEnum, auto
from typing import cast

import numpy as np
from numpy.typing import NDArray

from evalio.utils import print_warning

from .types import SE3, Stamp, Trajectory


def check_overstep(stamps: list[Stamp], s: Stamp, idx: int) -> bool:
    """Checks if we overshot the closest stamp.

    Args:
        stamps (list[Stamp]): List of stamps
        s (Stamp): Stamp we want to find the closest to
        idx (int): Index of the closest stamp

    Returns:
        bool: True if we overshot the closest stamp (ie it's idx - 1), False if it's good (ie it's idx)
    """
    return abs((stamps[idx - 1] - s).to_sec()) < abs((stamps[idx] - s).to_sec())


class MetricKind(StrEnum):
    """Simple enum to define the metric to use for summarizing the error. Used in [Error][evalio.stats.Error.summarize]."""

    mean = auto()
    """Mean"""
    median = auto()
    """Median"""
    sse = auto()
    """Sqrt of Sum of squared errors"""


@dataclass(kw_only=True)
class Metric:
    """Simple dataclass to hold the resulting metrics. Likely output from [Error][evalio.stats.Error]."""

    trans: float
    """translation error in meters"""
    rot: float
    """rotation error in degrees"""


@dataclass(kw_only=True)
class Error:
    """
    Dataclass to hold the error between two trajectories.
    Generally output from computing [ate][evalio.stats.ate] or [rte][evalio.stats.rte].

    Contains a (n,) arrays of translation and rotation errors.
    """

    # Shape: (n,)
    trans: NDArray[np.float64]
    """translation error, shape (n,), in meters"""
    rot: NDArray[np.float64]
    """rotation error, shape (n,), in degrees"""

    def summarize(self, metric: MetricKind) -> Metric:
        """How to summarize the vector of errors.

        Args:
            metric (MetricKind): The metric to use for summarizing the error,
                either mean, median, or sse.

        Returns:
            Metric: The summarized error
        """
        match metric:
            case MetricKind.mean:
                return self.mean()
            case MetricKind.median:
                return self.median()
            case MetricKind.sse:
                return self.sse()

    def mean(self) -> Metric:
        """Compute the mean of the errors."""
        return Metric(rot=self.rot.mean(), trans=self.trans.mean())

    def sse(self) -> Metric:
        """Compute the sqrt of sum of squared errors."""
        length = len(self.rot)
        return Metric(
            rot=float(np.sqrt(self.rot @ self.rot / length)),
            trans=float(np.sqrt(self.trans @ self.trans / length)),
        )

    def median(self) -> Metric:
        """Compute the median of the errors."""
        return Metric(
            rot=cast(float, np.median(self.rot)),
            trans=cast(float, np.median(self.trans)),
        )


def align(
    traj: Trajectory, gt: Trajectory, in_place: bool = False
) -> tuple[Trajectory, Trajectory]:
    """Align the trajectories both spatially and temporally.

    The resulting trajectories will be have the same origin as the second ("gt") trajectory.
    See [align_poses][evalio.stats.align_poses] and [align_stamps][evalio.stats.align_stamps] for more details.

    Args:
        traj (Trajectory): One of the trajectories to align.
        gt (Trajectory): The other trajectory to align to.
        in_place (bool, optional): If true, the original trajectory will be modified. Defaults to False.
    """
    if not in_place:
        traj = deepcopy(traj)
        gt = deepcopy(gt)

    align_stamps(traj, gt)
    align_poses(traj, gt)

    return traj, gt


def align_poses(traj: Trajectory, other: Trajectory):
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


def align_stamps(traj1: Trajectory, traj2: Trajectory):
    """Select the closest poses in traj1 and traj2. Operates in place.

    Does this by finding the higher frame rate trajectory and subsampling it to the closest poses of the other one.
    Additionally it checks the beginning of the trajectories to make sure they start at about the same stamp.

    Args:
        traj1 (Trajectory): One trajectory
        traj2 (Trajectory): Other trajectory
    """
    # Check if we need to skip poses in traj1
    first_pose_idx = 0
    while traj1.stamps[first_pose_idx] < traj2.stamps[0]:
        first_pose_idx += 1
    if check_overstep(traj1.stamps, traj2.stamps[0], first_pose_idx):
        first_pose_idx -= 1
    traj1.stamps = traj1.stamps[first_pose_idx:]
    traj1.poses = traj1.poses[first_pose_idx:]

    # Check if we need to skip poses in traj2
    first_pose_idx = 0
    while traj2.stamps[first_pose_idx] < traj1.stamps[0]:
        first_pose_idx += 1
    if check_overstep(traj2.stamps, traj1.stamps[0], first_pose_idx):
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
    traj1_stamps: list[Stamp] = []
    traj1_poses: list[SE3] = []
    for i, stamp in enumerate(traj2.stamps):
        while traj1_idx < len(traj1) - 1 and traj1.stamps[traj1_idx] < stamp:
            traj1_idx += 1

        # go back one if we overshot
        if check_overstep(traj1.stamps, stamp, traj1_idx):
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


def _compute_metric(gts: list[SE3], poses: list[SE3]) -> Error:
    """Iterate and compute the SE(3) delta between two lists of poses.

    Args:
        gts (list[SE3]): One of the lists of poses
        poses (list[SE3]): The other list of poses

    Returns:
        Error: The computed error
    """
    assert len(gts) == len(poses)

    error_t = np.zeros(len(gts))
    error_r = np.zeros(len(gts))
    for i, (gt, pose) in enumerate(zip(gts, poses)):
        delta = gt.inverse() * pose
        error_t[i] = np.sqrt(delta.trans @ delta.trans)  # type: ignore
        r_diff = delta.rot.log()
        error_r[i] = np.sqrt(r_diff @ r_diff) * 180 / np.pi  # type: ignore

    return Error(rot=error_r, trans=error_t)


def _check_aligned(traj: Trajectory, gt: Trajectory) -> bool:
    """Check if the two trajectories are aligned.

    Args:
        traj (Trajectory): One of the trajectories
        gt (Trajectory): The other trajectory

    Returns:
        bool: True if the two trajectories are aligned, False otherwise
    """
    # Check if the two trajectories are aligned
    delta = gt.poses[0].inverse() * traj.poses[0]
    t = delta.trans
    r = delta.rot.log()
    return len(traj.stamps) == len(gt.stamps) and (t @ t < 1e-6) and (r @ r < 1e-6)  # type: ignore


def ate(traj: Trajectory, gt: Trajectory) -> Error:
    """Compute the Absolute Trajectory Error (ATE) between two trajectories.

    Will check if the two trajectories are aligned and if not, will align them.
    Will not modify the original trajectories.

    Args:
        traj (Trajectory): One of the trajectories
        gt (Trajectory): The other trajectory

    Returns:
        Error: The computed error
    """
    if not _check_aligned(traj, gt):
        traj, gt = align(traj, gt)

    # Compute the ATE
    return _compute_metric(gt.poses, traj.poses)


def rte(traj: Trajectory, gt: Trajectory, window: int = 100) -> Error:
    """Compute the Relative Trajectory Error (RTE) between two trajectories.

    Will check if the two trajectories are aligned and if not, will align them.
    Will not modify the original trajectories.

    Args:
        traj (Trajectory): One of the trajectories
        gt (Trajectory): The other trajectory
        window (int, optional): Window size for the RTE. Defaults to 100.

    Returns:
        Error: The computed error
    """
    if not _check_aligned(traj, gt):
        traj, gt = align(traj, gt)

    if window <= 0:
        raise ValueError("Window size must be positive")

    if window > len(gt) - 1:
        print_warning(f"Window size {window} is larger than number of poses {len(gt)}")
        return Error(rot=np.array([np.nan]), trans=np.array([np.nan]))

    window_deltas_poses: list[SE3] = []
    window_deltas_gts: list[SE3] = []
    for i in range(len(gt) - window):
        window_deltas_poses.append(traj.poses[i].inverse() * traj.poses[i + window])
        window_deltas_gts.append(gt.poses[i].inverse() * gt.poses[i + window])

    # Compute the RTE
    return _compute_metric(window_deltas_gts, window_deltas_poses)

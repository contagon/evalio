from enum import StrEnum, auto
from typing_extensions import TypeVar

from evalio.utils import print_warning
from evalio._cpp.helpers import closest  # type: ignore
from . import types as ty

from dataclasses import dataclass

import numpy as np

from typing import cast
from numpy.typing import NDArray

from copy import deepcopy


class MetricKind(StrEnum):
    """Simple enum to define the metric to use for summarizing the error. Used in [Error][evalio.stats.Error.summarize]."""

    mean = auto()
    """Mean"""
    median = auto()
    """Median"""
    sse = auto()
    """Sqrt of Sum of squared errors"""


@dataclass
class WindowMeters:
    """Dataclass to hold the parameters for a distance-based window."""

    value: float
    """Distance in meters"""

    def name(self) -> str:
        """Get a string representation of the window."""
        return f"{self.value:.1f}m"


@dataclass
class WindowSeconds:
    """Dataclass to hold the parameters for a time-based window."""

    value: float
    """Duration of the window in seconds"""

    def name(self) -> str:
        """Get a string representation of the window."""
        return f"{self.value}s"


WindowKind = WindowMeters | WindowSeconds
"""Type alias for either a [WindowMeters][evalio.stats.WindowMeters] or a [WindowSeconds][evalio.stats.WindowSeconds]."""


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
            The summarized error
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


M1 = TypeVar("M1", bound=ty.Metadata | None)
M2 = TypeVar("M2", bound=ty.Metadata | None)


def align(
    traj: ty.Trajectory[M1], gt: ty.Trajectory[M2], in_place: bool = False
) -> tuple[ty.Trajectory[M1], ty.Trajectory[M2]]:
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


def align_poses(traj: ty.Trajectory[M1], other: ty.Trajectory[M2]):
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


def align_stamps(traj1: ty.Trajectory[M1], traj2: ty.Trajectory[M2]):
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
    if not closest(
        traj2.stamps[0],
        traj1.stamps[first_pose_idx - 1],
        traj1.stamps[first_pose_idx],
    ):
        first_pose_idx -= 1
    traj1.stamps = traj1.stamps[first_pose_idx:]
    traj1.poses = traj1.poses[first_pose_idx:]

    # Check if we need to skip poses in traj2
    first_pose_idx = 0
    while traj2.stamps[first_pose_idx] < traj1.stamps[0]:
        first_pose_idx += 1
    if not closest(
        traj1.stamps[0],
        traj2.stamps[first_pose_idx - 1],
        traj2.stamps[first_pose_idx],
    ):
        first_pose_idx -= 1
    traj2.stamps = traj2.stamps[first_pose_idx:]
    traj2.poses = traj2.poses[first_pose_idx:]

    # Find the one that is at a higher frame rate
    # Leaves us with traj1 being the one with the higher frame rate
    swapped = False
    traj_1_dt = (traj1.stamps[-1] - traj1.stamps[0]).to_sec() / len(traj1.stamps)
    traj_2_dt = (traj2.stamps[-1] - traj2.stamps[0]).to_sec() / len(traj2.stamps)
    if traj_1_dt > traj_2_dt:
        traj1, traj2 = traj2, traj1  # type: ignore
        swapped = True

    # cache this value
    len_traj1 = len(traj1)

    # Align the two trajectories by subsampling keeping traj1 stamps
    traj1_idx = 0
    traj1_stamps: list[ty.Stamp] = []
    traj1_poses: list[ty.SE3] = []
    for i, stamp in enumerate(traj2.stamps):
        while traj1_idx < len_traj1 - 1 and traj1.stamps[traj1_idx] < stamp:
            traj1_idx += 1

        # go back one if we overshot
        if not closest(stamp, traj1.stamps[traj1_idx - 1], traj1.stamps[traj1_idx]):
            traj1_idx -= 1

        traj1_stamps.append(traj1.stamps[traj1_idx])
        traj1_poses.append(traj1.poses[traj1_idx])

        if traj1_idx >= len_traj1 - 1:
            traj2.stamps = traj2.stamps[: i + 1]
            traj2.poses = traj2.poses[: i + 1]
            break

    traj1.stamps = traj1_stamps
    traj1.poses = traj1_poses

    if swapped:
        traj1, traj2 = traj2, traj1  # type: ignore


def _compute_metric(gts: list[ty.SE3], poses: list[ty.SE3]) -> Error:
    """Iterate and compute the SE(3) delta between two lists of poses.

    Args:
        gts (list[SE3]): One of the lists of poses
        poses (list[SE3]): The other list of poses

    Returns:
        The computed error
    """
    assert len(gts) == len(poses)

    error_t = np.zeros(len(gts))
    error_r = np.zeros(len(gts))
    for i, (gt, pose) in enumerate(zip(gts, poses)):
        error_r[i], error_t[i] = ty.SE3.error(gt, pose)

    return Error(rot=error_r, trans=error_t)


def _check_aligned(traj: ty.Trajectory[M1], gt: ty.Trajectory[M2]) -> bool:
    """Check if the two trajectories are aligned.

    This is done by checking if the first poses are identical, and if there's the same number of poses in both trajectories.

    Args:
        traj (Trajectory): One of the trajectories
        gt (Trajectory): The other trajectory

    Returns:
        True if the two trajectories are aligned, False otherwise
    """
    # Check if the two trajectories are aligned
    delta = gt.poses[0].inverse() * traj.poses[0]
    r = delta.rot.log()
    return bool(
        len(traj.stamps) == len(gt.stamps)
        and (delta.trans @ delta.trans < 1e-6)
        and (r @ r < 1e-6)
    )


def ate(traj: ty.Trajectory[M1], gt: ty.Trajectory[M2]) -> Error:
    """Compute the Absolute Trajectory Error (ATE) between two trajectories.

    Will check if the two trajectories are aligned and if not, will align them.
    Will not modify the original trajectories.

    Args:
        traj (Trajectory): One of the trajectories
        gt (Trajectory): The other trajectory

    Returns:
        The computed error
    """
    if not _check_aligned(traj, gt):
        traj, gt = align(traj, gt)

    # Compute the ATE
    return _compute_metric(gt.poses, traj.poses)


def rte(
    traj: ty.Trajectory[M1],
    gt: ty.Trajectory[M2],
    window: WindowKind = WindowMeters(30),
) -> Error:
    """Compute the Relative Trajectory Error (RTE) between two trajectories.

    Will check if the two trajectories are aligned and if not, will align them.
    Will not modify the original trajectories.

    Args:
        traj (Trajectory): One of the trajectories
        gt (Trajectory): The other trajectory
        window (WindowKind, optional): The window to use for computing the RTE.
            Either a [WindowMeters][evalio.stats.WindowMeters] or a [WindowSeconds][evalio.stats.WindowSeconds].
            Defaults to WindowMeters(30), which is a 30 meter window.
    Returns:
        The computed error
    """
    if not _check_aligned(traj, gt):
        traj, gt = align(traj, gt)

    if window.value <= 0:
        raise ValueError("Window size must be positive")

    window_deltas_poses: list[ty.SE3] = []
    window_deltas_gts: list[ty.SE3] = []

    # cache this value
    len_gt = len(gt)

    if isinstance(window, WindowSeconds):
        # Find our pairs for computation
        end_idx = 1
        duration = ty.Duration.from_sec(window.value)

        for i in range(len_gt):
            while end_idx < len_gt and gt.stamps[end_idx] - gt.stamps[i] < duration:
                end_idx += 1

            if end_idx >= len_gt:
                break

            window_deltas_poses.append(traj.poses[i].inverse() * traj.poses[end_idx])
            window_deltas_gts.append(gt.poses[i].inverse() * gt.poses[end_idx])

    elif isinstance(window, WindowMeters):
        # Compute deltas for all of ground truth poses
        dist = np.zeros(len_gt)
        for i in range(1, len_gt):
            dist[i] = ty.SE3.distance(gt.poses[i], gt.poses[i - 1])

        cum_dist = np.cumsum(dist)
        end_idx = 1
        end_idx_prev = 0

        # Find our pairs for computation
        for i in range(len_gt):
            while end_idx < len_gt and cum_dist[end_idx] - cum_dist[i] < window.value:
                end_idx += 1

            if end_idx >= len_gt:
                break
            elif end_idx == end_idx_prev:
                continue

            window_deltas_poses.append(traj.poses[i].inverse() * traj.poses[end_idx])
            window_deltas_gts.append(gt.poses[i].inverse() * gt.poses[end_idx])

            end_idx_prev = end_idx

    if len(window_deltas_poses) == 0:
        if isinstance(traj.metadata, ty.Experiment):
            print_warning(
                f"No {window} windows found for '{traj.metadata.name}' on '{traj.metadata.sequence}'"
            )
        else:
            print_warning(f"No {window} windows found")
        return Error(rot=np.array([np.nan]), trans=np.array([np.nan]))

    # Compute the RTE
    return _compute_metric(window_deltas_gts, window_deltas_poses)

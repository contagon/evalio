from copy import deepcopy

import numpy as np
from evalio import stats
from evalio.types import SE3, Stamp, Trajectory
from utils import isclose_se3, rand_se3

ID = SE3.identity()


def s(t: float) -> Stamp:
    return Stamp.from_sec(t)


def test_already_aligned():
    traj = Trajectory(
        stamps=[s(0), s(1), s(2)],
        poses=[ID, ID, ID],
    )

    traj1_out = deepcopy(traj)
    traj2_out = deepcopy(traj)
    stats.align_stamps(traj1_out, traj2_out)

    assert (traj1_out, traj2_out) == (traj, traj)


def test_subsample_first():
    traj1 = Trajectory(
        stamps=[s(i) for i in range(10)],
        poses=[ID for _ in range(10)],
    )

    traj2 = Trajectory(
        stamps=[s(i) for i in range(0, 10, 2)],
        poses=[ID for _ in range(0, 10, 2)],
    )

    traj1_out, traj2_out = deepcopy(traj1), deepcopy(traj2)
    stats.align_stamps(traj1_out, traj2_out)

    if traj1_out != traj2:
        raise ValueError(
            f"traj1out is wrong exp: {traj2.stamps}, got: {traj1_out.stamps}"
        )

    if traj2_out != traj2:
        raise ValueError(
            f"traj2out is wrong exp: {traj2.stamps}, got: {traj2_out.stamps}"
        )


def test_overstep():
    r = list(range(1, 11))
    traj1 = Trajectory(
        stamps=[s(i - 0.1) for i in r],
        poses=[ID for _ in r],
    )

    traj2 = Trajectory(
        stamps=[s(i) for i in r],
        poses=[ID for _ in r],
    )

    traj1_out, traj2_out = deepcopy(traj1), deepcopy(traj2)
    stats.align_stamps(traj1_out, traj2_out)

    if traj1_out != traj1:
        raise ValueError(
            f"traj1out is wrong exp: {traj1.stamps}, got: {traj1_out.stamps}"
        )

    if traj2_out != traj2:
        raise ValueError(
            f"traj2out is wrong exp: {traj2.stamps}, got: {traj2_out.stamps}"
        )


def testalign_poses():
    np.random.seed(0)
    gt = Trajectory(
        stamps=[s(i) for i in range(10)],
        poses=[rand_se3() for _ in range(10)],
    )

    offset = rand_se3()

    traj2 = Trajectory(
        stamps=[s(i) for i in range(10)],
        poses=[offset * pose for pose in gt.poses],
    )

    stats.align_poses(traj2, gt)

    for a, b in zip(traj2.poses, gt.poses):
        assert isclose_se3(a, b), f"{a} != {b}"

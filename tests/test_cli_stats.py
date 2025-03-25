from evalio.types import Stamp
from evalio.types import SE3, Trajectory
from evalio.cli.stats import align_stamps

ID = SE3.identity()


def s(t: float) -> Stamp:
    return Stamp.from_sec(t)


def test_already_aligned():
    traj = Trajectory(
        metadata={},
        stamps=[s(0), s(1), s(2)],
        poses=[ID, ID, ID],
    )

    assert align_stamps(traj, traj) == (traj, traj)


def test_subsample_first():
    traj1 = Trajectory(
        metadata={},
        stamps=[s(i) for i in range(10)],
        poses=[ID for _ in range(10)],
    )

    traj2 = Trajectory(
        metadata={},
        stamps=[s(i) for i in range(0, 10, 2)],
        poses=[ID for _ in range(0, 10, 2)],
    )

    traj1_out, traj2_out = align_stamps(traj1, traj2)

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
        metadata={},
        stamps=[s(i - 0.1) for i in r],
        poses=[ID for _ in r],
    )

    traj2 = Trajectory(
        metadata={},
        stamps=[s(i) for i in r],
        poses=[ID for _ in r],
    )

    traj1_out, traj2_out = align_stamps(traj1, traj2)

    if traj1_out != traj1:
        raise ValueError(
            f"traj1out is wrong exp: {traj1.stamps}, got: {traj1_out.stamps}"
        )

    if traj2_out != traj2:
        raise ValueError(
            f"traj2out is wrong exp: {traj2.stamps}, got: {traj2_out.stamps}"
        )

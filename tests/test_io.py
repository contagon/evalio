from pathlib import Path
from evalio import types as ty
from evalio._cpp.helpers import parse_csv_line  # type: ignore
import numpy as np


def make_exp() -> ty.Experiment:
    return ty.Experiment(
        name="test",
        status=ty.ExperimentStatus.Complete,
        sequence="newer_college_2020/short_experiment",
        sequence_length=1000,
        pipeline="fake",
        pipeline_version="0.1.0",
        pipeline_params={"param1": 1, "param2": "value"},
        total_elapsed=10.5,
        max_elapsed=0.24,
    )


def test_metadata_serde():
    exp = make_exp()
    out = ty.Metadata.from_yaml(exp.to_yaml())
    assert exp == out

    gt = ty.GroundTruth(sequence="newer_college_2020/short_experiment")
    out = ty.Metadata.from_yaml(gt.to_yaml())
    assert gt == out


def test_trajectory_serde(tmp_path: Path):
    path = tmp_path / "traj.csv"

    traj = ty.Trajectory(
        stamps=[ty.Stamp.from_sec(i) for i in range(5)],
        poses=[ty.SE3.exp(np.random.rand(6)) for _ in range(5)],
        metadata=make_exp(),
    )

    traj.metadata.file = path

    traj.to_file(path)

    loaded = ty.Trajectory.from_file(path)
    assert isinstance(loaded, ty.Trajectory)
    assert traj.stamps == loaded.stamps
    assert traj.poses == loaded.poses
    assert traj.metadata == loaded.metadata


def test_trajectory_incremental_serde(tmp_path: Path):
    path = tmp_path / "traj.csv"

    traj = ty.Trajectory(
        stamps=[ty.Stamp.from_sec(i) for i in range(5)],
        poses=[ty.SE3.exp(np.random.rand(6)) for _ in range(5)],
        metadata=make_exp(),
    )
    traj.metadata.file = path

    # poses are automatically written as they are added
    traj.open(path)
    traj.append(ty.Stamp.from_sec(5), ty.SE3.exp(np.random.rand(6)))
    traj.close()

    new_traj = ty.Trajectory.from_file(path)
    assert traj == new_traj

    # must trigger entire rewrite to update metadata
    traj.open(path)
    traj.metadata.sequence = "random_name"
    traj.rewrite()
    traj.close()

    new_traj = ty.Trajectory.from_file(path)
    assert traj == new_traj


def test_csv_line():
    exp_pose = ty.SE3(
        rot=ty.SO3(
            qx=-0.9998805501718303,
            qy=0.005631361428549012,
            qz=0.0033964292086203895,
            qw=0.013987044904833533,
        ),
        trans=np.array(
            [
                0.04846940144357503,
                -0.03130991015433452,
                -0.01876146196188756,
            ]
        ),
    )

    fields = ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"]
    fields = {v: i for i, v in enumerate(fields)}

    # Try a standard one
    line = "1670403901.143296798,0.04846940144357503,-0.03130991015433452,-0.01876146196188756,-0.9998805501718303,0.005631361428549012,0.0033964292086203895,0.013987044904833533"
    stamp, pose = parse_csv_line(line, ",", fields)

    assert stamp == ty.Stamp(sec=1670403901, nsec=143296798)
    assert pose == exp_pose

    # Try one with not padded nsec
    # Try a standard one
    line = "1670403901.143296,0.04846940144357503,-0.03130991015433452,-0.01876146196188756,-0.9998805501718303,0.005631361428549012,0.0033964292086203895,0.013987044904833533"
    stamp, pose = parse_csv_line(line, ",", fields)

    assert stamp == ty.Stamp(sec=1670403901, nsec=143296000)
    assert pose == exp_pose

    # Try one with both sec and nsec
    fields = ["sec", "nsec", "x", "y", "z", "qx", "qy", "qz", "qw"]
    fields = {v: i for i, v in enumerate(fields)}
    line = "1670403901,143296798,0.04846940144357503,-0.03130991015433452,-0.01876146196188756,-0.9998805501718303,0.005631361428549012,0.0033964292086203895,0.013987044904833533"
    stamp, pose = parse_csv_line(line, ",", fields)

    assert stamp == ty.Stamp(sec=1670403901, nsec=143296798)
    assert pose == exp_pose

    # Try one with just nsec
    fields = ["nsec", "x", "y", "z", "qx", "qy", "qz", "qw"]
    fields = {v: i for i, v in enumerate(fields)}
    line = "1670403901143296798,0.04846940144357503,-0.03130991015433452,-0.01876146196188756,-0.9998805501718303,0.005631361428549012,0.0033964292086203895,0.013987044904833533"
    stamp, pose = parse_csv_line(line, ",", fields)

    assert stamp == ty.Stamp.from_nsec(1670403901143296798)
    assert pose == exp_pose

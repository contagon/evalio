from pathlib import Path
from evalio import types as ty
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
        max_step_elapsed=0.24,
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

    if traj.metadata is not None:
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

    if traj.metadata is not None:
        traj.metadata.file = path

    # poses are automatically written as they are added
    traj.open(path)
    traj.append(ty.Stamp.from_sec(5), ty.SE3.exp(np.random.rand(6)))
    traj.close()

    new_traj = ty.Trajectory.from_file(path)
    assert traj == new_traj

    # must trigger entire rewrite to update metadata
    traj.open(path)
    if traj.metadata is not None:
        traj.metadata.sequence = "random_name"  # type: ignore
    traj.rewrite()
    traj.close()

    new_traj = ty.Trajectory.from_file(path)
    assert traj == new_traj

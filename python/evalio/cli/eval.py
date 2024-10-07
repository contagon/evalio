from pathlib import Path
from evalio.types import Stamp, SE3, SO3

import numpy as np

import csv
import yaml


def load(path: str) -> list[(Stamp, SE3)]:
    fieldnames = ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"]

    poses = []

    with open(path) as file:
        metadata = filter(lambda row: row[0] == "#", file)
        metadata = [row[1:].strip() for row in metadata]
        metadata.pop(-1)
        metadata = "\n".join(metadata)
        # remove the header row
        metadata = yaml.safe_load(metadata)

        # Load trajectory
        file.seek(0)
        csvfile = filter(lambda row: row[0] != "#", file)
        reader = csv.DictReader(csvfile, fieldnames=fieldnames)
        for line in reader:
            r = SO3(
                qw=float(line["qw"]),
                qx=float(line["qx"]),
                qy=float(line["qy"]),
                qz=float(line["qz"]),
            )
            t = np.array([float(line["x"]), float(line["y"]), float(line["z"])])
            pose = SE3(r, t)

            if "nsec" not in fieldnames:
                stamp = Stamp.from_sec(float(line["sec"]))
            elif "sec" not in fieldnames:
                stamp = Stamp.from_nsec(float(line["nsec"]))
            else:
                stamp = Stamp(sec=int(line["sec"]), nsec=int(line["nsec"]))
            poses.append((stamp, pose))

    return metadata, poses


def align(traj: list[(Stamp, SE3)], gt: list[(Stamp, SE3)]) -> list[(Stamp, SE3)]:
    first_stamp, imu_o_T_imu_0 = traj[0]

    # TODO: Handle case when they don't line up perfectly
    for stamp, pose in gt:
        if stamp == first_stamp:
            gt_o_T_imu_0 = pose
            break

    imu_o_T_gt_0 = imu_o_T_imu_0 * gt_o_T_imu_0.inverse()

    return [(stamp, imu_o_T_gt_0 * pose) for stamp, pose in gt]


def clean_gt(
    traj: list[(Stamp, SE3)], gt: list[(Stamp, SE3)]
) -> tuple[list[(Stamp, SE3)], list[(Stamp, SE3)]]:
    # Check if we need to skip any poses while waiting for ground truth
    first_pose_idx = 0
    while traj[first_pose_idx][0] < gt[0][0]:
        first_pose_idx += 1
    traj = traj[first_pose_idx:]

    # make it so they have the same origin
    gt = align(traj, gt)
    gt = gt[: len(traj)]

    # Double check the stamps line up
    for gti, traji in zip(gt, traj):
        assert gti[0] == traji[0]

    return traj, gt


def ate(traj: list[(Stamp, SE3)], gt_poses: list[(Stamp, SE3)]) -> float:
    """
    Computes the Absolute Trajectory Error
    """
    assert len(gt_poses) == len(traj)

    error = 0
    for gt, pose in zip(gt_poses, traj):
        e = np.linalg.norm(gt[1].trans - pose[1].trans)
        error += e

    return error / len(gt_poses)


def eval(dir: Path):
    pass


if __name__ == "__main__":
    meta, poses = load(
        "deterministic/newer_college_2020_01_short_experiment_kiss_icp_2.csv"
    )
    meta, gt = load("deterministic/newer_college_2020_01_short_experiment_gt.csv")

    poses, gt = clean_gt(poses, gt)

    print(ate(poses, gt))

    # import os
    # import sys

    # os.environ["TCL_LIBRARY"] = os.path.join(
    #     os.path.dirname(sys.executable), "..", "lib", "tcl8.6"
    # )
    import rerun as rr
    import rerun.blueprint as rrb
    import evalio.vis as evis

    rr.init(
        "evalio",
        spawn=False,
        default_blueprint=rrb.Spatial3DView(
            overrides={"imu/lidar": [rrb.components.Visible(False)]}
        ),
    )
    rr.connect("172.31.78.57:9876")

    # Extract positions from poses
    gt = [pose for _, pose in gt]
    poses = [pose for _, pose in poses]

    rr.log(
        "gt",
        evis.poses_to_points(gt, color=[0, 0, 255]),
        static=True,
    )
    rr.log(
        "poses",
        evis.poses_to_points(poses, color=[255, 0, 0]),
        static=True,
    )

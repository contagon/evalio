import csv
from copy import deepcopy
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import yaml
from attr import dataclass
from tabulate import tabulate

from evalio.types import SE3, SO3, Stamp, Trajectory


@dataclass(kw_only=True)
class Ate:
    trans: float
    rot: float


def load(path: Path) -> Trajectory:
    fieldnames = ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"]

    poses = []
    stamps = []

    with open(path) as file:
        metadata_filter = filter(lambda row: row[0] == "#", file)
        metadata_list = [row[1:].strip() for row in metadata_filter]
        metadata_list.pop(-1)
        metadata_str = "\n".join(metadata_list)
        # remove the header row
        metadata = yaml.safe_load(metadata_str)

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
                stamp = Stamp.from_nsec(int(line["nsec"]))
            else:
                stamp = Stamp(sec=int(line["sec"]), nsec=int(line["nsec"]))
            poses.append(pose)
            stamps.append(stamp)

    return Trajectory(metadata=metadata, stamps=stamps, poses=poses)


def align_stamps(
    traj1: Trajectory, traj2: Trajectory, tol: float = 1e-2
) -> tuple[Trajectory, Trajectory]:
    # Check if we need to skip poses in traj1
    first_pose_idx = 0
    while (traj1.stamps[first_pose_idx] - traj2.stamps[0]) < -tol:
        first_pose_idx += 1
    traj1.stamps = traj1.stamps[first_pose_idx:]
    traj1.poses = traj1.poses[first_pose_idx:]

    # Check if we need to skip poses in traj2
    first_pose_idx = 0
    while (traj2.stamps[first_pose_idx] - traj1.stamps[0]) < -tol:
        first_pose_idx += 1
    traj2.stamps = traj2.stamps[first_pose_idx:]
    traj2.poses = traj2.poses[first_pose_idx:]

    # Find the one that is at a higher frame rate
    # Leaves us with traj1 being the one with the higher frame rate
    swapped = False
    traj_1_rate = (traj1.stamps[-1] - traj1.stamps[0]) / len(traj1.stamps)
    traj_2_rate = (traj2.stamps[-1] - traj2.stamps[0]) / len(traj2.stamps)
    if (traj_1_rate - traj_2_rate) < -tol:
        traj1, traj2 = traj2, traj1
        swapped = True

    # Align the two trajectories by selectively keeping traj1 stamps
    traj1_idx = 0
    traj1_stamps = []
    traj1_poses = []
    for i, stamp in enumerate(traj2.stamps):
        while traj1_idx < len(traj1) - 1 and traj1.stamps[traj1_idx] - stamp < -tol:
            traj1_idx += 1

        traj1_stamps.append(traj1.stamps[traj1_idx])
        traj1_poses.append(traj1.poses[traj1_idx])

        if traj1_idx >= len(traj1) - 1:
            traj2.stamps = traj2.stamps[: i + 1]
            traj2.poses = traj2.poses[: i + 1]
            break

    traj1 = Trajectory(metadata=traj1.metadata, stamps=traj1_stamps, poses=traj1_poses)

    if swapped:
        traj1, traj2 = traj2, traj1

    return traj1, traj2


def align_poses(traj: Trajectory, gt: Trajectory):
    """Transforms the first to look like the second"""
    imu_o_T_imu_0 = traj.poses[0]
    gt_o_T_imu_0 = gt.poses[0]
    gt_o_T_imu_o = gt_o_T_imu_0 * imu_o_T_imu_0.inverse()

    traj.poses = [gt_o_T_imu_o * pose for pose in traj.poses]


def compute_ate(traj: Trajectory, gt_poses: Trajectory) -> Ate:
    """
    Computes the Absolute Trajectory Error
    """
    assert len(gt_poses) == len(traj)

    error_t = 0.0
    error_r = 0.0
    for gt, pose in zip(gt_poses.poses, traj.poses):
        error_t += float(np.linalg.norm(gt.trans - pose.trans))
        error_r += float(np.linalg.norm((gt.rot * pose.rot.inverse()).log()))

    error_t /= len(gt_poses)
    error_r /= len(gt_poses)

    return Ate(rot=error_r, trans=error_t)


def dict_diff(dicts: Sequence[dict]) -> list[str]:
    """
    Assumes each dictionary has the same keys
    """
    # quick sanity check
    size = len(dicts[0])
    for d in dicts:
        assert len(d) == size

    # compare all dictionaries to find varying keys
    diff = []
    for k in dicts[0].keys():
        if any(d[k] != dicts[0][k] for d in dicts):
            diff.append(k)

    return diff


def eval_dataset(dir: Path, visualize: bool, sort: Optional[str]):
    # Load all trajectories
    trajectories = []
    for file_path in dir.glob("*.csv"):
        traj = load(file_path)
        trajectories.append(traj)

    gt_list: list[Trajectory] = []
    trajs: list[Trajectory] = []
    for t in trajectories:
        (gt_list if "gt" in t.metadata else trajs).append(t)

    assert len(gt_list) == 1, f"Found multiple ground truths in {dir}"
    gt_og = gt_list[0]

    # Setup visualization
    # if visualize:
    #     import rerun as rr

    #     import evalio.vis as evis

    #     rr.init(
    #         str(dir),
    #         spawn=False,
    #     )
    #     rr.connect("0.0.0.0:9876")
    #     rr.log(
    #         "gt",
    #         evis.poses_to_points(gt_og.poses, color=[0, 0, 255]),
    #         static=True,
    #     )

    # Group into pipelines
    pipelines = set(traj.metadata["pipeline"] for traj in trajs)
    grouped_trajs: dict[str, list[Trajectory]] = {p: [] for p in pipelines}
    for traj in trajs:
        grouped_trajs[traj.metadata["pipeline"]].append(traj)

    # Find all keys that were different
    keys_to_print = ["pipeline"]
    for pipeline, trajs in grouped_trajs.items():
        keys = dict_diff([traj.metadata for traj in trajs])
        if len(keys) > 0:
            keys.remove("name")
            keys_to_print += keys

    header = ["ATEt", "ATEr", *keys_to_print]

    print(f"\nResults for {dir}")
    results = []
    for pipeline, trajs in grouped_trajs.items():
        # Iterate over each
        for traj in trajs:
            traj, gt = align_stamps(traj, deepcopy(gt_og))
            align_poses(traj, gt)
            ate = compute_ate(traj, gt)
            results.append(
                [
                    ate.trans,
                    ate.rot,
                    *[traj.metadata.get(k, "--") for k in keys_to_print],
                ]
            )

            # if visualize:
            #     rr.log(
            #         traj.metadata["name"],
            #         evis.poses_to_points(traj.poses, color=[255, 0, 0]),
            #         static=True,
            #     )

        if sort is None:
            pass
        elif sort.lower() == "atet":
            results = sorted(results, key=lambda x: x[0])
        elif sort.lower() == "ater":
            results = sorted(results, key=lambda x: x[1])

    print(tabulate(results, headers=header, tablefmt="fancy"))


def _contains_dir(directory: Path) -> bool:
    return any(directory.is_dir() for directory in directory.glob("*"))


def eval(directories: list[Path], visualize: bool, sort: Optional[str] = None):
    # Collect all bottom level directories
    bottom_level_dirs = []
    for directory in directories:
        for subdir in directory.glob("**/"):
            if not _contains_dir(subdir):
                bottom_level_dirs.append(subdir)

    for d in bottom_level_dirs:
        eval_dataset(d, visualize, sort)

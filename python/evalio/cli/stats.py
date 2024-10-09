from pathlib import Path
from attr import dataclass
from evalio.types import Stamp, SE3, SO3
from copy import deepcopy
from tabulate import tabulate

import numpy as np

import csv
import yaml


@dataclass(kw_only=True)
class Trajectory:
    metadata: dict
    stamps: list[Stamp]
    poses: list[SE3]

    def __getitem__(self, idx):
        return self.stamps[idx], self.poses[idx]

    def __len__(self):
        return len(self.stamps)


@dataclass(kw_only=True)
class Ate:
    trans: float
    rot: float


def load(path: str) -> Trajectory:
    fieldnames = ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"]

    poses = []
    stamps = []

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
            poses.append(pose)
            stamps.append(stamp)

    return Trajectory(metadata=metadata, stamps=stamps, poses=poses)


def align_stamps(traj1: Trajectory, traj2: Trajectory) -> tuple[Trajectory, Trajectory]:
    # Check if we need to skip poses in traj1
    first_pose_idx = 0
    while traj1.stamps[first_pose_idx] < traj2.stamps[0]:
        first_pose_idx += 1
    traj1.stamps = traj1.stamps[first_pose_idx:]
    traj1.poses = traj1.poses[first_pose_idx:]

    # Check if we need to skip poses in traj2
    first_pose_idx = 0
    while traj2.stamps[first_pose_idx] < traj1.stamps[0]:
        first_pose_idx += 1
    traj2.stamps = traj2.stamps[first_pose_idx:]
    traj2.poses = traj2.poses[first_pose_idx:]

    # Find the one that is at a higher frame rate
    # Leaves us with traj1 being the one with the higher frame rate
    swapped = False
    if traj1.stamps[1] - traj1.stamps[0] < traj2.stamps[1] - traj2.stamps[0]:
        traj1, traj2 = traj2, traj1
        swapped = True

    # Align the two trajectories by selectively keeping traj1 stamps
    traj1_idx = 0
    traj1_stamps = []
    traj1_poses = []
    for i, stamp in enumerate(traj2.stamps):
        while traj1_idx < len(traj1) - 1 and traj1.stamps[traj1_idx] < stamp:
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


def align_poses(traj: Trajectory, gt: Trajectory) -> Trajectory:
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

    error_t = 0
    error_r = 0
    for gt, pose in zip(gt_poses.poses, traj.poses):
        error_t += np.linalg.norm(gt.trans - pose.trans)
        error_r += np.linalg.norm((gt.rot * pose.rot.inverse()).log())

    error_t /= len(gt_poses)
    error_r /= len(gt_poses)

    return Ate(rot=error_r, trans=error_t)


def eval_dataset(dir: Path, visualize: bool):
    # Load all trajectories
    trajectories = []
    for file_path in dir.glob("*.csv"):
        traj = load(file_path)
        trajectories.append(traj)

    gt = []
    trajs = []
    for t in trajectories:
        (gt if t.metadata.get("gt", False) else trajs).append(t)

    assert len(gt) == 1, f"Found multiple ground truths in {dir}"
    gt_og = gt[0]

    # Setup visualization
    if visualize:
        import rerun as rr
        import evalio.vis as evis

        rr.init(
            str(dir),
            spawn=False,
        )
        rr.connect("0.0.0.0:9876")
        rr.log(
            "gt",
            evis.poses_to_points(gt_og.poses, color=[0, 0, 255]),
            static=True,
        )

    results = []
    for traj in trajs:
        traj, gt = align_stamps(traj, deepcopy(gt_og))
        align_poses(traj, gt)
        ate = compute_ate(traj, gt)
        results.append((traj.metadata["pipeline"], ate.trans, ate.rot))

        if visualize:
            rr.log(
                traj.metadata["pipeline"],
                evis.poses_to_points(traj.poses, color=[255, 0, 0]),
                static=True,
            )

    print(f"\nResults for {'/'.join(dir.parts[-2:])}")
    print(tabulate(results, headers=["Pipeline", "ATEt", "ATEr"], tablefmt="fancy"))


def eval(dir: Path, visualize: bool):
    # TODO: Detect if a single folder or if we should glob
    print("Evaluating experiments in", dir)

    # Glob over folders
    for dir in dir.glob("*/*"):
        if not dir.is_dir():
            continue
        eval_dataset(dir, visualize)

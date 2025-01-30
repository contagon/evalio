from dataclasses import dataclass

from evalio.types import Trajectory
import numpy as np

from .base import (
    EVALIO_DATA,
    SE3,
    SO3,
    Dataset,
    ImuParams,
    LidarParams,
    RosbagIter,
    load_pose_csv,
)


@dataclass
class NewerCollege2020(Dataset):
    # ------------------------- For loading data ------------------------- #
    def __iter__(self):
        return RosbagIter(
            EVALIO_DATA / NewerCollege2020.name() / self.seq,
            "/os1_cloud_node/points",
            "/os1_cloud_node/imu",
            self.lidar_params(),
        )

    def ground_truth_raw(self) -> Trajectory:
        # For some reason bag #7 is different
        if self.seq == "07_parkland_mound":
            return load_pose_csv(
                EVALIO_DATA / NewerCollege2020.name() / self.seq / "ground_truth.csv",
                ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"],
                delimiter=" ",
            )

        return load_pose_csv(
            EVALIO_DATA / NewerCollege2020.name() / self.seq / "ground_truth.csv",
            ["sec", "nsec", "x", "y", "z", "qx", "qy", "qz", "qw"],
        )

    # ------------------------- For loading params ------------------------- #
    @staticmethod
    def url() -> str:
        return "https://ori-drs.github.io/newer-college-dataset/stereo-cam/"

    @staticmethod
    def name() -> str:
        return "newer_college_2020"

    @staticmethod
    def sequences() -> list[str]:
        return [
            "01_short_experiment",
            "02_long_experiment",
            "05_quad_with_dynamics",
            "06_dynamic_spinning",
            "07_parkland_mound",
        ]

    def imu_T_lidar(self) -> SE3:
        return SE3(
            SO3(qx=0.0, qy=0.0, qz=1.0, qw=0.0),
            np.array(
                [0.006252999883145094, -0.011775000020861626, 0.007644999772310257]
            ),
        )

    def imu_T_gt(self) -> SE3:
        return SE3(
            SO3(qx=0.0, qy=0.0, qz=0.38268, qw=0.92388),
            np.array([0.035643, 0.089026, -0.021653]),
        )

    def imu_params(self) -> ImuParams:
        return ImuParams(
            gyro=0.000261799,
            accel=0.000230,
            gyro_bias=0.0000261799,
            accel_bias=0.0000230,
            bias_init=1e-7,
            integration=1e-7,
            gravity=np.array([0, 0, -9.81]),
        )

    def lidar_params(self) -> LidarParams:
        return LidarParams(
            num_rows=64,
            num_columns=1024,
            min_range=0.1,
            max_range=120.0,
        )

    # ------------------------- For downloading ------------------------- #
    @staticmethod
    def check_download(seq: str) -> bool:
        dir = EVALIO_DATA / NewerCollege2020.name() / seq
        # Check how many bag files it should have
        should_have = {
            "01_short_experiment": 10,
            "02_long_experiment": 16,
            "05_quad_with_dynamics": 3,
            "06_dynamic_spinning": 1,
            "07_parkland_mound": 3,
        }[seq]

        if not dir.exists():
            return False
        elif not (dir / "ground_truth.csv").exists():
            return False
        elif len(list(dir.glob("*.bag"))) != should_have:
            return False
        else:
            return True

    @staticmethod
    def download(seq: str):
        folder_id = {
            "01_short_experiment": "1WWtyU6bv4-JKwe-XuSeKEEEBhbgoFHRG",
            "02_long_experiment": "1pg3jzNF59YJX_lqVf4dcYI99TyBHcJX_",
            "05_quad_with_dynamics": "1ScfmWiRQ_nGy3Xj5VqRSpzkEJl5BHPQv",
            "06_dynamic_spinning": "1x1f_WfkQIf5AtdRhnWblhkPLur5_5ck0",
            "07_parkland_mound": "1PAywZT8T9TbKy_XJEgWXJkFvr5C6M1pS",
        }[seq]

        gt_url = {
            "01_short_experiment": "11VWvHxjitd4ijARD4dJ3WjFuZ_QbInVy",
            "02_long_experiment": "1fT1_MhFkCn_RWzLTzo4i-sjoKa_TbIUW",
            "05_quad_with_dynamics": "1Cc7fiYUCtNL8qnvA0x-m4uQvRWQLdrWO",
            "06_dynamic_spinning": "16lLgl2iqVs5qSz-N3OZv9bZWBbvAXyP3",
            "07_parkland_mound": "1CMcmw9pAT1Mm-Zh-nS87i015CO-xFHwl",
        }[seq]

        import gdown  # type: ignore

        folder = EVALIO_DATA / NewerCollege2020.name() / seq

        print(f"Downloading {seq} to {folder}...")
        folder.mkdir(parents=True, exist_ok=True)
        gdown.download(id=gt_url, output=str(folder / "ground_truth.csv"), resume=True)
        gdown.download_folder(id=folder_id, output=str(folder), resume=True)

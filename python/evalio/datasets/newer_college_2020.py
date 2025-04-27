from evalio.datasets.loaders import (
    LidarDensity,
    LidarFormatParams,
    LidarMajor,
    LidarPointStamp,
    LidarStamp,
    RosbagIter,
)
from evalio.types import Trajectory, SO3
import numpy as np
from enum import auto
import os

from .base import (
    SE3,
    Dataset,
    ImuParams,
    LidarParams,
    DatasetIterator,
)

from pathlib import Path
from typing import Sequence


class NewerCollege2020(Dataset):
    short_experiment = auto()
    long_experiment = auto()
    quad_with_dynamics = auto()
    dynamic_spinning = auto()
    parkland_mound = auto()

    # ------------------------- For loading data ------------------------- #
    def data_iter(self) -> DatasetIterator:
        # Use Ouster IMU as lidar IMU since the realsense IMU is not time-synced
        return RosbagIter(
            self.folder,
            "/os1_cloud_node/points",
            "/os1_cloud_node/imu",
            self.lidar_params(),
            lidar_format=LidarFormatParams(
                stamp=LidarStamp.Start,
                point_stamp=LidarPointStamp.Start,
                major=LidarMajor.Column,
                density=LidarDensity.AllPoints,
            ),
        )

    def ground_truth_raw(self) -> Trajectory:
        # For some reason bag parkland mound is different
        if self.seq_name == "parkland_mound":
            return Trajectory.load_csv(
                self.folder / "registered_poses.csv",
                ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"],
                delimiter=" ",
            )

        return Trajectory.load_csv(
            self.folder / "registered_poses.csv",
            ["sec", "nsec", "x", "y", "z", "qx", "qy", "qz", "qw"],
        )

    # ------------------------- For loading params ------------------------- #
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
            brand="TDK",
            model="ICM-20948",
        )

    def lidar_params(self) -> LidarParams:
        return LidarParams(
            num_rows=64,
            num_columns=1024,
            min_range=0.1,
            max_range=120.0,
            brand="Ouster",
            model="OS1-64",
        )

    # ------------------------- dataset info ------------------------- #
    @staticmethod
    def url() -> str:
        return "https://ori-drs.github.io/newer-college-dataset/stereo-cam/"

    def environment(self) -> str:
        return "Oxford Campus"

    def vehicle(self) -> str:
        return "Handheld"

    # ------------------------- For downloading ------------------------- #
    def files(self) -> Sequence[str | Path]:
        return {
            "dynamic_spinning": [
                "rooster_2020-07-10-09-23-18_0.bag",
            ],
            "short_experiment": [
                "rooster_2020-03-10-10-36-30_0.bag",
                "rooster_2020-03-10-10-39-18_1.bag",
                "rooster_2020-03-10-10-42-05_2.bag",
                "rooster_2020-03-10-10-44-52_3.bag",
                "rooster_2020-03-10-10-47-39_4.bag",
                "rooster_2020-03-10-10-50-26_5.bag",
                "rooster_2020-03-10-10-53-13_6.bag",
                "rooster_2020-03-10-10-56-00_7.bag",
                "rooster_2020-03-10-10-58-47_8.bag",
                "rooster_2020-03-10-11-01-34_9.bag",
            ],
            "long_experiment": [
                "rooster_2020-03-10-11-36-51_0.bag",
                "rooster_2020-03-10-11-39-38_1.bag",
                "rooster_2020-03-10-11-42-25_2.bag",
                "rooster_2020-03-10-11-45-12_3.bag",
                "rooster_2020-03-10-11-47-59_4.bag",
                "rooster_2020-03-10-11-50-46_5.bag",
                "rooster_2020-03-10-11-53-33_6.bag",
                "rooster_2020-03-10-11-56-20_7.bag",
                "rooster_2020-03-10-11-59-07_8.bag",
                "rooster_2020-03-10-12-01-54_9.bag",
                "rooster_2020-03-10-12-04-41_10.bag",
                "rooster_2020-03-10-12-07-28_11.bag",
                "rooster_2020-03-10-12-10-15_12.bag",
                "rooster_2020-03-10-12-13-02_13.bag",
                "rooster_2020-03-10-12-15-49_14.bag",
                "rooster_2020-03-10-12-18-36_15.bag",
            ],
            "quad_with_dynamics": [
                "rooster_2020-07-10-09-13-52_0.bag",
                "rooster_2020-07-10-09-16-39_1.bag",
                "rooster_2020-07-10-09-19-26_2.bag",
            ],
            "parkland_mound": [
                "rooster_2020-07-10-09-31-24_0.bag",
                "rooster_2020-07-10-09-34-11_1.bag",
                "rooster_2020-07-10-09-36-58_2.bag",
            ],
        }[self.seq_name] + ["registered_poses.csv"]

    def download(self):
        folder_id = {
            "short_experiment": "1WWtyU6bv4-JKwe-XuSeKEEEBhbgoFHRG",
            "long_experiment": "1pg3jzNF59YJX_lqVf4dcYI99TyBHcJX_",
            "quad_with_dynamics": "1ScfmWiRQ_nGy3Xj5VqRSpzkEJl5BHPQv",
            "dynamic_spinning": "1x1f_WfkQIf5AtdRhnWblhkPLur5_5ck0",
            "parkland_mound": "1PAywZT8T9TbKy_XJEgWXJkFvr5C6M1pS",
        }[self.seq_name]

        gt_url = {
            "short_experiment": "11VWvHxjitd4ijARD4dJ3WjFuZ_QbInVy",
            "long_experiment": "1fT1_MhFkCn_RWzLTzo4i-sjoKa_TbIUW",
            "quad_with_dynamics": "1Cc7fiYUCtNL8qnvA0x-m4uQvRWQLdrWO",
            "dynamic_spinning": "16lLgl2iqVs5qSz-N3OZv9bZWBbvAXyP3",
            "parkland_mound": "1CMcmw9pAT1Mm-Zh-nS87i015CO-xFHwl",
        }[self.seq_name]

        import gdown  # type: ignore

        print(f"Downloading to {self.folder}...")

        self.folder.mkdir(parents=True, exist_ok=True)
        gdown.download(id=gt_url, output=f"{self.folder}{os.sep}", resume=True)
        gdown.download_folder(id=folder_id, output=str(self.folder), resume=True)

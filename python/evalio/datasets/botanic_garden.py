from enum import auto

from evalio.datasets.iterators import (
    LidarDensity,
    LidarFormatParams,
    LidarMajor,
    LidarPointStamp,
    LidarStamp,
    RosbagIter,
)
from evalio._cpp._helpers import fill_col_split_row_velodyne  # type: ignore
from evalio.types import Trajectory
import numpy as np

from .base import (
    SE3,
    Dataset,
    ImuParams,
    LidarParams,
    load_pose_csv,
    DatasetIterator,
)


class BotanicGarden(Dataset):
    b1005_00 = auto()
    b1005_01 = auto()
    b1005_07 = auto()
    b1006_01 = auto()
    b1008_03 = auto()
    b1018_00 = auto()
    b1018_13 = auto()

    # ------------------------- For loading data ------------------------- #
    def data_iter(self) -> DatasetIterator:
        return RosbagIter(
            self.folder / f"{self.seq_name}.bag",
            "/velodyne_points",
            "/imu/data",
            self.lidar_params(),
            lidar_format=LidarFormatParams(
                stamp=LidarStamp.End,
                point_stamp=LidarPointStamp.End,
                major=LidarMajor.Column,
                density=LidarDensity.OnlyValidPoints,
            ),
            custom_col_func=fill_col_split_row_velodyne,
        )

    def ground_truth_raw(self) -> Trajectory:
        if self.seq_name == "1008_03":
            filename = f"{self.seq_name}_gt_output.txt"
        else:
            filename = f"{self.seq_name}_GT_output.txt"

        return load_pose_csv(
            self.folder / filename,
            ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"],
            delimiter=" ",
        )

    # ------------------------- For loading params ------------------------- #
    @staticmethod
    def url() -> str:
        return "https://github.com/robot-pesg/BotanicGarden"

    def imu_T_lidar(self) -> SE3:
        # https://github.com/robot-pesg/BotanicGarden/blob/main/calib/extrinsics/calib_chain.yaml
        return SE3.fromMat(
            np.array(
                [
                    [
                        0.999678872580465,
                        0.0252865664429322,
                        0.00150422292234868,
                        0.0584867781527745,
                    ],
                    [
                        -0.0252723438960774,
                        0.999649431893338,
                        -0.0078025434141585,
                        0.00840419966766332,
                    ],
                    [
                        -0.00170103929405540,
                        0.00776298237926191,
                        0.99996789371916,
                        0.168915521980526,
                    ],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            )
        )

    def imu_T_gt(self) -> SE3:
        # I believe ground truth is in the lidar frame
        # https://github.com/robot-pesg/BotanicGarden/tree/main?tab=readme-ov-file#evaluation
        return self.imu_T_lidar()

    def imu_params(self) -> ImuParams:
        # Xsens Mti-680G
        # https://github.com/robot-pesg/BotanicGarden/blob/main/calib/imu_intrinsics/xsens_imu_param.yaml
        return ImuParams(
            gyro=0.0002,
            accel=0.0005,
            gyro_bias=0.0004,
            accel_bias=0.0002,
            bias_init=1e-8,
            integration=1e-8,
            gravity=np.array([0, 0, -9.81]),
        )

    def lidar_params(self) -> LidarParams:
        return LidarParams(
            num_rows=16,
            num_columns=1825,
            min_range=0.1,
            max_range=100.0,
        )

    def files(self) -> list[str]:
        raise NotImplementedError

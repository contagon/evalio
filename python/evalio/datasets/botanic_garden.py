from dataclasses import dataclass

from evalio._cpp._helpers import split_row_pc2_to_evalio  # type: ignore
from evalio.types import Trajectory
import numpy as np
from pathlib import Path

from .base import (
    EVALIO_DATA,
    SE3,
    Dataset,
    ImuParams,
    LidarParams,
    RosbagIter,
    load_pose_csv,
    Measurement,
    ImuMeasurement,
    LidarMeasurement,
    Stamp,
)


# The botanic garden dataset returns stamps at the end of the scan
# Offset so they are actually taken from the start of the scan
# Additionally, the point offsets are negative
# We hackily add five to them earlier, subtract that back off now
class OffsetRosbagIter(RosbagIter):
    def __init__(
        self,
        path: Path,
        lidar_topic: str,
        imu_topic: str,
        params,
        is_mcap: bool = False,
        cpp_point_loader=None,
    ):
        super().__init__(
            path, lidar_topic, imu_topic, params, is_mcap, cpp_point_loader
        )

    def __next__(self) -> Measurement:
        msg = super().__next__()
        if isinstance(msg, ImuMeasurement):
            return msg
        elif isinstance(msg, LidarMeasurement):
            msg.stamp = Stamp.from_sec(msg.stamp.to_sec() - 0.1)
            return msg
        else:
            raise ValueError("Unknown message type")


@dataclass
class BotanicGarden(Dataset):
    # ------------------------- For loading data ------------------------- #
    def __iter__(self):
        return OffsetRosbagIter(
            EVALIO_DATA / BotanicGarden.name() / self.seq / f"{self.seq[1:]}.bag",
            "/velodyne_points",
            "/imu/data",
            self.lidar_params(),
            cpp_point_loader=split_row_pc2_to_evalio,
        )

    def ground_truth_raw(self) -> Trajectory:
        if self.seq == "b1008_03":
            filename = f"{self.seq[1:]}_gt_output.txt"
        else:
            filename = f"{self.seq[1:]}_GT_output.txt"

        return load_pose_csv(
            EVALIO_DATA / BotanicGarden.name() / self.seq / filename,
            ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"],
            delimiter=" ",
        )

    # ------------------------- For loading params ------------------------- #
    @staticmethod
    def url() -> str:
        return "https://github.com/robot-pesg/BotanicGarden"

    @staticmethod
    def name() -> str:
        return "botanic_garden"

    @staticmethod
    def sequences() -> list[str]:
        return [
            "b1005_00",
            "b1005_01",
            "b1005_07",
            "b1006_01",
            "b1008_03",
            "b1018_00",
            "b1018_13",
        ]

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

    # ------------------------- For downloading ------------------------- #
    @staticmethod
    def check_download(seq: str) -> bool:
        # dir = EVALIO_DATA / BotanicGarden.name() / seq
        # if not dir.exists():
        #     return False
        # elif not (dir / f"{seq}.bag").exists():
        #     return False
        # elif not (
        #     (dir / f"{seq}_GT_output.txt").exists()
        #     or (dir / f"{seq}_gt_output.txt").exists()
        # ):
        #     return False
        # else:
        return True

import numpy as np

from .base import (
    EVALIO_DATA,
    Dataset,
    RosbagIter,
    ImuParams,
    LidarParams,
    load_pose_csv,
    SO3,
    SE3,
    Stamp,
)


class NewerCollege2021(Dataset):
    def __init__(self, seq: str):
        self.seq = self.process_seq(seq)

        raise ValueError("This dataset isn't completed yet")

        if not self.check_download():
            print(f"Data for {self.seq} not found, downloading now")
            self.download()

    def __iter__(self):
        return RosbagIter(
            EVALIO_DATA / NewerCollege2021.name() / self.seq,
            "/os1_cloud_node/points",
            "/os1_cloud_node/imu",
        )

    def ground_truth(self) -> list[(Stamp, SE3)]:
        return load_pose_csv(
            EVALIO_DATA / NewerCollege2021.name() / self.seq / "ground_truth.csv"
        )

    def check_download(self) -> bool:
        dir = EVALIO_DATA / NewerCollege2021.name() / self.seq
        # Check how many bag files it should have
        should_have = {
            "01_short_experiment": 10,
            "02_long_experiment": 16,
            "05_quad_with_dynamics": 3,
            "06_dynamic_spinning": 1,
            "07_parkland_mound": 3,
        }[self.seq]

        if not dir.exists():
            return False
        elif dir / "ground_truth.csv":
            return False
        elif len(list(dir.glob("*.bag"))) != should_have:
            return False
        else:
            return True

    def download(self):
        folder_id = {
            "01_short_experiment": "1WWtyU6bv4-JKwe-XuSeKEEEBhbgoFHRG",
            "02_long_experiment": "1pg3jzNF59YJX_lqVf4dcYI99TyBHcJX_",
            "05_quad_with_dynamics": "1ScfmWiRQ_nGy3Xj5VqRSpzkEJl5BHPQv",
            "06_dynamic_spinning": "1x1f_WfkQIf5AtdRhnWblhkPLur5_5ck0",
            "07_parkland_mound": "1PAywZT8T9TbKy_XJEgWXJkFvr5C6M1pS",
        }[self.seq]

        gt_url = {
            "01_short_experiment": "11VWvHxjitd4ijARD4dJ3WjFuZ_QbInVy",
            "02_long_experiment": "1fT1_MhFkCn_RWzLTzo4i-sjoKa_TbIUW",
            "05_quad_with_dynamics": "1Cc7fiYUCtNL8qnvA0x-m4uQvRWQLdrWO",
            "06_dynamic_spinning": "16lLgl2iqVs5qSz-N3OZv9bZWBbvAXyP3",
            "07_parkland_mound": "1CMcmw9pAT1Mm-Zh-nS87i015CO-xFHwl",
        }[self.seq]

        import gdown

        folder = EVALIO_DATA / NewerCollege2021.name() / self.seq

        print(f"Making folder {folder}...")
        folder.mkdir(parents=True, exist_ok=True)

        print(f"Downloading {self.seq} to {folder}...")
        gdown.download(id=gt_url, output=str(folder / "ground_truth.csv"))
        gdown.download_folder(id=folder_id, output=str(folder))

    # ------------------------- Static Methods ------------------------- #
    @staticmethod
    def name() -> str:
        return "newer_college_2021"

    @staticmethod
    def nickname() -> str:
        return "nc21"

    @staticmethod
    def sequences() -> list[str]:
        return [
            "01_short_experiment",
            "02_long_experiment",
            "05_quad_with_dynamics",
            "06_dynamic_spinning",
            "07_parkland_mound",
        ]

    @staticmethod
    def nicksequences() -> list[str]:
        return ["01", "02", "05", "06", "07"]

    @staticmethod
    def imu_T_lidar() -> SE3:
        return SE3(
            SO3(qx=0.0, qy=0.0, qz=1.0, qw=0.0),
            [0.006252999883145094, -0.011775000020861626, 0.007644999772310257],
        )

    @staticmethod
    def imu_T_gt() -> SE3:
        return SE3(
            SO3(qx=0.0, qy=0.0, qz=0.38268, qw=0.92388),
            [0.035643, 0.089026, -0.021653],
        )

    @staticmethod
    def imu_params() -> ImuParams:
        return ImuParams(
            gyro=0.000261799,
            accel=0.000230,
            gyro_bias=0.0000261799,
            accel_bias=0.0000230,
            bias_init=1e-7,
            integration=1e-7,
            gravity=np.array([0, 0, 9.81]),
        )

    @staticmethod
    def lidar_params() -> LidarParams:
        return LidarParams(
            num_rows=64,
            num_columns=1024,
            min_range=0.1,
            max_range=120.0,
        )

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

"""
As a reference, we use the built in Ouster IMU instead of the alphasense one
Extrinsics are more likely to be accurate
Also, the alphasense IMU (Bosch BMI085) has fairly similar specs to the Ouster one (ICM-20948)

"""


@dataclass
class NewerCollege2021(Dataset):
    # ------------------------- For loading data ------------------------- #
    def __iter__(self):
        return RosbagIter(
            EVALIO_DATA / NewerCollege2021.name() / self.seq,
            "/os_cloud_node/points",
            "/os_cloud_node/imu",
            self.lidar_params(),
        )

    def ground_truth_raw(self) -> Trajectory:
        return load_pose_csv(
            EVALIO_DATA / NewerCollege2021.name() / self.seq / "ground_truth.csv",
            ["sec", "nsec", "x", "y", "z", "qx", "qy", "qz", "qw"],
        )

    # ------------------------- For loading params ------------------------- #
    @staticmethod
    def url() -> str:
        return "https://ori-drs.github.io/newer-college-dataset/multi-cam/"

    @staticmethod
    def name() -> str:
        return "newer_college_2021"

    @staticmethod
    def sequences() -> list[str]:
        return [
            "quad-easy",
            "quad-medium",
            "quad-hard",
            "stairs",
            "cloister",
            "park",
            "maths-easy",
            "maths-medium",
            "maths-hard",
        ]

    def imu_T_lidar(self) -> SE3:
        return SE3(
            SO3(qx=0.0032925, qy=-0.004627, qz=-0.0024302, qw=0.99998),
            np.array([0.013801, -0.012207, -0.01514]),
        )

    def imu_T_gt(self) -> SE3:
        return SE3(
            SO3(qx=0.0032925, qy=-0.004627, qz=-0.0024302, qw=0.99998),
            np.array([0.013642, -0.011607, -0.10583]),
        )

    def imu_params(self) -> ImuParams:
        # ICM-20948
        # https://invensense.tdk.com/wp-content/uploads/2024/03/DS-000189-ICM-20948-v1.6.pdf
        return ImuParams(
            gyro=0.000261799387799,
            accel=0.0022563,
            gyro_bias=0.0000261799387799,
            accel_bias=0.00022563,
            bias_init=1e-8,
            integration=1e-8,
            gravity=np.array([0, 0, -9.81]),
        )

    def lidar_params(self) -> LidarParams:
        return LidarParams(
            num_rows=128,
            num_columns=1024,
            min_range=0.1,
            max_range=50.0,
        )

    # ------------------------- For downloading ------------------------- #
    @staticmethod
    def check_download(seq: str) -> bool:
        # TODO:
        dir = EVALIO_DATA / NewerCollege2021.name() / seq
        # Check how many bag files it should have
        should_have = {
            "quad-easy": 1,
            "quad-medium": 1,
            "quad-hard": 1,
            "stairs": 1,
            "cloister": 2,
            "park": 8,
            "maths-easy": 2,
            "maths-medium": 1,
            "maths-hard": 2,
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
    def download(seq: str) -> None:
        # TODO:
        bag_ids = {
            "quad-easy": ["1hF2h83E1THbFAvs7wpR6ORmrscIHxKMo"],
            "quad-medium": ["11bZfJce1MCM4G9YUTCyUifM715N7FSbO"],
            "quad-hard": ["1ss6KPSTZ4CRS7uHAMgqnd4GQ6tKEEiZD"],
            "stairs": ["1ql0C8el5PJs6O0x4xouqaW9n2RZy53q9"],
            "cloister": [
                "1zzX_ZrMkVOtpSoD2jQg6Gdrtv8-UjYbF",
                "1QNFQSb81gG1_jX338vO7RQkScKGT_hf1",
            ],
            "park": [
                "1KZo-gPVQTMJ4hRaiaqV3hfuVNaONScDp",
                "1eGVPwFSaG0M2M7Lci6IBjKQrEf1uqtVn",
                "1nhuoH0OcLbovbXkq3eW6whk6TIKk2SEu",
                "1pXBE1iD9iivFFliFFNs7uvWi1zjo1S0s",
                "1_eZ5i2CGL7fNHeowRd1P_sllX1nxGwGL",
                "1wMCZVbB7eaSuz6u3ObSTdGbgbRFyRQ7m",
                "10o1oR7guReYKiVk3nBiPrFWp1MnERTiH",
                "1VpLV_WUJqr770NBjF-O-DrXb5dhXRWyM",
            ],
            "maths-easy": [
                "1wRnRSni9bcBRauJEJ80sxHIaJaonrC3C",
                "1ORkYwGpQNvD48WRXICDDecbweg8MxYA8",
            ],
            "maths-medium": ["1IOq2e8Nfx79YFauBHCgdBSW9Ur5710GD"],
            "maths-hard": [
                "1qD147UWPo30B9lK3gABggCOxSAU6Pop2",
                "1_Mps_pCeUz3ZOc53lj6Hy_zDeJfPAqy8",
            ],
        }[seq]

        gt_ids = {
            "quad-easy": "1BdQiOhb_NW7VqjNtbbRAjI0JH56uKFI-",
            "quad-medium": "18aHhzTcVzXsppmk2WpiZnJhzOfREUwYP",
            "quad-hard": "1KMAG65pH8PsHUld-hTkFK4-SIIBqT5yP",
            "stairs": "17q_NYxn1SLBmUq20jgljO8HSlFF9LjDs",
            "cloister": "15I8qquSPWlySuY5_4ZBa_wL4UC7c-rQ7",
            "park": "1AkJ7lm5x2WdS3aGhKwe1PnUn6w0rbUjf",
            "maths-easy": "1dq1PqMODQBb4Hkn82h2Txgf5ZygS5udp",
            "maths-medium": "1H1U3aXv2AJQ_dexTnjaHIfzYfx8xVXpS",
            "maths-hard": "1Rb2TBKP7ISC2XzDGU68ix5lFjEB6jXeX",
        }[seq]

        import gdown  # type: ignore

        folder = EVALIO_DATA / NewerCollege2021.name() / seq

        print(f"Downloading {seq} to {folder}...")
        folder.mkdir(parents=True, exist_ok=True)
        gdown.download(id=gt_ids, output=str(folder / "ground_truth.csv"), resume=True)
        for bid in bag_ids:
            gdown.download(id=bid, output=str(folder) + "/", resume=True)

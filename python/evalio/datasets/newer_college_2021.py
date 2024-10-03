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
    # ------------------------- For loading data ------------------------- #
    def __iter__(self):
        return RosbagIter(
            EVALIO_DATA / NewerCollege2021.name() / self.seq,
            "/os1_cloud_node/points",
            "/os1_cloud_node/imu",
        )

    def ground_truth(self) -> list[(Stamp, SE3)]:
        # TODO: Verify this
        return load_pose_csv(
            EVALIO_DATA / NewerCollege2021.name() / self.seq / "ground_truth.csv"
        )

    # ------------------------- For loading params ------------------------- #
    @staticmethod
    def name() -> str:
        return "newer_college_2021"

    @staticmethod
    def nickname() -> str:
        return "nc21"

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

    @staticmethod
    def nicksequences() -> list[str]:
        return ["q1", "q2", "q3", "s", "c", "p", "m1", "m2", "m3"]

    @staticmethod
    def imu_T_lidar() -> SE3:
        return SE3(
            SO3(qx=0.0032925, qy=-0.004627, qz=-0.0024302, qw=0.99998),
            [0.013801, -0.012207, -0.01514],
        )

    @staticmethod
    def imu_T_gt() -> SE3:
        return SE3(
            SO3(qx=0.0032925, qy=-0.004627, qz=-0.0024302, qw=0.99998),
            [0.013642, -0.011607, -0.10583],
        )

    @staticmethod
    def imu_params() -> ImuParams:
        # TODO:
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
            num_rows=128,
            num_columns=1024,
            min_range=0.0,
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
            "cloister": 0,
            "park": 0,
            "maths-easy": 2,
            "maths-medium": 1,
            "maths-hard": 2,
        }[seq]

        if not dir.exists():
            return False
        elif dir / "ground_truth.csv":
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

        import gdown

        folder = EVALIO_DATA / NewerCollege2021.name() / seq

        print(f"Making folder {folder}...")
        folder.mkdir(parents=True, exist_ok=True)

        print(f"Downloading {seq} to {folder}...")
        gdown.download(id=gt_ids, output=str(folder / "ground_truth.csv"), resume=True)
        for bid in bag_ids:
            gdown.download(id=bid, output=str(folder), resume=True)

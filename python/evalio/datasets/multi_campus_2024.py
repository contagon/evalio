from dataclasses import dataclass
from pathlib import Path

from evalio.types import LidarMeasurement, ImuMeasurement, Stamp
from evalio.types import Trajectory
import numpy as np

from .base import (
    EVALIO_DATA,
    SE3,
    Dataset,
    ImuParams,
    LidarParams,
    Measurement,
    RosbagIter,
    load_pose_csv,
)


# The multi-campus dataset lidar scan timestamps are taken at the end of the scan
# Offset so they are actually taken from the start of the scan
class OffsetRosbagIter(RosbagIter):
    def __init__(
        self,
        path: Path,
        lidar_topic: str,
        imu_topic: str,
        params,
        is_mcap: bool = False,
    ):
        super().__init__(path, lidar_topic, imu_topic, params, is_mcap)

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
class MultiCampus2024(Dataset):
    # ------------------------- For loading data ------------------------- #
    def __iter__(self):
        # The NTU sequences use the ATV platform and a VectorNav vn100 IMU
        if "ntu" in self.seq:
            return OffsetRosbagIter(
                EVALIO_DATA / MultiCampus2024.name() / self.seq,
                "/os_cloud_node/points",
                "/vn100/imu",
                self.lidar_params(),
            )
        # The KTH and TUHH sequences use the hand-held platform and a VectorNav vn200 IMU
        elif "kth" in self.seq or "tuhh" in self.seq:
            return OffsetRosbagIter(
                EVALIO_DATA / MultiCampus2024.name() / self.seq,
                "/os_cloud_node/points",
                "/vn200/imu",
                self.lidar_params(),
            )
        else:
            raise ValueError(f"Unknown sequence: {self.seq}")

    def ground_truth_raw(self) -> Trajectory:
        return load_pose_csv(
            EVALIO_DATA / MultiCampus2024.name() / self.seq / "pose_inW.csv",
            ["num", "t", "x", "y", "z", "qx", "qy", "qz", "qw"],
            skip_lines=1,
        )

    # ------------------------- For loading params ------------------------- #
    @staticmethod
    def url() -> str:
        return "https://mcdviral.github.io/"

    @staticmethod
    def name() -> str:
        return "multi_campus_2024"

    @staticmethod
    def sequences() -> list[str]:
        return [
            "ntu_day_01",
            "ntu_day_02",
            "ntu_day_10",
            "ntu_night_04",
            "ntu_night_08",
            "ntu_night_13",
            "kth_day_06",
            "kth_day_09",
            "kth_day_10",
            "kth_night_01",
            "kth_night_04",
            "kth_night_05",
            "tuhh_day_02",
            "tuhh_day_03",
            "tuhh_day_04",
            "tuhh_night_07",
            "tuhh_night_08",
            "tuhh_night_09",
        ]

    def imu_T_lidar(self) -> SE3:
        # The NTU sequences use the ATV platform
        # Taken from calib file at: https://mcdviral.github.io/Download.html#calibration
        if "ntu" in self.seq:
            return SE3.fromMat(
                np.array(
                    [
                        [
                            0.9999346552051229,
                            0.003477624535771754,
                            -0.010889970036688295,
                            -0.060649229060416594,
                        ],
                        [
                            0.003587143302461965,
                            -0.9999430279821171,
                            0.010053516443599904,
                            -0.012837544242408117,
                        ],
                        [
                            -0.010854387257665576,
                            -0.01009192338171122,
                            -0.999890161647627,
                            -0.020492606896077407,
                        ],
                        [0.0, 0.0, 0.0, 1.0],
                    ]
                )
            )
        # The KTH and TUHH sequences use the hand-held platform
        # Taken from calib file at: https://mcdviral.github.io/Download.html#calibration
        elif "kth" in self.seq or "tuhh" in self.seq:
            return SE3.fromMat(
                np.array(
                    [
                        [
                            0.9999135040741837,
                            -0.011166365511073898,
                            -0.006949579221822984,
                            -0.04894521120494695,
                        ],
                        [
                            -0.011356389542502144,
                            -0.9995453006865824,
                            -0.02793249526856565,
                            -0.03126929060348084,
                        ],
                        [
                            -0.006634514801117132,
                            0.02800900135032654,
                            -0.999585653686922,
                            -0.01755515794222565,
                        ],
                        [0.0, 0.0, 0.0, 1.0],
                    ]
                )
            )
        else:
            raise ValueError(f"Unknown sequence: {self.seq}")

    def imu_T_gt(self) -> SE3:
        return SE3.identity()

    def imu_params(self) -> ImuParams:
        # The NTU sequences use the ATV platform and a VectorNav vn100 IMU
        # The KTH and TUHH sequences use the hand-held platform and VectorNav vn200 IMU
        # Both the vn100 and vn200 have the same IMU specifications
        return ImuParams(
            gyro=0.000061087,  # VectorNav Datasheet
            accel=0.00137,  # VectorNav Datasheet
            gyro_bias=0.0000261799,  # TODO (dan) - Fix currently stolen from newer college
            accel_bias=0.0000230,  # TODO (dan) - Fix currently stolen from newer college
            bias_init=1e-7,
            integration=1e-7,
            gravity=np.array([0, 0, -9.81]),
        )
        # Note- Current estimates for imu bias should be pessimistic estimates

    def lidar_params(self) -> LidarParams:
        # The NTU sequences use the ATV platform and an Ouster OS1 - 128
        if "ntu" in self.seq:
            return LidarParams(
                num_rows=128,
                num_columns=1024,
                min_range=0.1,
                max_range=120.0,
            )
        # The KTH and TUHH sequences use the hand-held platform and an Ouster OS1 - 64
        elif "kth" in self.seq or "tuhh" in self.seq:
            return LidarParams(
                num_rows=64,
                num_columns=1024,
                min_range=0.1,
                max_range=120.0,
            )
        else:
            raise ValueError(f"Unknown sequence: {self.seq}")

    # ------------------------- For downloading ------------------------- #
    @staticmethod
    def check_download(seq: str) -> bool:
        dir = EVALIO_DATA / MultiCampus2024.name() / seq
        if not dir.exists():
            return False
        elif not (dir / "pose_inW.csv").exists():
            return False
        elif not (dir / "spline.csv").exists():
            return False
        elif len(list(dir.glob("*.bag"))) != 2:
            return False
        else:
            return True

    @staticmethod
    def download(seq: str):
        ouster_url = {
            "ntu_day_01": "127Rk2jX4I95CEWK1AOZRD9AQRxRVlWjY",
            "ntu_day_02": "1jDS84WvHCfM_L73EptXKp-BKPIPKoE0Z",
            "ntu_day_10": "1p18Fa5SXbVcCa9BJb_Ed8Fk_NRcahkCF",
            "ntu_night_04": "1k9olfETU3f3iq_9QenzEfjTpD56bOtaV",
            "ntu_night_08": "1BbtBDwT3sLCHCOFfZWeVVWbG72mWq8x8",
            "ntu_night_13": "17Fn_HRVwSEzQqXwkw0J3NnqxekUMjnYI",
            "kth_day_06": "1DHpRSoY5ysK1h2nRwks_6Sz-QZqERiXH",
            "kth_day_09": "1mhMpwr3NDYfUWL0dVAh_kCTTTLFen31C",
            "kth_day_10": "1NbOHfVaCZkXPz28VwLrWLfITXYn25odh",
            "kth_night_01": "1mbLMoTPdhUI9u-ZOYFQJOYgrcQJb3rvN",
            "kth_night_04": "1SRMbAu1UyA4lJB4hZdmY-0mic-paGkKF",
            "kth_night_05": "1m8DYu6y5BkolXkKqC9E8Lm77TpzpyeNR",
            "tuhh_day_02": "1LErPETriJjLWhMBE5jvfpxoFujn0Z3cp",
            "tuhh_day_03": "1zTU8dnYNn1WRBGY-YkzqEiofH11vryTu",
            "tuhh_day_04": "1IFzZoEyqjboOwntyiPHTUxGcBndE2e9S",
            "tuhh_night_07": "1y1GJkaofleWVU8ZoUByGkmXkq2lwm-k-",
            "tuhh_night_08": "16t33lVBzbSxrtt0vFt-ztWAxiciONWTX",
            "tuhh_night_09": "1_FsTTQe-NKvQ-1shlYNeG0uWqngA2XzC",
        }[seq]

        imu_url = {
            "ntu_day_01": "1bBKRlzwG4v7K4mBmLAQzfwp_O6yOR0Ld",
            "ntu_day_02": "1FHsJ1Hosn_j4m5KivJrdtECdFEj3Is0G",
            "ntu_day_10": "14IydATXlqbJ0333iNY7H-bFDBBBYF-nC",
            "ntu_night_04": "1dLvaCBmac-05QtPy-ZsiU6L5gY35Z_ii",
            "ntu_night_08": "1oTUfLaQO9sUjesg6Bn3xbSZt3XgQqVRo",
            "ntu_night_13": "1lru1JVyjfzM_QmctEzMtgD6ps8ib5xYs",
            "kth_day_06": "1cf_dmcFAX9-5zxB8WcFVc3MaVNczEMqn",
            "kth_day_09": "16j2Ud99lrgkNtIlPQ_OV6caqZZc-bHA-",
            "kth_day_10": "13qyhDyrj6doa7s0cdbtF1e_Bh-erFMUv",
            "kth_night_01": "1RMfF_DYxUkP6ImwCK039-qJpzbGKw_m7",
            "kth_night_04": "10KIUpaJIID293P3um8OfWWiiQ1NArj2o",
            "kth_night_05": "1_LvH-KVfBOW4ltSo8ERLEHWRb31OoAgW",
            "tuhh_day_02": "1N3l-HskmBkta4OQVAneqnJhU29-6IeK8",
            "tuhh_day_03": "12SJQrHjFKNUMeoNuXNh7l0gd1w--B5Vl",
            "tuhh_day_04": "1EToB3VXrxmoyPtdL1bnlFgG-fcegAIOt",
            "tuhh_night_07": "1Ngy1_UXOfhjhwr-BEpG6Rsh1gi1rrMho",
            "tuhh_night_08": "1bDjyQLINKWBVOg_7Q1n1mooUfM3VifOu",
            "tuhh_night_09": "1jVQTmFX2pnYNULU5CjbOVa6hp_7zQoez",
        }[seq]

        gt_url = {
            "ntu_day_01": "1Pdj4_0SRES4v9WiyCVp8dYMcRvE8X3iH",
            "ntu_day_02": "1fB-AJx6jRwEWhJ0jVLlWkc38PpKCMTNy",
            "ntu_day_10": "11DKcJWgMFjuJlvp3Ez6bFpwtTvq42JBY",
            "ntu_night_04": "1mF-fd-NRMOpx_2jhuJeiOnxKTGYLQFsx",
            "ntu_night_08": "1vTnLttDiUdLr2mSxKyKmixFENwGWAEZU",
            "ntu_night_13": "15eHWp4sfJk4inD5u3EoFjDRxWJQ6e4Dd",
            "kth_day_06": "1ilY5Krkp9E4TtFS6WD2jrhvbIqWlxk5Z",
            "kth_day_09": "1OBfXm4GS52vWGn8cAKe_FHng91GQqg7w",
            "kth_day_10": "11cdWjQ5TXHD6cDBpTsMZbeKbBeDmKeLf",
            "kth_night_01": "1zued8z-H5Qav3W2f1Gz6YU_JnzmRdedc",
            "kth_night_04": "1G6qigMKh0aUZpbwRD0a3BdB_KI0vH0cZ",
            "kth_night_05": "1HfSMwGyzAndgO66H2mpxT3IG_SZnCExC",
            "tuhh_day_02": "1PXKc0wglgSxMBxqTGOFPQvJ4abeYHmFa",
            "tuhh_day_03": "1W53_HhhNlyf8Xc185Sd171k7RXFXln0n",
            "tuhh_day_04": "1yZJdd3EekbzoZkIH4-b7lfRa3IFSpFiO",
            "tuhh_night_07": "1QDQflr2OLCNJZ1dNUWfULICf70VhV0bt",
            "tuhh_night_08": "1bF-uj8gw7HkBXzvWXwtDNS-BBbEtuKrb",
            "tuhh_night_09": "1xr5dTBydbjIhE42hNdELklruuhxgYkld",
        }[seq]

        gt_spline_url = {
            "ntu_day_01": "13PwhWOuIEJmaCtWY8JvZAJ-uTA4vcmV3",
            "ntu_day_02": "19j_KBh8Tmfhi6DALY-soZDd90bsXy35X",
            "ntu_day_10": "1AR0H2c3MYlU3qYMCrS6CuUlwJ0QkP2mJ",
            "ntu_night_04": "1oOiSH6l6Au_9HNlw7wtKri_hKil8syJY",
            "ntu_night_08": "1rNXsVACfYFGzApQVk4s0nJoau53Y6Kn5",
            "ntu_night_13": "1uXqwSo_PqsqNQT4LC7-3p9OCSIeeWZQn",
            "kth_day_06": "1omeDSQt5XowK4F9dM0ur5PSmjZfDrVgS",
            "kth_day_09": "1ZkqJ9mmGhcbbkwHWoSNYoJP1TKw5ChXF",
            "kth_day_10": "1lSQgTH4cgY77c-Mw7PGqxndOj7v7vhfg",
            "kth_night_01": "15rKuojnx1apTA7C1XkzFLBkrY5jp_9gS",
            "kth_night_04": "1X7i89QWFsl0UkYtIfJYBXXBz9exPAAeI",
            "kth_night_05": "1tdHyg4tmFrJiP17cCP-cDaGcgS69G4Pg",
            "tuhh_day_02": "1mWwXjk7rCOJytMH8809hrvy5i-wqnwoL",
            "tuhh_day_03": "19rvvdCcE-o5bZr-1UGWXN9GBz4YHC7TX",
            "tuhh_day_04": "1EXQI70rvFbJkvZ8Fww6PvcQxwJsJAo1W",
            "tuhh_night_07": "1CIhyAK7bqwrdj3YoGWBc_AvtBDMY3pAi",
            "tuhh_night_08": "1fWRpwboPgkk1KVrVKB_4Vu-nVe3YObuX",
            "tuhh_night_09": "1ERnoR7Thtmsib6yu_4GACUODSyzyMLCk",
        }[seq]

        import gdown  # type: ignore

        folder = EVALIO_DATA / MultiCampus2024.name() / seq

        print(f"Downloading {seq} to {folder}...")
        folder.mkdir(parents=True, exist_ok=True)

        gdown.download(id=gt_url, output=str(folder / "pose_inW.csv"), resume=True)
        gdown.download(id=gt_spline_url, output=str(folder / "spline.csv"), resume=True)
        gdown.download(id=ouster_url, output=str(folder / "ouster.bag"), resume=True)
        gdown.download(id=imu_url, output=str(folder / "vectornav.bag"), resume=True)

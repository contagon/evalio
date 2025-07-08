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
Note, we skip over a number of trajectories due to missing ground truth data.
https://docs.google.com/document/d/1RS9QSOP4rC7BWoCD6EYUCm9uV_oMkfa3b61krn9OLG8/edit?tab=t.0
"""


@dataclass
class OxfordSpires(Dataset):
    # ------------------------- For loading data ------------------------- #
    def __iter__(self):
        return RosbagIter(
            EVALIO_DATA / OxfordSpires.name() / self.seq,
            "/hesai/pandar",
            "/alphasense_driver_ros/imu",
            self.lidar_params(),
            is_mcap=True,
        )

    def ground_truth_raw(self) -> Trajectory:
        # Some of these are within a few milliseconds of each other
        # skip over ones that are too close
        traj = load_pose_csv(
            EVALIO_DATA / OxfordSpires.name() / self.seq / "gt-tum.txt",
            ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"],
            delimiter=" ",
        )

        poses = []
        stamps = []
        for i in range(1, len(traj)):
            if traj.stamps[i] - traj.stamps[i - 1] > 1e-2:
                poses.append(traj.poses[i])
                stamps.append(traj.stamps[i])

        return Trajectory(metadata=traj.metadata, stamps=stamps, poses=poses)

    # ------------------------- For loading params ------------------------- #
    def cam_T_lidar(self) -> SE3:
        r = SO3(
            qx=0.5023769275907106,
            qy=0.49990425097844265,
            qz=-0.49648618825384844,
            qw=0.5012131556048427,
        )
        t = np.array([0.003242860366163889, -0.07368532755947366, -0.05485800045216396])
        return SE3(r, t)

    def cam_T_imu(self) -> SE3:
        r = SO3(
            qx=-0.003150684959962717,
            qy=0.7095105504964175,
            qz=-0.7046875827967661,
            qw=0.0005124164367280889,
        )
        t = np.array(
            [-0.005000230026155717, -0.0031440163748744266, -0.07336562959794378]
        )
        return SE3(r, t)

    @staticmethod
    def url() -> str:
        return "https://dynamic.robots.ox.ac.uk/datasets/oxford-spires/"

    @staticmethod
    def name() -> str:
        return "oxford_spires"

    @staticmethod
    def sequences() -> list[str]:
        return [
            "blenheim_palace_01",
            "blenheim_palace_02",
            "blenheim_palace_05",
            "bodleian_library_02",
            "christ_church_01",
            "christ_church_02",
            "christ_church_03",
            "christ_church_05",
            "keble_college_02",
            "keble_college_03",
            "keble_college_04",
            "observatory_quarter_01",
            "observatory_quarter_02",
        ]

    def imu_T_lidar(self) -> SE3:
        return self.cam_T_imu().inverse() * self.cam_T_lidar()

    def imu_T_gt(self) -> SE3:
        # Ground truth was found in the lidar frame, but is reported in the "base frame"
        # We go back to the lidar frame (as this transform should be what they used as well)
        # then use calibration to go to imu frame
        # https://github.com/ori-drs/oxford_spires_dataset/blob/main/config/sensor.yaml
        gt_T_lidar = SE3(
            SO3(qx=0.0, qy=0.0, qz=1.0, qw=0.0), np.array([0.0, 0.0, 0.124])
        )
        return self.imu_T_lidar() * gt_T_lidar.inverse()

    def imu_params(self) -> ImuParams:
        # Same one as hilti
        # From their kalibur config (in pdf)
        # https://tp-public-facing.s3.eu-north-1.amazonaws.com/Challenges/2022/2022322_calibration_files.zip
        # https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi085-ds001.pdf
        return ImuParams(
            gyro=0.00019,
            accel=0.00132435,
            gyro_bias=0.000266,
            accel_bias=0.0043,
            bias_init=1e-8,
            integration=1e-8,
            gravity=np.array([0, 0, -9.81]),
        )

    def lidar_params(self) -> LidarParams:
        return LidarParams(
            num_rows=64,
            num_columns=1200,
            min_range=0.1,
            max_range=60.0,
        )

    # ------------------------- For downloading ------------------------- #
    @staticmethod
    def check_download(seq: str) -> bool:
        dir = EVALIO_DATA / OxfordSpires.name() / seq

        if not dir.exists():
            return False
        elif not (dir / "gt-tum.txt").exists():
            return False
        elif not (dir / "metadata.yaml").exists():
            return False
        elif len(list(dir.glob("*.db3"))) < 1:
            return False
        else:
            return True

    @staticmethod
    def download(seq: str):
        folder_id = {
            "blenheim_palace_01": "1sQZhbdWZqyR0fLStesW2sJYuvIW9xyCD",
            "blenheim_palace_02": "1vaU7pn0cxbrBbJk1XKr9hOeeF7Zv00K9",
            "blenheim_palace_05": "1CZpiSX84g4391D-87E6AMlZI4ky1TG7p",
            "bodleian_library_02": "1koKUZrZ3dPazy2qwguPeFqla04V7-opp",
            # This one is split into two... figure out how to handle
            "christ_church_01": "1yd9jl1o4AEacKaYXCHV-AzV-FZTfRm6r",
            # This one is split into two... figure out how to handle
            "christ_church_02": "1f41VoG6mMAvpLxKciqGLhtOuj5iUK_3r",
            "christ_church_03": "14YTkMnxEWnE-iyrk30iu0I39QG-smREF",
            "christ_church_05": "1EDyyuFJ-KnLUv-S5IFv7OInNwPEFSHwl",
            "keble_college_02": "1qAocPo1_h8B2u-LdQoD2td49RJlD-1IW",
            "keble_college_03": "1u-QIvxQvjRXtt0k3vXYdB59XZQ61Khj9",
            # This one is split into two... figure out how to handle
            "keble_college_04": "1VJB8oIAoVVIhGCnbXKYz_uHfiNkwn9_i",
            "observatory_quarter_01": "1Wys3blrdfPVn-EFsXn_a0_0ngvzgSiFb",
            "observatory_quarter_02": "109uXFhqzYhn2bHv_37aQF7xPrJhOOu-_",
        }[seq]

        gt_url = {
            "blenheim_palace_01": "16et7vJhZ15yOCNYYU-i8HVOXemJM3puz",
            "blenheim_palace_02": "191MBEJuABCbb14LJhnJuvq4_ULqqbeQU",
            "blenheim_palace_05": "1jWYpiX4eUz1By1XN1g22ktzb-BCMyE7q",
            "bodleian_library_02": "1_EP7H_uO0XNhIKaFXB4nro8ymhpWB2qg",
            "christ_church_01": "1qXlfhf_0Jr6daeM3v9qmmhC-5yEI6TB5",
            "christ_church_02": "19vyhMivn4I1u-ZkLlIpoa7Nc3sdWNOU1",
            "christ_church_03": "13LbReIW7mJd6jMVBI6IcSRPWvpG6WP9c",
            "christ_church_05": "1Nxbjmwudu2b02Z2zWkJYO1I9rDPLe2Uf",
            "keble_college_02": "1hgNbsqIx8L0vM4rnPSX155sxxiIsDnJC",
            "keble_college_03": "1ybkRnb-wu4VMEqa37RUAHbUv2i-4UluB",
            "keble_college_04": "1iaGvgpDN-3CrwPPZzQjwAveXQOyQnAU4",
            "observatory_quarter_01": "1IOqvzepLesYecizYJh6JU0lJZu2WeW68",
            "observatory_quarter_02": "1iPQQD2zijlCf8a6J8YW5QBlVE2KsYRdZ",
        }[seq]

        import gdown  # type: ignore

        folder = EVALIO_DATA / OxfordSpires.name() / seq

        print(f"Downloading {seq} to {folder}...")
        folder.mkdir(parents=True, exist_ok=True)
        gdown.download(id=gt_url, output=str(folder / "gt-tum.csv"), resume=True)
        gdown.download_folder(id=folder_id, output=str(folder), resume=True)

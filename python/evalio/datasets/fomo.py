from enum import auto
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from evalio._cpp.helpers import fomo_bin_to_evalio  # type: ignore
from evalio.datasets.loaders import RawDataIter
from evalio.types import (
    SE3,
    SO3,
    ImuMeasurement,
    ImuParams,
    LidarParams,
    Trajectory,
    Stamp,
)

from .base import Dataset, DatasetIterator

# fmt: off
MAP_IDX_TO_ROW = [4, 5, 6, 7, 2, 0, 1, 3, 12, 13, 14, 15, 8, 9, 10, 11, 20, 21, 22, 23, 16, 17, 18, 19, 28, 29, 30, 31, 24, 25, 26, 27, 36, 37, 38, 39, 32, 33, 34, 35, 44, 45, 46, 47, 40, 41, 42, 43, 52, 53, 54, 55, 48, 49, 50, 51, 60, 61, 62, 63, 56, 57, 58, 59, 68, 69, 70, 71, 64, 65, 66, 67, 76, 77, 78, 79, 72, 73, 74, 75, 84, 85, 86, 87, 80, 81, 82, 83, 92, 93, 94, 95, 88, 89, 90, 91, 100, 101, 102, 103, 96, 97, 98, 99, 108, 109, 110, 111, 104, 105, 106, 107, 116, 117, 118, 119, 112, 113, 114, 115, 124, 125, 126, 127, 120, 121, 122, 123]
MAP_ROW_TO_IDX = [-1] * 128
for i, row in enumerate(MAP_IDX_TO_ROW):
    MAP_ROW_TO_IDX[row] = i
# fmt: on


class Fomo(Dataset):
    """The FoMo dataset is a multi-season collection designed to support research on robust algorithms for robot autonomy under adverse conditions.

    Data was collected in the same location across 12 deployments spanning two seasons (March and August 2025), with each deployment containing six trajectories of varying lengths. The dataset includes synchronized data from a Robosense Ruby Plus lidar, an Applanix POS LV IMU, and ground truth trajectories provided in TUM format. The FoMo dataset is ideal for evaluating the performance of algorithms under challenging conditions such as snow, rain, and fog, making it a valuable resource for researchers working on robust perception and localization for autonomous vehicles.

    It should be noted that they DO NOT report GT orientation. We synthetically create plausible orientation for visualization purposes.

    Data can be browsed here, but does require an amazon account:
    https://s3.console.aws.amazon.com/s3/buckets/fomo-dataset/
    """

    # There are 12 deployments x 6 trajectories = 72 sequences total
    # We choose two deployments with largest variability
    # One in winter (march) and one in summer (august)
    blue_25_03 = auto()
    green_25_03 = auto()
    magenta_25_03 = auto()
    orange_25_03 = auto()
    red_25_03 = auto()
    yellow_25_03 = auto()

    blue_25_08 = auto()
    green_25_08 = auto()
    magenta_25_08 = auto()
    orange_25_08 = auto()
    red_25_08 = auto()
    yellow_25_08 = auto()

    # ------------------------- For loading data ------------------------- #
    def data_iter(self) -> DatasetIterator:
        # Load IMU data
        # Format: time,wx,wy,wz,ax,ay,az
        imu_file = self.folder / "vectornav.csv"
        imu_raw = np.loadtxt(imu_file, delimiter=",", skiprows=1)  # skip header
        imu_data = [
            ImuMeasurement(
                stamp=Stamp.from_sec(row[0]),
                gyro=row[1:4],
                accel=row[4:7],
            )
            for row in imu_raw
        ]

        # Setup lidar files
        lidar_path = self.folder / "robosense"
        lidar_files = sorted(list(lidar_path.glob("*.bin")))
        params = self.lidar_params()

        def lidar_iter():
            for lidar_file in lidar_files:
                stamp = Stamp.from_nsec(int(lidar_file.stem) * 1000)
                yield fomo_bin_to_evalio(str(lidar_file), stamp, params, MAP_ROW_TO_IDX)

        return RawDataIter(
            lidar_iter(),
            iter(imu_data),
            len(lidar_files),
        )

    def ground_truth_raw(self) -> Trajectory:
        # gt provided in tum format
        # unfortunately no orientation provided, so is just set to identity
        path = self.folder / "gt.txt"
        traj_utm = Trajectory.from_csv(
            path,
            fieldnames=["sec", "x", "y", "z", "qx", "qy", "qz", "qw"],
            delimiter=" ",
        )

        # Subtract out the initial position to get a local frame reference
        # more likely to have numerical issues with large UTM coordinates than with small ones
        init_position = traj_utm.poses[0].trans
        return Trajectory(
            poses=[SE3(p.rot, p.trans - init_position) for p in traj_utm.poses],
            stamps=traj_utm.stamps,
        )

    # ------------------------- For loading params ------------------------- #
    def lidar_T_cam(self) -> SE3:
        # from robosense to zedx_left
        return SE3(
            SO3(
                qx=0.5729177995838587,
                qy=0.5754445229626066,
                qz=-0.4169693162625371,
                qw=-0.4083691775928738,
            ),
            np.array(
                [-0.07952352627148711, -0.04440832511705086, -0.21333361167112735]
            ),
        )

    def cam_T_imu(self) -> SE3:
        # from zedx_left to vectornav
        return SE3(
            SO3(
                qx=0.8110699587208322,
                qy=0.0038604101545591683,
                qz=-0.005826532480602177,
                qw=0.5849074036232389,
            ),
            np.array([-0.06927926773327224, 0.4414898523924328, -0.803272927508376]),
        )

    def imu_T_lidar(self) -> SE3:
        # They have a vectornav and a xsens IMU
        # The vectornav has better parameters
        return self.cam_T_imu().inverse() * self.lidar_T_cam().inverse()

    def imu_T_gt(self) -> SE3:
        # TODO: This is really unclear in their paper
        return SE3.identity()

    def imu_params(self) -> ImuParams:
        # From their allan variance plots
        return ImuParams(
            gyro=0.0000617,
            accel=0.001,
            gyro_bias=0.0000010471975512,
            accel_bias=0.00002,
            bias_init=1e-6,
            integration=1e-6,
            gravity=np.array([0, 0, -9.81]),
            rate=200.0,
            brand="VectorNav",
            model="VN100",
        )

    def lidar_params(self) -> LidarParams:
        # Robosense Ruby Plus
        # 128 channels, 10 Hz, range up to 245 m (10% reflectivity)
        # Source: https://data.ouster.io/downloads/datasheets/velodyne/63-9679_Rev-B_DATASHEET_ALPHA-PRIME_web.pdf
        # Source: https://github.com/utiasASRL/pyboreas/DATA_REFERENCE.md
        return LidarParams(
            num_rows=128,
            # This is approximate, I never saw values greater than this
            num_columns=1800,
            min_range=0.4,
            max_range=240.0,
            rate=10.0,
            brand="Robosense",
            model="Ruby Plus",
        )

    # ------------------------- dataset info ------------------------- #
    @staticmethod
    def url() -> str:
        return "https://github.com/utiasASRL/pyboreas/blob/master/DATA_REFERENCE.md"

    def environment(self) -> str:
        return "Off-road"

    def vehicle(self) -> str:
        return "Car"

    # ------------------------- For downloading ------------------------- #
    def files(self) -> Sequence[str | Path]:
        # TODO: This returns true if only a single lidar file is downloaded
        return [
            "vectornav.csv",
            "gt.txt",
            "robosense",
        ]

    def download(self):
        from subprocess import Popen, PIPE, run
        from tqdm import tqdm
        import sys
        # NOTE: This is experimental; not sure if should use aws cli or boto3 to download from S3
        # boto3 is slower for all those tiny lidar files

        # Find the AWS cli installed in the same environment as evalio
        aws = Path(sys.prefix) / "bin" / "aws"

        # We drop a little info in the names, so we'll have to enumerate things here
        s3_bucket: str
        match self:
            case Fomo.blue_25_03:
                s3_bucket = "2025-03-10/blue_2025-03-10-16-59"
            case Fomo.green_25_03:
                s3_bucket = "2025-03-10/green_2025-03-14-08-48"
            case Fomo.magenta_25_03:
                s3_bucket = "2025-03-10/magenta_2025-03-14-10-27"
            case Fomo.orange_25_03:
                s3_bucket = "2025-03-10/orange_2025-03-14-09-07"
            case Fomo.red_25_03:
                s3_bucket = "2025-03-10/red_2025-03-10-16-53"
            case Fomo.yellow_25_03:
                s3_bucket = "2025-03-10/yellow_2025-03-10-17-22"
            case Fomo.blue_25_08:
                s3_bucket = "2025-08-20/blue_2025-08-20-11-25"
            case Fomo.green_25_08:
                s3_bucket = "2025-08-20/green_2025-08-20-11-39"
            case Fomo.magenta_25_08:
                s3_bucket = "2025-08-20/magenta_2025-08-20-12-41"
            case Fomo.orange_25_08:
                s3_bucket = "2025-08-20/orange_2025-08-20-13-17"
            case Fomo.red_25_08:
                s3_bucket = "2025-08-20/red_2025-08-20-10-42"
            case Fomo.yellow_25_08:
                s3_bucket = "2025-08-20/yellow_2025-08-20-10-54"

        # Figure out how many total files they are
        output = run(
            [
                str(aws),
                "s3",
                "ls",
                "--no-sign-request",
                f"s3://fomo-dataset/data/{s3_bucket}/robosense/",
                "--summarize",
            ],
            capture_output=True,
            text=True,
        )

        # get the last few lines of aws CLI output to show total file count and size
        count = int(output.stdout.splitlines()[-2].split(":")[1].strip())
        count += 2

        loop = tqdm(total=count, desc="Downloading", unit="file")
        popen = Popen(
            [
                str(aws),
                "s3",
                "sync",
                "--no-sign-request",
                f"s3://fomo-dataset/data/{s3_bucket}/",
                self.folder,
                "--no-progress",
                "--exclude",
                "*",
                "--include",
                "vectornav.csv",
                "--include",
                "gt.txt",
                "--include",
                "robosense/*",
            ],
            stdout=PIPE,
            universal_newlines=True,
        )
        for stdout_line in iter(popen.stdout.readline, ""):  # type: ignore
            loop.update(1)

    def quick_len(self) -> Optional[int]:
        # TODO
        return None

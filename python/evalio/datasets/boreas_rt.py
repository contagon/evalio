from enum import auto
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from evalio._cpp.helpers import boreas_bin_to_evalio  # type: ignore
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

from .boreas import MAP_ROW_TO_IDX
from .base import Dataset, DatasetIterator


class BoreasRT(Dataset):
    """Extended Boreas dataset with newer sequences and a higher-grade IMU.

    Collected with the same Velodyne Alpha-Prime 128-beam lidar as the original Boreas
    dataset but adds a Silicon Sensing DMU41 MEMS IMU for higher-fidelity inertial data.
    Ground truth is from the Applanix post-processed solution.

    NOTE: The IMU data has a handful of issues, namely:
        1) Occasional dropouts noted in their paper
        2) Not time-synchronized, but manually time-synced after the fact
    """

    # odom_test_rt sequences (15 sequences)
    # Source: https://github.com/utiasASRL/pyboreas/blob/master/pyboreas/data/splits.py
    glen_2024_12_03_12_54 = auto()  # suburbs (glen shields) mapping + odometry
    glen_2025_01_08_11_22 = auto()  # suburbs (glen shields) odometry
    glen_2025_02_15_17_19 = auto()  # suburbs (glen shields) odometry
    industrial_2024_12_05_14_12 = auto()  # industrial mapping + odometry
    industrial_2024_12_23_16_27 = auto()  # industrial odometry
    industrial_2024_12_23_16_44 = auto()  # industrial odometry
    skyway_2024_12_04_11_45 = auto()  # skyway mapping + odometry
    skyway_2024_12_04_12_08 = auto()  # skyway odometry
    skyway_2024_12_04_12_34 = auto()  # skyway odometry
    tunnel_2024_12_04_14_28 = auto()  # tunnel east mapping + odometry
    tunnel_2024_12_04_14_50 = auto()  # tunnel east odometry
    tunnel_2024_12_04_15_19 = auto()  # tunnel east odometry
    farm_2025_07_18_14_55 = auto()  # farm mapping + odometry
    farm_2025_07_18_15_30 = auto()  # farm odometry
    farm_2025_08_13_09_01 = auto()  # farm odometry

    # There are more sequences, but these should be sufficient?
    # If requested we can add the other ~45 sequences, but they should be mostly identical to these

    # ------------------------- For loading data ------------------------- #
    def data_iter(self) -> DatasetIterator:
        lidar_path = self.folder / "lidar"

        # Load IMU data
        # Format: GPSTime,angvel_z,angvel_y,angvel_x,accelz,accely,accelx
        imu_file = self.folder / "imu" / "dmu_imu.csv"
        imu_raw = np.loadtxt(imu_file, delimiter=",", skiprows=1)  # skip header
        imu_data = [
            ImuMeasurement(
                stamp=Stamp.from_nsec(row[0]),
                gyro=row[1:4],
                accel=row[4:7],
            )
            for row in imu_raw
        ]

        # Setup lidar files
        # Lidar timestamps are in microseconds, and we convert to nanoseconds for consistency with the rest of evalio.
        lidar_files = sorted(list(lidar_path.glob("*.bin")))
        lidar_stamps = [Stamp.from_nsec(int(f.stem) * 1000) for f in lidar_files]
        params = self.lidar_params()

        def lidar_iter():
            for lidar_file in lidar_files:
                stamp = Stamp.from_nsec(int(lidar_file.stem) * 1000)
                yield boreas_bin_to_evalio(
                    str(lidar_file), stamp, params, MAP_ROW_TO_IDX
                )

        return RawDataIter(
            lidar_iter(),
            iter(imu_data),
            len(lidar_stamps),
        )

    def ground_truth_raw(self) -> Trajectory:
        """Load Boreas ground truth from applanix/gps_post_process.csv.

        File format: t, x, y, z, vx, vy, vz, r, p, y, wz, wy, wx
        where t is in microseconds and r, p, y are roll, pitch, yaw in radians.
        GT is in ENU frame.

        The reference says timestamps are in microseconds, but they are actually in seconds (with decimal points)!

        Source: https://github.com/utiasASRL/pyboreas/blob/master/DATA_REFERENCE.md
        """
        path = self.folder / "applanix" / "gps_post_process.csv"

        stamps = np.loadtxt(path, usecols=0, delimiter=",", skiprows=1)
        stamps = [Stamp.from_sec(x) for x in stamps]

        all_xyz = np.loadtxt(path, usecols=(1, 2, 3), delimiter=",", skiprows=1)
        all_rpy = np.loadtxt(path, usecols=(7, 8, 9), delimiter=",", skiprows=1)
        # The official implementation has negatives for all of the individual rot matrices
        # https://github.com/utiasASRL/pyboreas/blob/14be1c090d91f6e7e958fcca8545775038a455ce/pyboreas/utils/utils.py#L30-L48
        all_R = [SO3.from_ypr(-y, -p, -r) for r, p, y in all_rpy]
        poses = [SE3(R, xyz) for R, xyz in zip(all_R, all_xyz)]

        return Trajectory(stamps=stamps, poses=poses)

    # ------------------------- For loading params ------------------------- #
    def lidar_T_gt(self) -> SE3:
        # Given by T_applanix_lidar
        # fmt: off
        return SE3.from_mat(
            np.array([
                [0.734962, -0.678108, 0.000000, 0.000000],
                [0.678108,  0.734962, 0.000000, 0.000000],
                [0.000000,  0.000000, 1.000000, 0.130000],
                [0.000000,  0.000000, 0.000000, 1.000000],
            ])
        ).inverse()
        # fmt: on

    def imu_T_gt(self) -> SE3:
        # Given by T_applanix_dmu
        # fmt: off
        return SE3.from_mat(
            np.array([
                [0.9999510, -0.009823, -0.001420,  0.000000],
                [-0.009824, -0.999952, -0.000242,  0.000000],
                [-0.001418,  0.000256, -0.999999, -0.150000],
                [0.0000000,  0.000000,  0.000000,  1.000000],
            ])
        ).inverse()
        # fmt: on

    def imu_T_lidar(self) -> SE3:
        return self.imu_T_gt() * self.lidar_T_gt().inverse()

    def imu_params(self) -> ImuParams:
        # Silicon Sensing DMU41 MEMS IMU — higher-grade MEMS IMU used in the newer Boreas RT sequences.
        # Specs taken from the table 2 in their paper
        # https://arxiv.org/pdf/2602.16870
        # TODO: Request Tushar's help to extract this info
        return ImuParams(
            gyro=1.0e-3,  # rad/s/sqrt(Hz) — conservative estimate
            accel=1.0e-2,  # m/s^2/sqrt(Hz) — conservative estimate
            gyro_bias=1.0e-4,  # rad/s^2/sqrt(Hz)
            accel_bias=1.0e-3,  # m/s^3/sqrt(Hz)
            bias_init=1e-6,
            integration=1e-6,
            gravity=np.array([0, 0, -9.81]),
            rate=200.0,
            brand="Applanix",
            model="POS LV",
        )

    def lidar_params(self) -> LidarParams:
        # Velodyne Alpha-Prime (VLS-128)
        # 128 channels, 10 Hz, range up to 245 m (10% reflectivity)
        # Source: https://data.ouster.io/downloads/datasheets/velodyne/63-9679_Rev-B_DATASHEET_ALPHA-PRIME_web.pdf
        # Source: https://github.com/utiasASRL/pyboreas/blob/master/DATA_RT_REFERENCE.md
        return LidarParams(
            num_rows=128,
            # This is approximate, I never saw values greater than this
            num_columns=2000,
            min_range=0.4,
            max_range=245.0,
            rate=10.0,
            brand="Velodyne",
            model="VLS-128",
        )

    # ------------------------- dataset info ------------------------- #
    @staticmethod
    def url() -> str:
        return "https://github.com/utiasASRL/pyboreas/blob/master/DATA_RT_REFERENCE.md"

    def environment(self) -> str:
        return "Urban Driving"

    def vehicle(self) -> str:
        return "Car"

    # ------------------------- For downloading ------------------------- #
    def files(self) -> Sequence[str | Path]:
        # TODO: This returns true if only a single lidar file is downloaded
        return [
            "imu/dmu_imu.csv",
            "applanix/gps_post_process.csv",
            "lidar",
        ]

    def download(self):
        from subprocess import Popen, PIPE, run
        from tqdm import tqdm
        import sys
        # NOTE: This is experimental; not sure if should use aws cli or boto3 to download from S3
        # boto3 is slower for all those tiny lidar files

        # Find the AWS cli installed in the same environment as evalio
        aws = Path(sys.prefix) / "bin" / "aws"

        seq = "boreas-" + self.seq_name.replace("_", "-").split("-", 1)[1]

        # Figure out how many total files they are
        output = run(
            [
                str(aws),
                "s3",
                "ls",
                "--no-sign-request",
                f"s3://boreas/{seq}/lidar/",
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
                f"s3://boreas/{seq}/",
                self.folder,
                "--no-progress",
                "--exclude",
                "*",
                "--include",
                "imu/dmu_imu.csv",
                "--include",
                "applanix/gps_post_process.csv",
                "--include",
                "lidar/*",
            ],
            stdout=PIPE,
            universal_newlines=True,
        )
        for stdout_line in iter(popen.stdout.readline, ""):  # type: ignore
            loop.update(1)

    def quick_len(self) -> Optional[int]:
        # TODO
        return None

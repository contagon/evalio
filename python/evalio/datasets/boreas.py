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

from .base import Dataset, DatasetIterator

# Snippet for determining order
# prev_col = 0
# prev_idx = 0
# idx = 0
# points = mm.points
# while True:
#     if points[idx].col != prev_col:
#         print(f"col {prev_col} has {idx - prev_idx} points")

#         if idx - prev_idx == 128:
#             print("all rows seen, stopping")
#             cols = [p.row for p in points[prev_idx:idx]]
#             print(cols)
#             quit()

#         prev_col = points[idx].col
#         prev_idx = idx

#     idx += 1
# fmt: off
MAP_IDX_TO_ROW = [4,53,102,23,64,113,34,83,12,61,110,31,72,121,42,91,36,85,6,55,96,17,66,115,44,93,14,63,104,25,74,123,68,117,38,87,0,49,98,19,76,125,46,95,8,57,106,27,100,21,70,119,32,81,2,51,108,29,78,127,40,89,10,59,20,69,118,39,80,1,50,99,28,77,126,47,88,9,58,107,52,101,22,71,112,33,82,3,60,109,30,79,120,41,90,11,84,5,54,103,16,65,114,35,92,13,62,111,24,73,122,43,116,37,86,7,48,97,18,67,124,45,94,15,56,105,26,75,]
MAP_ROW_TO_IDX = [-1] * 128
for i, row in enumerate(MAP_IDX_TO_ROW):
    MAP_ROW_TO_IDX[row] = i
# fmt: on


class Boreas(Dataset):
    """Large-scale long-term autonomous driving dataset collected in Toronto, Canada.

    Data was collected across all four seasons and various weather conditions using a
    Velodyne Alpha-Prime 128-beam lidar and an Applanix POS LV IMU. Ground truth is
    from the Applanix post-processed solution.

    We use the odometry benchmark splits (odom_train + odom_test).

    We treat the base/IMU frame as the Applanix frame, which is ENU with x-right, y-forward, z-up.
    (raw IMU data is actually x-backward, y-left, z-down, but we apply the necessary reordering in data_iter).

    Data can be browsed here, but does require an amazon account:
    https://s3.console.aws.amazon.com/s3/buckets/boreas/
    """

    # odom_train sequences (31 sequences)
    # Source: https://github.com/utiasASRL/pyboreas/blob/master/pyboreas/data/splits.py
    boreas_2020_11_26_13_58 = auto()
    boreas_2020_12_01_13_26 = auto()
    boreas_2021_01_15_12_17 = auto()
    boreas_2021_01_19_15_08 = auto()
    boreas_2021_01_26_10_59 = auto()
    boreas_2021_02_02_14_07 = auto()
    boreas_2021_02_09_12_55 = auto()
    boreas_2021_03_02_13_38 = auto()
    boreas_2021_03_09_14_23 = auto()
    boreas_2021_03_23_12_43 = auto()
    boreas_2021_04_08_12_44 = auto()
    boreas_2021_04_13_14_49 = auto()
    boreas_2021_04_20_14_11 = auto()
    boreas_2021_04_27_13_45 = auto()
    boreas_2021_05_13_16_22 = auto()
    boreas_2021_06_01_14_13 = auto()
    boreas_2021_06_03_11_34 = auto()
    boreas_2021_06_17_17_52 = auto()
    boreas_2021_06_29_18_05 = auto()
    boreas_2021_07_20_17_33 = auto()
    boreas_2021_07_27_14_43 = auto()
    boreas_2021_08_05_13_34 = auto()
    boreas_2021_09_07_09_35 = auto()
    boreas_2021_09_14_20_00 = auto()
    boreas_2021_09_28_15_45 = auto()
    boreas_2021_10_05_15_35 = auto()
    boreas_2021_10_26_12_51 = auto()
    boreas_2021_11_06_18_55 = auto()
    boreas_2021_11_14_11_58 = auto()
    boreas_2021_11_20_16_19 = auto()
    boreas_2021_11_23_14_27 = auto()

    # odom_test sequences (13 sequences)
    # These don't have ground truth so we don't include them
    # boreas_2020_12_04_14_00 = auto()
    # boreas_2021_01_26_11_22 = auto()
    # boreas_2021_02_09_13_25 = auto()
    # boreas_2021_03_09_14_56 = auto()
    # boreas_2021_04_13_15_35 = auto()
    # boreas_2021_05_13_17_23 = auto()
    # boreas_2021_06_17_18_46 = auto()
    # boreas_2021_07_27_15_31 = auto()
    # boreas_2021_08_05_14_14 = auto()
    # boreas_2021_09_14_20_28 = auto()
    # boreas_2021_10_05_16_13 = auto()
    # boreas_2021_10_26_13_28 = auto()
    # boreas_2021_11_28_09_18 = auto()

    # ------------------------- For loading data ------------------------- #
    def data_iter(self) -> DatasetIterator:
        lidar_path = self.folder / "lidar"

        # Load IMU data
        # Format: GPSTime,angvel_z,angvel_y,angvel_x,accelz,accely,accelx
        #
        # Timestamps are GPS time in seconds (float).
        # IMU frame: x-backward, y-left, z-down (body frame)
        # Applanix frame: x-right, y-forward, z-up (ENU frame)
        # We permute as needed
        # Source: https://github.com/utiasASRL/pyboreas/DATA_REFERENCE.md
        # https://github.com/utiasASRL/pyboreas/issues/44
        #
        # Additionally, gravity is already removed from the raw IMU data, so we'll set that to 0 later
        # https://github.com/utiasASRL/pyboreas/issues/24
        imu_file = self.folder / "applanix" / "imu_raw.csv"
        imu_raw = np.loadtxt(imu_file, delimiter=",", skiprows=1)  # skip header
        imu_data = [
            ImuMeasurement(
                stamp=Stamp.from_sec(s),
                gyro=np.array([-wy, -wx, wz]),  # reorder to x,y,z
                accel=np.array([-ay, -ax, az]),  # reorder to x,y,z
            )
            for s, wz, wy, wx, az, ay, ax in imu_raw
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
    def imu_T_lidar(self) -> SE3:
        # T_applanix_lidar: transformation from Applanix (IMU) frame to Velodyne lidar frame.
        # Source: s3://boreas/boreas-2020-11-26-13-58/calib/T_applanix_lidar.txt
        # fmt: off
        return SE3.from_mat(
            np.array([
                    [7.360555651493132512e-01, -6.769211217083995757e-01, 0.000000000000000000e+00, 0.000000000000000000e+00],
                    [6.769211217083995757e-01, 7.360555651493132512e-01, 0.000000000000000000e+00, 0.000000000000000000e+00],
                    [0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00, 1.300000000000000044e-01],
                    [0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]
            ])
        )
        # fmt: on

    def imu_T_gt(self) -> SE3:
        # Ground truth is in the Applanix (IMU) frame
        return SE3.identity()

    def imu_params(self) -> ImuParams:
        # Applanix POS LV (SPAN) — tactical-grade MEMS IMU integrated in the Applanix system.
        # Noise values are estimates from the Boreas dataset paper and Applanix datasheet.
        # Source: https://www.applanix.com/downloads/products/specs/POS-LV-Specifications.pdf
        # TODO: Verify these parameters, they seem wrong...
        return ImuParams(
            gyro=1.0e-3,  # rad/s/sqrt(Hz) — conservative estimate
            accel=1.0e-2,  # m/s^2/sqrt(Hz) — conservative estimate
            gyro_bias=1.0e-4,  # rad/s^2/sqrt(Hz)
            accel_bias=1.0e-3,  # m/s^3/sqrt(Hz)
            bias_init=1e-6,
            integration=1e-6,
            gravity=np.array(
                [0, 0, 0]
            ),  # Gravity is already removed from the raw IMU data
            rate=200.0,
            brand="Applanix",
            model="POS LV",
        )

    def lidar_params(self) -> LidarParams:
        # Velodyne Alpha-Prime (VLS-128)
        # 128 channels, 10 Hz, range up to 300 m (10% reflectivity)
        # Source: https://data.ouster.io/downloads/datasheets/velodyne/63-9679_Rev-B_DATASHEET_ALPHA-PRIME_web.pdf
        # Source: https://github.com/utiasASRL/pyboreas/DATA_REFERENCE.md
        return LidarParams(
            num_rows=128,
            # This is approximate, I never saw values greater than this
            num_columns=2000,
            min_range=0.4,
            max_range=300.0,
            rate=10.0,
            brand="Velodyne",
            model="VLS-128",
        )

    # ------------------------- dataset info ------------------------- #
    @staticmethod
    def url() -> str:
        return "https://github.com/utiasASRL/pyboreas/blob/master/DATA_REFERENCE.md"

    def environment(self) -> str:
        return "Urban Driving"

    def vehicle(self) -> str:
        return "Car"

    # ------------------------- For downloading ------------------------- #
    def files(self) -> Sequence[str | Path]:
        # TODO: This returns true if only a single lidar file is downloaded
        return [
            "applanix/imu_raw.csv",
            "applanix/gps_post_process.csv",
            "lidar",
        ]

    def is_downloaded(self) -> bool:
        return True

    def download(self):
        from subprocess import Popen, PIPE, run
        from tqdm import tqdm
        import sys
        # NOTE: This is experimental; not sure if should use aws cli or boto3 to download from S3
        # boto3 is slower for all those tiny lidar files

        seq = self.seq_name.replace("_", "-")

        # Figure out how many total files they are
        output = run(
            [
                "aws",
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

        # Find the AWS cli installed in the same environment as evalio
        aws = Path(sys.prefix) / "bin" / "aws"

        loop = tqdm(total=count, desc="Downloading Boreas data", unit="file")
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
                "applanix/imu_raw.csv",
                "--include",
                "applanix/*",
                "--include",
                "lidar/*",
            ],
            stdout=PIPE,
            universal_newlines=True,
        )
        for stdout_line in iter(popen.stdout.readline, ""):  # type: ignore
            print(stdout_line, end="")
            loop.update(1)

    def quick_len(self) -> Optional[int]:
        # TODO
        return None

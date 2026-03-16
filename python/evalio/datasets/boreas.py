from enum import auto
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from evalio._cpp.types import Duration, LidarMeasurement, Point, Stamp  # type: ignore
from evalio.datasets.loaders import RawDataIter
from evalio.types import (
    SE3,
    SO3,
    ImuMeasurement,
    ImuParams,
    LidarParams,
    Trajectory,
)

from .base import Dataset, DatasetIterator


def _seq_name_to_folder(seq_name: str) -> str:
    """Convert evalio snake_case seq_name to Boreas folder name.

    e.g. 'boreas_2020_11_26_13_58' -> 'boreas-2020-11-26-13-58'
    """
    return seq_name.replace("_", "-")


def _yaw_pitch_roll_to_rot(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """Convert yaw-pitch-roll (ZYX Euler angles, in radians) to a 3x3 rotation matrix.

    Implements yawPitchRollToRot = roll(r) @ pitch(p) @ yaw(y) as used in Boreas.
    Source: https://github.com/utiasSTARS/pyboreas/blob/master/pyboreas/utils/utils.py
    """
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)

    # Rotation matrix: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)  [standard ZYX]
    R = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    return R


def _load_boreas_gt(path: Path) -> Trajectory:
    """Load Boreas ground truth from applanix/lidar_poses.csv.

    File format: t, x, y, z, vx, vy, vz, r, p, y, wz, wy, wx
    where t is in microseconds and r, p, y are roll, pitch, yaw in radians.
    GT is in ENU frame.

    Source: https://github.com/utiasSTARS/boreas-devkit / DATA_REFERENCE.md
    """
    stamps = []
    poses = []

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 10:
                continue
            t_usec = int(parts[0])
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
            # cols 4-6 are velocities (skip)
            roll = float(parts[7])
            pitch = float(parts[8])
            yaw = float(parts[9])

            stamp = Stamp.from_nsec(t_usec * 1000)
            R = _yaw_pitch_roll_to_rot(yaw, pitch, roll)
            rot = SO3.from_mat(R)
            trans = np.array([x, y, z])
            poses.append(SE3(rot, trans))
            stamps.append(stamp)

    return Trajectory(stamps=stamps, poses=poses)


def _load_boreas_bin(path: Path, stamp: Stamp, params: LidarParams) -> LidarMeasurement:
    """Load a Boreas Velodyne .bin point cloud file.

    Binary format: float32[N, 6] where columns are [x, y, z, intensity, ring, t].
    't' is offset in microseconds from the start of the scan.

    Source: https://github.com/utiasSTARS/boreas-devkit / DATA_REFERENCE.md
    """
    data = np.fromfile(path, dtype=np.float32).reshape(-1, 6)
    num_cols = params.num_columns

    points = []
    col_counters = {}
    for row_data in data:
        x, y, z, intensity, ring, t_usec = row_data
        row = int(ring)
        t_duration = Duration.from_nsec(int(t_usec * 1000))
        col = col_counters.get(row, 0)
        col_counters[row] = col + 1

        if row >= params.num_rows or col >= num_cols:
            continue

        p = Point(
            x=float(x),
            y=float(y),
            z=float(z),
            intensity=float(intensity),
            t=t_duration,
            range=0,
            row=row,
            col=col,
        )
        points.append(p)

    # Build a properly-sized row-major point cloud with gaps filled
    mm = LidarMeasurement(stamp)
    mm.points = [
        Point(row=r, col=c) for r in range(params.num_rows) for c in range(num_cols)
    ]
    for p in points:
        mm.points[p.row * num_cols + p.col] = p

    return mm


class Boreas(Dataset):
    """Large-scale long-term autonomous driving dataset collected in Toronto, Canada.

    Data was collected across all four seasons and various weather conditions using a
    Velodyne Alpha-Prime 128-beam lidar and an Applanix POS LV IMU. Ground truth is
    from the Applanix post-processed solution.

    We use the odometry benchmark splits (odom_train + odom_test).
    """

    # odom_train sequences (31 sequences)
    # Source: https://github.com/utiasSTARS/pyboreas/blob/master/pyboreas/data/splits.py
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
    boreas_2020_12_04_14_00 = auto()
    boreas_2021_01_26_11_22 = auto()
    boreas_2021_02_09_13_25 = auto()
    boreas_2021_03_09_14_56 = auto()
    boreas_2021_04_13_15_35 = auto()
    boreas_2021_05_13_17_23 = auto()
    boreas_2021_06_17_18_46 = auto()
    boreas_2021_07_27_15_31 = auto()
    boreas_2021_08_05_14_14 = auto()
    boreas_2021_09_14_20_28 = auto()
    boreas_2021_10_05_16_13 = auto()
    boreas_2021_10_26_13_28 = auto()
    boreas_2021_11_28_09_18 = auto()

    # ------------------------- For loading data ------------------------- #
    @property
    def _boreas_folder_name(self) -> str:
        """Return the original Boreas folder name (with dashes)."""
        return _seq_name_to_folder(self.seq_name)

    def data_iter(self) -> DatasetIterator:
        imu_file = self.folder / "applanix" / "imu_raw.csv"
        lidar_path = self.folder / "lidar"
        lidar_params = self.lidar_params()

        # Load IMU data
        # Format: GPSTime,angvel_z,angvel_y,angvel_x,accelz,accely,accelx
        # Timestamps are GPS time in seconds (float).
        # IMU frame: x-backward, y-left, z-up (Applanix body frame)
        # Source: https://github.com/utiasSTARS/boreas-devkit / DATA_REFERENCE.md
        imu_data = []
        with open(imu_file) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or line.lower().startswith("gps"):
                    continue
                parts = line.split(",")
                if len(parts) < 7:
                    continue
                t_sec = float(parts[0])
                stamp = Stamp.from_sec(t_sec)
                # File order: angvel_z, angvel_y, angvel_x
                gyro = np.array([float(parts[3]), float(parts[2]), float(parts[1])])
                # File order: accelz, accely, accelx
                accel = np.array([float(parts[6]), float(parts[5]), float(parts[4])])
                imu_data.append(ImuMeasurement(stamp, gyro, accel))

        # Setup lidar files
        lidar_files = sorted(list(lidar_path.glob("*.bin")))
        lidar_stamps = [Stamp.from_nsec(int(f.stem) * 1000) for f in lidar_files]

        def lidar_iter():
            for stamp, lidar_file in zip(lidar_stamps, lidar_files):
                yield _load_boreas_bin(lidar_file, stamp, lidar_params)

        return RawDataIter(
            lidar_iter(),
            iter(imu_data),
            len(lidar_stamps),
        )

    def ground_truth_raw(self) -> Trajectory:
        return _load_boreas_gt(self.folder / "applanix" / "lidar_poses.csv")

    # ------------------------- For loading params ------------------------- #
    def imu_T_lidar(self) -> SE3:
        # T_applanix_lidar: transformation from Applanix (IMU) frame to Velodyne lidar frame.
        # Source: s3://boreas/boreas-2020-11-26-13-58/calib/T_applanix_lidar.txt
        return SE3.from_mat(
            np.array(
                [
                    [7.360555651493132512e-01, -6.769211217083995757e-01, 0.0, 0.0],
                    [6.769211217083995757e-01, 7.360555651493132512e-01, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.13],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            )
        )

    def imu_T_gt(self) -> SE3:
        # Ground truth is in the Applanix (IMU) frame
        return SE3.identity()

    def imu_params(self) -> ImuParams:
        # Applanix POS LV (SPAN) — tactical-grade MEMS IMU integrated in the Applanix system.
        # Noise values are estimates from the Boreas dataset paper and Applanix datasheet.
        # Source: https://arxiv.org/abs/2203.10168 (Boreas dataset paper)
        # Source: https://www.applanix.com/downloads/products/specs/POS-LV-Specifications.pdf
        return ImuParams(
            gyro=1.0e-3,  # rad/s/sqrt(Hz) — conservative estimate
            accel=1.0e-2,  # m/s^2/sqrt(Hz) — conservative estimate
            gyro_bias=1.0e-4,  # rad/s^2/sqrt(Hz)
            accel_bias=1.0e-3,  # m/s^3/sqrt(Hz)
            bias_init=1e-6,
            integration=1e-6,
            gravity=np.array([0, 0, -9.81]),  # ENU frame, gravity points down
            rate=200.0,
            brand="Applanix",
            model="POS LV",
        )

    def lidar_params(self) -> LidarParams:
        # Velodyne Alpha-Prime (VLS-128)
        # 128 channels, 10 Hz, range up to 300 m (10% reflectivity)
        # Source: https://velodynelidar.com/products/alpha-prime/
        # Source: https://github.com/utiasSTARS/boreas-devkit / DATA_REFERENCE.md
        return LidarParams(
            num_rows=128,
            num_columns=1800,
            min_range=0.4,
            max_range=300.0,
            rate=10.0,
            brand="Velodyne",
            model="VLS-128",
        )

    # ------------------------- dataset info ------------------------- #
    @staticmethod
    def url() -> str:
        return "https://www.boreas.utias.utoronto.ca/"

    def environment(self) -> str:
        return "Urban Driving"

    def vehicle(self) -> str:
        return "Car"

    # ------------------------- For downloading ------------------------- #
    def files(self) -> Sequence[str | Path]:
        return [
            "lidar",
            "applanix/imu_raw.csv",
            "applanix/lidar_poses.csv",
        ]

    def download(self):
        import boto3
        from botocore import UNSIGNED
        from botocore.config import Config

        # Boreas data is hosted on S3 at s3://boreas (publicly accessible, no sign required)
        # Source: https://www.boreas.utias.utoronto.ca/#/download
        s3 = boto3.client(
            "s3",
            config=Config(signature_version=UNSIGNED),
        )

        bucket = "boreas"
        download_prefixes = [
            f"{self._boreas_folder_name}/lidar/",
            f"{self._boreas_folder_name}/applanix/imu_raw.csv",
            f"{self._boreas_folder_name}/applanix/lidar_poses.csv",
        ]

        self.folder.mkdir(parents=True, exist_ok=True)
        (self.folder / "applanix").mkdir(exist_ok=True)
        (self.folder / "lidar").mkdir(exist_ok=True)

        for dl_prefix in download_prefixes:
            paginator = s3.get_paginator("list_objects_v2")
            for page in paginator.paginate(Bucket=bucket, Prefix=dl_prefix):
                for obj in page.get("Contents", []):
                    key = obj["Key"]
                    # Compute local path relative to the sequence folder
                    relative = key[len(f"{self._boreas_folder_name}/") :]
                    local_path = self.folder / relative
                    local_path.parent.mkdir(parents=True, exist_ok=True)
                    if not local_path.exists():
                        print(f"Downloading {key} -> {local_path}")
                        s3.download_file(bucket, key, str(local_path))

    def quick_len(self) -> Optional[int]:
        return None

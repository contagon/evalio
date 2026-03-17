# from enum import auto
# from pathlib import Path
# from typing import Optional, Sequence

# import numpy as np

# from evalio._cpp.types import Duration, LidarMeasurement, Point, Stamp  # type: ignore
# from evalio.datasets.loaders import RawDataIter
# from evalio.types import (
#     SE3,
#     SO3,
#     ImuMeasurement,
#     ImuParams,
#     LidarParams,
#     Trajectory,
# )

# from .base import Dataset, DatasetIterator
# from .boreas import (
#     _download_boreas_sequence,
#     _load_boreas_bin,
#     _load_boreas_gt,
#     _seq_name_to_folder,
# )


# class BoreasRt(Dataset):
#     """Extended Boreas dataset with newer sequences and a higher-grade IMU.

#     Collected with the same Velodyne Alpha-Prime 128-beam lidar as the original Boreas
#     dataset but adds a Silicon Sensing DMU41 MEMS IMU for higher-fidelity inertial data.
#     Ground truth is from the Applanix post-processed solution.

#     We use the odom_test_rt benchmark split.
#     """

#     # odom_test_rt sequences (15 sequences)
#     # Source: https://github.com/utiasSTARS/pyboreas/blob/master/pyboreas/data/splits.py
#     boreas_2024_12_03_12_54 = auto()
#     boreas_2024_12_10_12_42 = auto()
#     boreas_2025_01_14_11_45 = auto()
#     boreas_2025_01_21_13_56 = auto()
#     boreas_2025_01_28_11_31 = auto()
#     boreas_2025_02_04_12_57 = auto()
#     boreas_2025_02_11_14_51 = auto()
#     boreas_2025_03_04_11_37 = auto()
#     boreas_2025_03_11_11_27 = auto()
#     boreas_2025_03_25_11_54 = auto()
#     boreas_2025_04_08_11_32 = auto()
#     boreas_2025_04_15_11_37 = auto()
#     boreas_2025_04_22_11_19 = auto()
#     boreas_2025_05_13_12_10 = auto()
#     boreas_2025_08_13_09_01 = auto()

#     # ------------------------- For loading data ------------------------- #
#     @property
#     def _boreas_folder_name(self) -> str:
#         """Return the original Boreas folder name (with dashes)."""
#         return _seq_name_to_folder(self.seq_name)

#     def data_iter(self) -> DatasetIterator:
#         # Use the Silicon Sensing DMU41 IMU (infilled at perfect 200 Hz)
#         # Source: https://github.com/utiasSTARS/boreas-devkit / DATA_RT_REFERENCE.md
#         imu_file = self.folder / "imu" / "dmu_imu_infilled.csv"
#         lidar_path = self.folder / "lidar"
#         lidar_params = self.lidar_params()

#         # Load IMU data
#         # Format: time,wx,wy,wz,ax,ay,az
#         # Timestamps are in nanoseconds (int64).
#         # Source: https://github.com/utiasSTARS/boreas-devkit / DATA_RT_REFERENCE.md
#         imu_data = []
#         with open(imu_file) as f:
#             for line in f:
#                 line = line.strip()
#                 if not line or line.startswith("#") or line.lower().startswith("time"):
#                     continue
#                 parts = line.split(",")
#                 if len(parts) < 7:
#                     continue
#                 t_nsec = int(parts[0])
#                 stamp = Stamp.from_nsec(t_nsec)
#                 gyro = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
#                 accel = np.array([float(parts[4]), float(parts[5]), float(parts[6])])
#                 imu_data.append(ImuMeasurement(stamp, gyro, accel))

#         # Setup lidar files
#         lidar_files = sorted(list(lidar_path.glob("*.bin")))
#         lidar_stamps = [Stamp.from_nsec(int(f.stem) * 1000) for f in lidar_files]

#         def lidar_iter():
#             for stamp, lidar_file in zip(lidar_stamps, lidar_files):
#                 yield _load_boreas_bin(lidar_file, stamp, lidar_params)

#         return RawDataIter(
#             lidar_iter(),
#             iter(imu_data),
#             len(lidar_stamps),
#         )

#     def ground_truth_raw(self) -> Trajectory:
#         return _load_boreas_gt(self.folder / "applanix" / "lidar_poses.csv")

#     # ------------------------- For loading params ------------------------- #
#     def imu_T_lidar(self) -> SE3:
#         # T_applanix_lidar for BoreasRT Velodyne (slightly different calibration).
#         # Source: s3://boreas/boreas-2024-12-03-12-54/calib/T_applanix_lidar.txt
#         return SE3.from_mat(
#             np.array(
#                 [
#                     [7.34962e-01, -6.78108e-01, 0.0, 0.0],
#                     [6.78108e-01, 7.34962e-01, 0.0, 0.0],
#                     [0.0, 0.0, 1.0, 0.13],
#                     [0.0, 0.0, 0.0, 1.0],
#                 ]
#             )
#         )

#     def imu_T_gt(self) -> SE3:
#         # Ground truth is in the Applanix (IMU) frame. The Applanix is the reference
#         # frame for all calibrations. The DMU41 is rigidly mounted relative to it.
#         # We report in the DMU41 frame, so we need T_dmu_applanix = T_applanix_dmu^{-1}.
#         # Source: s3://boreas/boreas-2024-12-03-12-54/calib/T_applanix_dmu.txt
#         T_applanix_dmu = SE3.from_mat(
#             np.array(
#                 [
#                     [0.9999510, -0.009823, -0.001420, 0.000000],
#                     [-0.009824, -0.999952, -0.000242, 0.000000],
#                     [-0.001418, 0.000256, -0.999999, -0.150000],
#                     [0.000000, 0.000000, 0.000000, 1.000000],
#                 ]
#             )
#         )
#         # GT is in Applanix frame; IMU is DMU41. Return T_dmu_applanix = inverse(T_applanix_dmu)
#         return T_applanix_dmu.inverse()

#     def imu_params(self) -> ImuParams:
#         # Silicon Sensing DMU41 MEMS IMU
#         # Angular Random Walk: 0.02 deg/sqrt(hr) = 5.82e-5 rad/sqrt(s)  [== rad/s/sqrt(Hz)]
#         # Velocity Random Walk: 0.05 m/s/sqrt(hr) = 8.33e-4 m/s^2/sqrt(s)  [== m/s^2/sqrt(Hz)]
#         # Angular Bias Instability: 0.1 deg/hr = 4.85e-7 rad/s
#         # Linear Bias Instability: 15 ug = 1.47e-4 m/s^2
#         # Source: https://arxiv.org/abs/2602.16870 (BoreasRT dataset paper)
#         # Source: https://www.siliconsensing.com/media/s2lnkn5h/dmu41-datasheet.pdf
#         return ImuParams(
#             gyro=5.82e-5,  # rad/s/sqrt(Hz)  — Angular Random Walk
#             accel=8.33e-4,  # m/s^2/sqrt(Hz)  — Velocity Random Walk
#             gyro_bias=4.85e-7,  # rad/s^2/sqrt(Hz) — Angular Bias Instability
#             accel_bias=1.47e-4,  # m/s^3/sqrt(Hz)  — Linear Bias Instability
#             bias_init=1e-6,
#             integration=1e-6,
#             gravity=np.array([0, 0, -9.81]),  # ENU frame, gravity points down
#             rate=200.0,
#             brand="Silicon Sensing",
#             model="DMU41",
#         )

#     def lidar_params(self) -> LidarParams:
#         # Velodyne Alpha-Prime (VLS-128) — same sensor as original Boreas dataset
#         # 128 channels, 10 Hz, range up to 300 m (10% reflectivity)
#         # Source: https://velodynelidar.com/products/alpha-prime/
#         # Source: https://github.com/utiasSTARS/boreas-devkit / DATA_RT_REFERENCE.md
#         return LidarParams(
#             num_rows=128,
#             num_columns=1800,
#             min_range=0.4,
#             max_range=300.0,
#             rate=10.0,
#             brand="Velodyne",
#             model="VLS-128",
#         )

#     # ------------------------- dataset info ------------------------- #
#     @staticmethod
#     def url() -> str:
#         return "https://www.boreas.utias.utoronto.ca/"

#     def environment(self) -> str:
#         return "Urban Driving"

#     def vehicle(self) -> str:
#         return "Car"

#     # ------------------------- For downloading ------------------------- #
#     def files(self) -> Sequence[str | Path]:
#         return [
#             "lidar",
#             "imu/dmu_imu_infilled.csv",
#             "applanix/lidar_poses.csv",
#         ]

#     def download(self, max_workers: int = 16):
#         _download_boreas_sequence(
#             folder=self.folder,
#             boreas_folder_name=self._boreas_folder_name,
#             download_prefixes=[
#                 f"{self._boreas_folder_name}/lidar/",
#                 f"{self._boreas_folder_name}/imu/dmu_imu_infilled.csv",
#                 f"{self._boreas_folder_name}/applanix/lidar_poses.csv",
#             ],
#             subdirs=["applanix", "imu", "lidar"],
#             max_workers=max_workers,
#         )

#     def quick_len(self) -> Optional[int]:
#         return None

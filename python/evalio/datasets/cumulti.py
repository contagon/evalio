import os
import urllib
import urllib.request
import zipfile
from enum import auto
from pathlib import Path

import numpy as np
from tqdm.rich import tqdm

from evalio.datasets.loaders import (
    LidarDensity,
    LidarFormatParams,
    LidarMajor,
    LidarPointStamp,
    LidarStamp,
    RosbagIter,
)
from evalio.types import SE3, Trajectory

from .base import Dataset, DatasetIterator, ImuParams, LidarParams


# https://github.com/pytorch/vision/blob/fc746372bedce81ecd53732ee101e536ae3afec1/torchvision/datasets/utils.py#L27
def _urlretrieve(url: str, filename: Path, chunk_size: int = 1024 * 32) -> None:
    """
    Retrieves a file from url using urllib
    """
    with urllib.request.urlopen(
        urllib.request.Request(url, headers={"User-Agent": "evalio"})
    ) as response:
        with (
            open(filename, "wb") as fh,
            tqdm(
                total=100e9,  # GLOBUS does not include size in request response, guess=100GB
                unit="B",
                unit_scale=True,
                dynamic_ncols=True,
            ) as pbar,
        ):
            while chunk := response.read(chunk_size):
                fh.write(chunk)
                pbar.update(len(chunk))


def _extract_noreplace(zip_file: Path, dest_dir: Path):
    """
    Extracts files from a zip archive (zip_file) but skips any file that already exists at the destination
    (Allows to resume zip extractions)
    """
    print(f"Extracting: {zip_file.name}")
    with zipfile.ZipFile(str(zip_file)) as archive:
        with tqdm(total=len(archive.namelist()), dynamic_ncols=True) as pbar:
            for filename in archive.namelist():
                if not (dest_dir / filename).is_file():
                    archive.extract(filename, path=dest_dir)
                pbar.update()


class CUMulti(Dataset):
    """
    Dataset collected by a ground robot (AgileX - Hunter) on the University of Colorado Boulder Campus.
    """

    kittredge_loop_robot1 = auto()
    kittredge_loop_robot2 = auto()
    kittredge_loop_robot3 = auto()
    kittredge_loop_robot4 = auto()
    main_campus_robot1 = auto()
    main_campus_robot2 = auto()
    main_campus_robot3 = auto()
    main_campus_robot4 = auto()

    # ------------------------- For loading data ------------------------- #
    def data_iter(self) -> DatasetIterator:
        lidar_format = LidarFormatParams(
            stamp=LidarStamp.Start,
            point_stamp=LidarPointStamp.Start,
            major=LidarMajor.Row,
            density=LidarDensity.AllPoints,
        )

        robot_name = self.seq_name.split("_")[-1]
        return RosbagIter(
            self.folder,
            f"{robot_name}/ouster/points",
            f"{robot_name}/imu/data",
            self.lidar_params(),
            lidar_format=lidar_format,
        )

    def ground_truth_raw(self) -> Trajectory:
        # Sequence Naming information
        components = self.seq_name.split("_")
        robot_name = components[-1]
        loc_name = "_".join(components[:2])
        gt_file = self.folder / f"{loc_name}_{robot_name}_ref.csv"

        # Load the Trajectory which provides solutions in the UTM frame
        traj_utm = Trajectory.from_csv(
            gt_file, ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"]
        )

        # Subtract out the initial position to get a local frame reference
        init_position = traj_utm.poses[0].trans
        return Trajectory(
            poses=[SE3(p.rot, p.trans - init_position) for p in traj_utm.poses],
            stamps=traj_utm.stamps,
        )

    # ------------------------- For loading params ------------------------- #
    def imu_T_lidar(self) -> SE3:
        # Supplied by CU-Multi Authors
        return SE3.fromMat(
            np.array(
                [
                    [-1.0, 0.0, 0.0, -0.058038],
                    [0.0, -1.0, 0.0, 0.015573],
                    [0.0, 0.0, 1.0, 0.049603],
                    [0.0, 0.0, 0.0, 1.000000],
                ]
            )
        )

    def imu_T_gt(self) -> SE3:
        # Groundtruth provided in the IMU Frame
        return SE3.identity()

    def imu_params(self) -> ImuParams:
        # https://www.mouser.com/datasheet/2/1083/3dmgq7_gnssins_ds_0-1900596.pdf
        return ImuParams(
            gyro=4.363323129985824e-05,  # From 0.15 Deg / sqrt(hr)
            accel=1.962e-4,  # From 20 micro-gravity / sqrt(Hz)
            gyro_bias=1e-6,  # TODO (dan)
            accel_bias=1e-6,  # TODO (dan)
            bias_init=1e-8,
            integration=1e-8,
            gravity=np.array([0, 0, -9.81]),
            brand="MicroStrain",
            model="3DM-GQ7",
        )

    def lidar_params(self) -> LidarParams:
        # Ouster OS 64 Rev 7
        return LidarParams(
            rate=20,
            num_rows=64,
            num_columns=1024,
            min_range=1.0,
            max_range=200.0,
            brand="Ouster",
            model="OS-64 (Rev 7)",
        )

    # ------------------------- dataset info ------------------------- #
    @staticmethod
    def url() -> str:
        return "https://arpg.github.io/cumulti/"

    @classmethod
    def dataset_name(cls) -> str:
        return "cumulti"

    def environment(self) -> str:
        return "CU Boulder Campus"

    def vehicle(self) -> str:
        return "ATV"

    # ------------------------- For downloading ------------------------- #
    def files(self) -> list[str]:
        components = self.seq_name.split("_")
        robot_name = components[-1]
        loc_name = "_".join(components[:2])

        return [
            self.folder / f"{robot_name}_{loc_name}_imu_gps",  # IMU Bag
            self.folder / f"{robot_name}_{loc_name}_lidar",  # LIDAR Bag
            self.folder / f"{loc_name}_{robot_name}_ref.csv",  # Reference Solution
        ]

    def download(self):
        # Sequence Naming information
        components = self.seq_name.split("_")
        robot_name = components[-1]
        loc_name = "_".join(components[:2])

        # File Download URLS
        globus_url = r"https://g-ad45ee.3d2bab.75bc.data.globus.org"
        seq_url_base_path = (
            f"{globus_url}/{loc_name}/{robot_name}/{robot_name}_{loc_name}"
        )
        lidar_url = f"{seq_url_base_path}_lidar.zip"
        imu_url = f"{seq_url_base_path}_imu_gps.zip"

        # Download destination zip files
        lidar_zip_file = self.folder / f"{robot_name}_{loc_name}_lidar.zip"
        imu_zip_file = self.folder / f"{robot_name}_{loc_name}_imu_gps.zip"

        # Download all of the data
        print(f"Downloading to {self.folder}...")
        self.folder.mkdir(parents=True, exist_ok=True)
        for zip_file, url in [(lidar_zip_file, lidar_url), (imu_zip_file, imu_url)]:
            if not zip_file.is_file():
                print(url)
                _urlretrieve(url, zip_file)
            else:
                print("Archive already exists. Skipping Download.")

            # Extract from the zip only what we dont already have
            print("Extracting data...")
            _extract_noreplace(zip_file, self.folder)

        # Download the groundtruth
        gt_url = f"{globus_url}/{loc_name}/{robot_name}/{loc_name}_{robot_name}/"
        gt_file = self.folder / f"{loc_name}_{robot_name}_ref.csv"
        print(gt_url)
        if not gt_file.is_file():
            _urlretrieve(gt_url, gt_file)
        else:
            print("Groundtruth already exists. Skipping Download.")

        # If we have extracted everything we need then remove the zip directory
        if self.is_downloaded():
            print("All data downloaded. Removing zip files...")
            for zip_file in [lidar_zip_file, imu_zip_file]:
                os.remove(zip_file)

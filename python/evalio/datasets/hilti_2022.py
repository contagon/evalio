from enum import auto
import urllib
import urllib.request
from pathlib import Path

from evalio.datasets.loaders import (
    LidarDensity,
    LidarFormatParams,
    LidarMajor,
    LidarPointStamp,
    LidarStamp,
    RosbagIter,
)
from evalio.types import Trajectory, SO3
import numpy as np
from tqdm import tqdm

from .base import (
    SE3,
    Dataset,
    ImuParams,
    LidarParams,
    DatasetIterator,
)


# https://github.com/pytorch/vision/blob/fc746372bedce81ecd53732ee101e536ae3afec1/torchvision/datasets/utils.py#L27
def _urlretrieve(url: str, filename: Path, chunk_size: int = 1024 * 32) -> None:
    with urllib.request.urlopen(
        urllib.request.Request(url, headers={"User-Agent": "evalio"})
    ) as response:
        with (
            open(filename, "wb") as fh,
            tqdm(
                total=response.length, unit="B", unit_scale=True, dynamic_ncols=True
            ) as pbar,
        ):
            while chunk := response.read(chunk_size):
                fh.write(chunk)
                pbar.update(len(chunk))


class Hilti2022(Dataset):
    construction_upper_level_1 = auto()
    construction_upper_level_2 = auto()
    construction_upper_level_3 = auto()
    basement_2 = auto()
    attic_to_upper_gallery_2 = auto()
    corridor_lower_gallery_2 = auto()

    # ------------------------- For loading data ------------------------- #
    def data_iter(self) -> DatasetIterator:
        bag, _ = self.files()
        return RosbagIter(
            self.folder / bag,
            "/hesai/pandar",
            "/alphasense/imu",
            self.lidar_params(),
            lidar_format=LidarFormatParams(
                stamp=LidarStamp.Start,
                point_stamp=LidarPointStamp.Start,
                major=LidarMajor.Column,
                density=LidarDensity.OnlyValidPoints,
            ),
        )

    def ground_truth_raw(self) -> Trajectory:
        # TODO: Update the path to the ground truth file
        _, gt = self.files()
        return Trajectory.load_csv(
            self.folder / gt,
            ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"],
            delimiter=" ",
        )

    # ------------------------- For loading params ------------------------- #
    @staticmethod
    def url() -> str:
        return "https://hilti-challenge.com/dataset-2022.html"

    def imu_T_lidar(self) -> SE3:
        return SE3(
            SO3(qx=0.7071068, qy=-0.7071068, qz=0.0, qw=0.0),
            np.array([-0.001, -0.00855, 0.055]),
        )

    def imu_T_gt(self) -> SE3:
        return SE3.identity()

    def imu_params(self) -> ImuParams:
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
            num_rows=32,
            num_columns=2000,
            # Increase this a smidge to remove vehicle from scan
            min_range=0.5,
            max_range=120.0,
        )

    # ------------------------- For downloading ------------------------- #
    def files(self) -> list[str]:
        filename = {
            "construction_upper_level_1": "exp04_construction_upper_level",
            "construction_upper_level_2": "exp05_construction_upper_level_2",
            "construction_upper_level_3": "exp06_construction_upper_level_3",
            "basement_2": "exp14_basement_2",
            "attic_to_upper_gallery_2": "exp16_attic_to_upper_gallery_2",
            "corridor_lower_gallery_2": "exp18_corridor_lower_gallery_2",
        }[self.seq_name]

        bag_file = f"{filename}.bag"
        gt_file = f"{filename}_imu.txt"

        # Extra space in these ones for some reason
        if "construction" in self.seq_name:
            gt_file = "exp_" + gt_file[3:]

        return [bag_file, gt_file]

    def download(self):
        bag_file, gt_file = self.files()

        url = "https://tp-public-facing.s3.eu-north-1.amazonaws.com/Challenges/2022/"

        print(f"Downloading to {self.folder}...")
        self.folder.mkdir(parents=True, exist_ok=True)
        if not (self.folder / gt_file).exists():
            print(f"Downloading {gt_file}")
            _urlretrieve(url + gt_file, self.folder / gt_file)
        if not (self.folder / bag_file).exists():
            print(f"Downloading {bag_file}")
            _urlretrieve(url + bag_file, self.folder / bag_file)

import urllib
import urllib.request
from dataclasses import dataclass
from pathlib import Path

from evalio.types import Trajectory
import numpy as np
from tqdm import tqdm

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


@dataclass
class Hilti2022(Dataset):
    # ------------------------- For loading data ------------------------- #
    def __iter__(self):
        bag, _ = Hilti2022.get_files(self.seq)
        return RosbagIter(
            EVALIO_DATA / Hilti2022.name() / self.seq / bag,
            "/hesai/pandar",
            "/alphasense/imu",
            self.lidar_params(),
        )

    def ground_truth_raw(self) -> Trajectory:
        # TODO: Update the path to the ground truth file
        _, gt = Hilti2022.get_files(self.seq)
        return load_pose_csv(
            EVALIO_DATA / Hilti2022.name() / self.seq / gt,
            ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"],
            delimiter=" ",
        )

    # ------------------------- For loading params ------------------------- #
    @staticmethod
    def url() -> str:
        return "https://hilti-challenge.com/dataset-2022.html"

    @staticmethod
    def name() -> str:
        return "hilti_2022"

    @staticmethod
    def sequences() -> list[str]:
        return [
            "construction_upper_level_1",
            "construction_upper_level_2",
            "construction_upper_level_3",
            "basement_2",
            "attic_to_upper_gallery_2",
            "corridor_lower_gallery_2",
        ]

    def imu_T_lidar(self) -> SE3:
        return SE3(
            SO3(qx=0.7071068, qy=-0.7071068, qz=0.0, qw=0.0),
            np.array([-0.001, -0.00855, 0.055]),
        )

    def imu_T_gt(self) -> SE3:
        return SE3.identity()

    def imu_params(self) -> ImuParams:
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

    def lidar_params(self) -> LidarParams:
        return LidarParams(
            num_rows=32,
            num_columns=2000,
            min_range=0.1,
            max_range=120.0,
        )

    # ------------------------- For downloading ------------------------- #
    @staticmethod
    def get_files(seq: str) -> tuple[str, str]:
        filename = {
            "construction_upper_level_1": "exp04_construction_upper_level",
            "construction_upper_level_2": "exp05_construction_upper_level_2",
            "construction_upper_level_3": "exp06_construction_upper_level_3",
            "basement_2": "exp14_basement_2",
            "attic_to_upper_gallery_2": "exp16_attic_to_upper_gallery_2",
            "corridor_lower_gallery_2": "exp18_corridor_lower_gallery_2",
        }[seq]

        bag_file = f"{filename}.bag"
        gt_file = f"{filename}_imu.txt"

        # Extra space in these ones for some reason
        if "construction" in seq:
            gt_file = "exp_" + gt_file[3:]

        return bag_file, gt_file

    @staticmethod
    def check_download(seq: str) -> bool:
        folder = EVALIO_DATA / Hilti2022.name() / seq

        bag_file, gt_file = Hilti2022.get_files(seq)

        if not folder.exists():
            return False
        elif not (folder / gt_file).exists():
            return False
        elif not (folder / bag_file).exists():
            return False
        else:
            return True

    @staticmethod
    def download(seq: str):
        bag_file, gt_file = Hilti2022.get_files(seq)

        folder = EVALIO_DATA / Hilti2022.name() / seq
        url = "https://tp-public-facing.s3.eu-north-1.amazonaws.com/Challenges/2022/"

        print(f"Downloading to {folder}...")
        folder.mkdir(parents=True, exist_ok=True)
        if not (folder / gt_file).exists():
            print(f"Downloading {gt_file}")
            _urlretrieve(url + gt_file, folder / gt_file)
        if not (folder / bag_file).exists():
            print(f"Downloading {bag_file}")
            _urlretrieve(url + bag_file, folder / bag_file)

import urllib
import urllib.request
from pathlib import Path
from enum import auto

from evalio.datasets.loaders import (
    LidarDensity,
    LidarFormatParams,
    LidarMajor,
    LidarPointStamp,
    LidarStamp,
    RosbagIter,
)
from evalio.types import Trajectory, SE3, SO3, ImuParams, LidarParams
import numpy as np
from tqdm.rich import tqdm

from .base import (
    Dataset,
    DatasetIterator,
)

from typing import Sequence, cast, Optional


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


class EnWide(Dataset):
    """Dataset taken in purposely degenerate locations such as a field, intersections, tunnels, and runways. All data comes directly from the Ouster unit.

    Note, this dataset does not have ground truth orientation, only ground truth positional values taken from a Leica MS60 Prism.
    """

    field_d = auto()
    field_s = auto()
    intersection_d = auto()
    intersection_s = auto()
    katzensee_d = auto()
    katzensee_s = auto()
    runway_d = auto()
    runway_s = auto()
    tunnel_d = auto()
    tunnel_s = auto()

    # ------------------------- For loading data ------------------------- #
    def data_iter(self) -> DatasetIterator:
        return RosbagIter(
            self.folder,
            "/ouster/points",
            "/ouster/imu",
            self.lidar_params(),
            lidar_format=LidarFormatParams(
                stamp=LidarStamp.Start,
                point_stamp=LidarPointStamp.Start,
                major=LidarMajor.Row,
                density=LidarDensity.AllPoints,
            ),
        )

    def ground_truth_raw(self) -> Trajectory:
        return Trajectory.from_csv(
            self.folder / f"gt-{self.seq_name}.csv",
            ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"],
            delimiter=" ",
        )

    # ------------------------- For loading params ------------------------- #
    def imu_T_lidar(self) -> SE3:
        scale = 100
        imu_T_sensor = SE3(
            SO3(qx=0.0, qy=0.0, qz=0.0, qw=1.0),
            np.array([6.253 / scale, -11.775 / scale, 7.645 / scale]),
        )
        lidar_T_sensor = SE3(
            SO3(qx=0.0, qy=0.0, qz=1.0, qw=0.0),
            np.array([0.0, 0.0, 0.3617 / scale]),
        )
        # TODO: Hardcode this later on
        return imu_T_sensor * lidar_T_sensor.inverse()

    def imu_T_gt(self) -> SE3:
        # TODO: Needs to be inverted?
        return SE3(
            SO3(qx=0.0, qy=0.0, qz=0.0, qw=1.0),
            np.array([-0.006253, 0.011775, 0.10825]),
        )

    def imu_params(self) -> ImuParams:
        # TODO: Verify these values
        return ImuParams(
            gyro=0.000261799,
            accel=0.000230,
            gyro_bias=0.0000261799,
            accel_bias=0.0000230,
            bias_init=1e-7,
            integration=1e-7,
            gravity=np.array([0, 0, 9.81]),
            brand="TDK",
            model="IAM-20680HT",
        )

    def lidar_params(self) -> LidarParams:
        return LidarParams(
            num_rows=128,
            num_columns=1024,
            min_range=0.0,
            max_range=100.0,
            brand="Ouster",
            model="OS0-128",
        )

    # ------------------------- dataset info ------------------------- #
    @classmethod
    def dataset_name(cls) -> str:
        return "enwide"

    @staticmethod
    def url() -> str:
        return "https://projects.asl.ethz.ch/datasets/enwide"

    def environment(self) -> str:
        if "tunnel" in self.seq_name:
            return "Tunnel"
        elif "runway" in self.seq_name or "intersection" in self.seq_name:
            return "Planar Road"
        else:
            return "Field"

    def vehicle(self) -> str:
        return "Handheld"

    # ------------------------- For downloading ------------------------- #
    def files(self) -> Sequence[str | Path]:
        return {
            "intersection_s": [
                "2023-08-09-16-19-09-intersection_s.bag",
                "gt-intersection_s.csv",
            ],
            "runway_s": ["2023-08-09-18-44-24-runway_s.bag", "gt-runway_s.csv"],
            "katzensee_s": [
                "2023-08-21-10-20-22-katzensee_s.bag",
                "gt-katzensee_s.csv",
            ],
            "runway_d": ["2023-08-09-18-52-05-runway_d.bag", "gt-runway_d.csv"],
            "tunnel_d": ["2023-08-08-17-50-31-tunnel_d.bag", "gt-tunnel_d.csv"],
            "field_d": ["2023-08-09-19-25-45-field_d.bag", "gt-field_d.csv"],
            "katzensee_d": [
                "2023-08-21-10-29-20-katzensee_d.bag",
                "gt-katzensee_d.csv",
            ],
            "tunnel_s": ["2023-08-08-17-12-37-tunnel_s.bag", "gt-tunnel_s.csv"],
            "intersection_d": [
                "2023-08-09-17-58-11-intersection_d.bag",
                "gt-intersection_d.csv",
            ],
            "field_s": ["2023-08-09-19-05-05-field_s.bag", "gt-field_s.csv"],
        }[self.seq_name]

    def download(self):
        bag_file, gt_file = cast(list[str], self.files())

        url = f"http://robotics.ethz.ch/~asl-datasets/2024_ICRA_ENWIDE/{self.seq_name}/"

        print(f"Downloading to {self.folder}...")
        self.folder.mkdir(parents=True, exist_ok=True)
        if not (self.folder / gt_file).exists():
            _urlretrieve(url + gt_file, self.folder / gt_file)
        if not (self.folder / bag_file).exists():
            _urlretrieve(url + bag_file, self.folder / bag_file)

    def quick_len(self) -> Optional[int]:
        return {
            "field_d": 1477,
            "field_s": 1671,
            "intersection_d": 1828,
            "intersection_s": 1997,
            "katzensee_d": 858,
            "katzensee_s": 1620,
            "runway_d": 1902,
            "runway_s": 2238,
            "tunnel_d": 1189,
            "tunnel_s": 2380,
        }[self.seq_name]

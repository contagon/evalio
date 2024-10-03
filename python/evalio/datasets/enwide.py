import numpy as np

from .base import (
    EVALIO_DATA,
    Dataset,
    RosbagIter,
    ImuParams,
    LidarParams,
    load_pose_csv,
    SO3,
    SE3,
    Stamp,
)

from tqdm import tqdm
from pathlib import Path
import urllib
import urllib.request


# https://github.com/pytorch/vision/blob/fc746372bedce81ecd53732ee101e536ae3afec1/torchvision/datasets/utils.py#L27
def _urlretrieve(url: str, filename: Path, chunk_size: int = 1024 * 32) -> None:
    with urllib.request.urlopen(
        urllib.request.Request(url, headers={"User-Agent": "evalio"})
    ) as response:
        with open(filename, "wb") as fh, tqdm(
            total=response.length, unit="B", unit_scale=True
        ) as pbar:
            while chunk := response.read(chunk_size):
                fh.write(chunk)
                pbar.update(len(chunk))


class EnWide(Dataset):
    # ------------------------- For loading data ------------------------- #
    def __iter__(self):
        return RosbagIter(
            EVALIO_DATA / EnWide.name() / self.seq,
            "/ouster/points",
            "/ouster/imu",
        )

    def ground_truth(self) -> list[(Stamp, SE3)]:
        return load_pose_csv(
            EVALIO_DATA / EnWide.name() / self.seq / f"gt-{self.seq}.csv",
            ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"],
            delimiter=" ",
        )

    # ------------------------- For loading params ------------------------- #
    @staticmethod
    def name() -> str:
        return "enwide"

    @staticmethod
    def nickname() -> str:
        return "enwide"

    @staticmethod
    def sequences() -> list[str]:
        return [
            "field_d",
            "field_s",
            "intersection_d",
            "intersection_s",
            "katzenee_d",
            "katzenee_s",
            "runway_d",
            "runway_s",
            "tunnel_d",
            "tunnel_s",
        ]

    @staticmethod
    def nicksequences() -> list[str]:
        return EnWide.sequences()

    @staticmethod
    def imu_T_lidar() -> SE3:
        scale = 100
        imu_T_sensor = SE3(
            SO3(qx=0.0, qy=0.0, qz=0.0, qw=1.0),
            [6.253 / scale, -11.775 / scale, 7.645 / scale],
        )
        lidar_T_sensor = SE3(
            SO3(qx=0.0, qy=0.0, qz=1.0, qw=0.0),
            [0.0, 0.0, 0.3617 / scale],
        )
        # TODO: Hardcode this later on
        return imu_T_sensor * lidar_T_sensor.inverse()

    @staticmethod
    def imu_T_gt() -> SE3:
        # TODO: Needs to be inverted?
        return SE3(
            SO3(qx=0.0, qy=0.0, qz=0.0, qw=1.0),
            [-0.006253, 0.011775, 0.10825],
        )

    @staticmethod
    def imu_params() -> ImuParams:
        # TODO: Verify these values
        return ImuParams(
            gyro=0.000261799,
            accel=0.000230,
            gyro_bias=0.0000261799,
            accel_bias=0.0000230,
            bias_init=1e-7,
            integration=1e-7,
            gravity=np.array([0, 0, 9.81]),
        )

    @staticmethod
    def lidar_params() -> LidarParams:
        return LidarParams(
            num_rows=128,
            num_columns=1024,
            min_range=0.0,
            max_range=100.0,
        )

    # ------------------------- For downloading ------------------------- #
    @staticmethod
    def check_download(seq: str) -> bool:
        dir = EVALIO_DATA / EnWide.name() / seq

        if not dir.exists():
            return False
        elif not (dir / f"gt-{seq}.csv").exists():
            return False
        elif len(list(dir.glob("*.bag"))) == 0:
            return False
        elif len(list(dir.glob("*.bag"))) > 1:
            raise ValueError(f"Too many bag files found, should only be 1 in {dir}")
        else:
            return True

    @staticmethod
    def download(seq: str):
        bag_date = {
            "field_d": "2023-08-09-19-25-45",
            "field_s": "2023-08-09-19-05-05",
            "intersection_d": "2023-08-09-17-58-11",
            "intersection_s": "2023-08-09-16-19-09",
            "katzenee_d": "2023-08-21-10-29-20",
            "katzenee_s": "2023-08-21-10-20-22",
            "runway_d": "2023-08-09-18-52-05",
            "runway_s": "2023-08-09-18-44-24",
            "tunnel_d": "2023-08-08-17-50-31",
            "tunnel_s": "2023-08-08-17-12-37",
        }[seq]
        bag_file = f"{bag_date}-{seq}.bag"
        gt_file = f"gt-{seq}.csv"

        folder = EVALIO_DATA / EnWide.name() / seq
        url = f"http://robotics.ethz.ch/~asl-datasets/2024_ICRA_ENWIDE/{seq}/"

        print(f"Making folder {folder}...")
        folder.mkdir(parents=True, exist_ok=True)

        print(f"Downloading to {folder}...")
        if not (folder / gt_file).exists():
            _urlretrieve(url + gt_file, folder / gt_file)
        if not (folder / bag_file).exists():
            _urlretrieve(url + bag_file, folder / bag_file)

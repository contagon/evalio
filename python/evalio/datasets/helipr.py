import tarfile
from dataclasses import dataclass

import numpy as np

from evalio.types import Trajectory
from evalio._cpp._helpers import helipr_bin_to_evalio  # type: ignore

from .base import (
    EVALIO_DATA,
    SE3,
    SO3,
    Dataset,
    ImuParams,
    LidarParams,
    load_pose_csv,
    Stamp,
    ImuMeasurement,
    DatasetIterator,
)
from .iterators import RawDataIter


"""
Note, we do everything based off of the Ouster Lidar, mounted at the top of the vehicle.
"""


@dataclass
class HeLiPR(Dataset):
    # ------------------------- For loading data ------------------------- #
    def data_iter(self) -> DatasetIterator:
        imu_file = EVALIO_DATA / HeLiPR.name() / self.seq / "xsens_imu.csv"

        # Load in all IMU data
        imu_stamps = np.loadtxt(imu_file, usecols=0, dtype=np.int64, delimiter=",")
        imu_stamps = [Stamp.from_nsec(x) for x in imu_stamps]
        imu_data = np.loadtxt(imu_file, usecols=(11, 12, 13, 14, 15, 16), delimiter=",")
        assert len(imu_stamps) == len(imu_data)
        imu_data = [
            ImuMeasurement(stamp, gyro, acc)
            for stamp, gyro, acc in zip(imu_stamps, imu_data[:, 3:], imu_data[:, :3])
        ]

        # setup all the lidar data
        lidar_path = EVALIO_DATA / HeLiPR.name() / self.seq / "Ouster"
        lidar_files = sorted(list(lidar_path.glob("*")))
        lidar_stamps = [Stamp.from_nsec(int(x.stem)) for x in lidar_files]
        lidar_params = self.lidar_params()

        def lidar_iter():
            for stamp, file in zip(lidar_stamps, lidar_files):
                mm = helipr_bin_to_evalio(str(file), stamp, lidar_params)
                yield mm

        return RawDataIter(
            lidar_iter(),
            imu_data.__iter__(),
        )

    def ground_truth_raw(self) -> Trajectory:
        return load_pose_csv(
            EVALIO_DATA / HeLiPR.name() / self.seq / "Ouster_gt.txt",
            ["nsec", "x", "y", "z", "qx", "qy", "qz", "qw"],
            delimiter=" ",
        )

    # ------------------------- For loading params ------------------------- #
    @staticmethod
    def url() -> str:
        return "https://sites.google.com/view/heliprdataset/"

    @staticmethod
    def name() -> str:
        return "helipr"

    @staticmethod
    def sequences() -> list[str]:
        # Had to remove a couple of them due to not having imu data
        return [
            # "kaist_04",
            "kaist_05",
            "kaist_06",
            # "dcc_04",
            "dcc_05",
            "dcc_06",
            # "riverside_04",
            "riverside_05",
            "riverside_06",
        ]

    def imu_T_lidar(self) -> SE3:
        return SE3(
            SO3.fromMat(
                np.array(
                    [
                        [0.999715495593027, 0.0223448061210468, -0.00834490926264448],
                        [-0.0224514077723064, 0.999664614804883, -0.0129070599303583],
                        [0.00805370475188661, 0.0130907427756056, 0.999881878170293],
                    ]
                )
            ),
            np.array([-0.41696532, -0.00301141, 0.2996]),
        )

    def imu_T_gt(self) -> SE3:
        # GT is in the Ouster frame
        return self.imu_T_lidar()

    def imu_params(self) -> ImuParams:
        # Xsens MTi-300
        # https://www.xsens.com/hubfs/Downloads/Leaflets/MTi-300.pdf
        return ImuParams(
            gyro=0.000174532925199,
            accel=0.00014715,
            gyro_bias=0.000174532925199,
            accel_bias=0.00014715,
            bias_init=1e-8,
            integration=1e-8,
            gravity=np.array([0, 0, -9.81]),
        )

    def lidar_params(self) -> LidarParams:
        return LidarParams(
            num_rows=128,
            # TODO: This seems wrong... but it's what I'm getting out of the data
            num_columns=1025,
            min_range=1.0,
            max_range=200.0,
        )

    # ------------------------- For downloading ------------------------- #
    @staticmethod
    def check_download(seq: str) -> bool:
        dir = EVALIO_DATA / HeLiPR.name() / seq
        if not dir.exists():
            return False
        elif not (dir / "Ouster_gt.txt").exists():
            return False
        elif not (dir / "xsens_imu.csv").exists():
            return False
        elif not (dir / "Ouster").exists():
            return False
        else:
            return True

    @staticmethod
    def download(seq: str):
        id_gt = {
            # "kaist_04": "",
            "kaist_05": "17S4649polOCfN0IOBCCxQ85UrdXf1jhF",
            "kaist_06": "1GzU4gaQKL1XPtM5t4HWw2r6EvKJOWJV_",
            # "dcc_04": "1ZG3muCWwrZrVhtOsSTUMBWdUhieBZnZy",
            "dcc_05": "1_GrcOKatx6F0DqglW0L9vk4ENBVFs0ZV",
            "dcc_06": "1Bxbl3T3OGrtn8Gh8BfeV8KJjwkgRtBMv",
            # "riverside_04": "",
            "riverside_05": "1IWj2bI7D5mPZWoh_09x0nyaM2Uf36VJM",
            "riverside_06": "1z7i2kxQV7edKmBtJ0jTjQKlShbc7MTeJ",
        }[seq]

        id_imu = {
            # "kaist_04": "",
            "kaist_05": "1R0Z7Z9BAhqOSNsD1ft6vqenH644rxXk9",
            "kaist_06": "1X-KiLi26PpJpTbZ3xupBq8I74OdXlSYU",
            # "dcc_04": "1u5eZK1sP_Jr3XasZ7PFKFGryVLuTUpGp",
            "dcc_05": "1X3_BJsVyaQZ7yL5t0stqr7j-G1K1sq5R",
            "dcc_06": "1WqxElIxpXR1uXTIRTow2mZgPFooy2BBt",
            # "riverside_04": "",
            "riverside_05": "1mqiOrq9gJLtrZn6QZdEts4yM2rGk_xfY",
            "riverside_06": "11EvRS44Eny6uwaT0RyfYRCxwvOPzSN4X",
        }[seq]

        id_lidar = {
            # "kaist_04": "",
            "kaist_05": "1FM5L17x12Lh3byp9m4h8njB-4RCP4HeZ",
            "kaist_06": "1FgNG22gYkOTaWN5mtaXmR2RHbFYJ5ufo",
            # "dcc_04": "1WcUkHU-kuH_g7GCSHJDrpkuJu_xm5Wfx",
            "dcc_05": "1YqCnquttT6WXcYyHnq4pzhgEGoEjvY4p",
            "dcc_06": "1tThqQ706gUuoV29g2l1c2vjEHbOfvKsa",
            # "riverside_04": "",
            "riverside_05": "1v1ZdYkNwlxXuFCpTug80pNdFfc5ZTWWG",
            "riverside_06": "1-ll0t8goMGb0Qb86nvkerC4quJS3mzPU",
        }[seq]

        import gdown  # type: ignore

        folder = EVALIO_DATA / HeLiPR.name() / seq

        print(f"Downloading {seq} to {folder}...")
        folder.mkdir(parents=True, exist_ok=True)
        folder_trail = f"{folder}/"
        gdown.download(id=id_gt, output=folder_trail, resume=True)
        gdown.download(id=id_imu, output=folder_trail, resume=True)

        if not (folder / "Ouster").exists():
            gdown.download(id=id_lidar, output=folder_trail, resume=True)
            # extract the lidar data
            print("Extracting lidar data...")
            tar_file = folder / "Ouster.tar.gz"
            with tarfile.open(str(tar_file)) as tar:
                tar.extractall(path=folder)
            print("Removing tar file...")
            tar_file.unlink()

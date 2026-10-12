import urllib
import urllib.request
from collections.abc import Iterator, Sequence
from dataclasses import dataclass
from enum import auto
from pathlib import Path
from typing import Any, Optional

import numpy as np
from tqdm.rich import tqdm

from evalio.datasets.loaders import RawDataIter
from evalio.types import (
    SE3,
    SO3,
    Duration,
    ImuMeasurement,
    ImuParams,
    LidarMeasurement,
    LidarParams,
    Point,
    Stamp,
    Trajectory,
)

from .base import (
    Dataset,
    DatasetIterator,
)

"""
All sequences are recorded with the same Mandeye handheld scanner, a Livox Mid-360 with its built-in IMU.
Data is stored in the raw Mandeye chunk layout: lidarNNNN.laz and imuNNNN.csv on a shared clock.
"""


# https://github.com/pytorch/vision/blob/fc746372bedce81ecd53732ee101e536ae3afec1/torchvision/datasets/utils.py#L27
def _urlretrieve(url: str, filename: Path, chunk_size: int = 1024 * 32) -> None:
    with (
        urllib.request.urlopen(
            urllib.request.Request(url, headers={"User-Agent": "evalio"})
        ) as response,
        open(filename, "wb") as fh,
        tqdm(
            total=response.length, unit="B", unit_scale=True, dynamic_ncols=True
        ) as pbar,
    ):
        while chunk := response.read(chunk_size):
            fh.write(chunk)
            pbar.update(len(chunk))


@dataclass
class _SequenceInfo:
    hf_repo: str
    # folder of the chunk files inside the Hugging Face repo
    subdir: str
    num_laz: int
    num_csv: int
    environment: str
    # TUM trajectory (HDMapping step 3 export), path inside the repo; stored next to the chunks locally
    gt_file: Optional[str] = None
    quick_len: Optional[int] = None


# Adding a recording is one entry here plus a member in Mandeye, e.g. the headered IMU csv / ns stamps of
# MichalPelka/MandeyeMappingLoopClosure (subdir="", 67 laz, 66 csv) are handled by the loaders below
_SEQUENCES = {
    "underground_parking": _SequenceInfo(
        hf_repo="MichalPelka/UndegroundParkingDataset",
        subdir="mandeye",
        num_laz=25,
        num_csv=25,
        environment="Underground Parking",
        gt_file="ground-truth/trajectory.tum",
        quick_len=2724,
    ),
}

_IMU_ID = 0
_SCAN_PERIOD_NS = 100_000_000
_G = 9.81


def _load_imu(file: Path) -> np.ndarray:
    """Rows of (stamp [ns], gyro [rad/s], accel [g]) for IMU id 0.

    Handles both the headered format and the legacy headerless "ts gx gy gz ax ay az [imuId [unix]]".
    """
    with open(file) as f:
        first = f.readline().split()
    if first and first[0] == "timestamp":
        col = {name: i for i, name in enumerate(first)}
        names = ["timestamp", "gyroX", "gyroY", "gyroZ", "accX", "accY", "accZ"]
        data = np.loadtxt(file, skiprows=1, ndmin=2)
        if data.size == 0:
            return np.zeros((0, 7))
        if "imuId" in col:
            data = data[data[:, col["imuId"]] == _IMU_ID]
        return data[:, [col[n] for n in names]]

    data = np.loadtxt(file, ndmin=2)
    if data.size == 0:
        return np.zeros((0, 7))
    if data.shape[1] > 7:
        data = data[data[:, 7] == _IMU_ID]
    return data[:, :7]


def _load_laz(file: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Time-sorted (stamps [ns], xyz, intensity) of a Mandeye chunk."""
    import laspy  # pyright: ignore[reportMissingTypeStubs]

    las: Any = laspy.read(file)  # pyright: ignore[reportUnknownMemberType]
    t = np.asarray(las.gps_time, dtype=np.float64)
    xyz = np.stack([np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)], axis=1)
    intensity = np.asarray(las.intensity, dtype=np.float64)

    keep = t != 0
    t, xyz, intensity = t[keep], xyz[keep], intensity[keep]
    # A chunk covers a few seconds, so a span below 1000 means seconds rather than nanoseconds
    if t.size > 0 and t.max() - t.min() < 1e3:
        t = t * 1e9
    stamps = np.round(t).astype(np.int64)

    order = np.argsort(stamps, kind="stable")
    return stamps[order], xyz[order], intensity[order]


class Mandeye(Dataset):
    """Handheld datasets recorded with the [Mandeye](https://github.com/JanuszBedkowski/mandeye_controller) open-hardware scanner, built around a Livox Mid-360 and its internal IMU.

    The raw data is stored in chunks of a few seconds, which are cut into 0.1 s scans. The underground parking sequence is accompanied by a terrestrial laser scan of the garage.
    """

    underground_parking = auto()

    @property
    def _info(self) -> _SequenceInfo:
        return _SEQUENCES[self.seq_name]

    def _laz_files(self) -> list[str]:
        return [f"lidar{i:04d}.laz" for i in range(self._info.num_laz)]

    def _csv_files(self) -> list[str]:
        return [f"imu{i:04d}.csv" for i in range(self._info.num_csv)]

    # ------------------------- For loading data ------------------------- #
    def data_iter(self) -> DatasetIterator:
        # Load in all IMU data
        imu = np.concatenate([_load_imu(self.folder / f) for f in self._csv_files()])
        imu = imu[np.argsort(imu[:, 0], kind="stable")]
        stamps = imu[:, 0].astype(np.int64)
        imu = imu[np.concatenate([[True], np.diff(stamps) > 0])]

        def imu_iter() -> Iterator[ImuMeasurement]:
            for row in imu:
                yield ImuMeasurement(
                    Stamp.from_nsec(int(row[0])), row[1:4], row[4:7] * _G
                )

        laz_files = [self.folder / f for f in self._laz_files()]

        def lidar_iter() -> Iterator[LidarMeasurement]:
            def scan(
                start: int, t: np.ndarray, xyz: np.ndarray, intensity: np.ndarray
            ) -> LidarMeasurement:
                stamp = Stamp.from_nsec(start)
                xyz_l: list[list[float]] = xyz.tolist()
                intensity_l: list[float] = intensity.tolist()
                t_l: list[int] = t.tolist()
                points = [
                    Point(
                        x=p[0],
                        y=p[1],
                        z=p[2],
                        intensity=i,
                        t=Duration.from_nsec(int(s - start)),
                    )
                    for p, i, s in zip(xyz_l, intensity_l, t_l)
                ]
                return LidarMeasurement(stamp, points)

            start: Optional[int] = None
            # Points of the last unfinished scan, carried over into the next chunk
            rest = (np.zeros(0, np.int64), np.zeros((0, 3)), np.zeros(0))
            for file in laz_files:
                t, xyz, intensity = _load_laz(file)
                t = np.concatenate([rest[0], t])
                xyz = np.concatenate([rest[1], xyz])
                intensity = np.concatenate([rest[2], intensity])
                if t.size == 0:
                    continue
                if start is None:
                    start = int(t[0])

                edges = np.arange(start, int(t[-1]) + 1, _SCAN_PERIOD_NS)
                idx = np.searchsorted(t, edges)
                for k in range(len(edges) - 1):
                    a, b = idx[k], idx[k + 1]
                    if b > a:
                        yield scan(int(edges[k]), t[a:b], xyz[a:b], intensity[a:b])
                start = int(edges[-1])
                a = idx[-1]
                rest = (t[a:], xyz[a:], intensity[a:])

            if start is not None and rest[0].size > 0:
                yield scan(start, *rest)

        return RawDataIter(
            lidar_iter(),
            imu_iter(),
            self.quick_len() or 0,
        )

    def ground_truth_raw(self) -> Optional[Trajectory]:
        if self._info.gt_file is None:
            return None
        return Trajectory.from_csv(
            self.folder / Path(self._info.gt_file).name,
            ["sec", "x", "y", "z", "qx", "qy", "qz", "qw"],
            delimiter=" ",
        )

    # ------------------------- For loading params ------------------------- #
    def imu_T_lidar(self) -> SE3:
        # IMU position in the Livox Mid-360 point cloud frame, from the Livox spec sheet
        return SE3(SO3.identity(), np.array([-0.011, -0.02329, 0.04412]))

    def imu_T_gt(self) -> SE3:
        # Ground truth is computed by HDMapping, which uses the lidar frame
        return self.imu_T_lidar()

    def imu_params(self) -> ImuParams:
        # ICM40609 inside the Mid-360, noise values are approximate
        return ImuParams(
            gyro=1.745e-3,
            accel=1.5e-2,
            gyro_bias=1e-5,
            accel_bias=1e-4,
            bias_init=1e-8,
            integration=1e-8,
            gravity=np.array([0, 0, -_G]),
            rate=200.0,
            brand="TDK",
            model="ICM40609",
        )

    def lidar_params(self) -> LidarParams:
        return LidarParams(
            num_rows=4,
            num_columns=5000,
            min_range=0.1,
            max_range=70.0,
            rate=10.0,
            brand="Livox",
            model="Mid-360",
        )

    # ------------------------- dataset info ------------------------- #
    @staticmethod
    def url() -> str:
        return "https://huggingface.co/MichalPelka"

    def environment(self) -> str:
        return self._info.environment

    def vehicle(self) -> str:
        return "Handheld"

    def quick_len(self) -> Optional[int]:
        return self._info.quick_len

    # ------------------------- For downloading ------------------------- #
    def files(self) -> Sequence[str | Path]:
        out = self._laz_files() + self._csv_files()
        if self._info.gt_file is not None:
            out.append(Path(self._info.gt_file).name)
        return out

    def download(self):
        info = self._info
        base = f"https://huggingface.co/datasets/{info.hf_repo}/resolve/main/"
        sub = f"{info.subdir}/" if info.subdir else ""
        remote = [sub + f for f in self._laz_files() + self._csv_files()]
        if info.gt_file is not None:
            remote.append(info.gt_file)

        print(f"Downloading to {self.folder}...")
        self.folder.mkdir(parents=True, exist_ok=True)
        for r in remote:
            path = self.folder / Path(r).name
            if path.exists():
                continue
            print(f"Downloading {r}...")
            tmp = path.with_suffix(path.suffix + ".part")
            _urlretrieve(base + r, tmp)
            tmp.rename(path)

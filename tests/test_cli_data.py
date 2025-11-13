from enum import auto
from pathlib import Path
from typing import Sequence
from evalio.cli import app
from evalio import datasets as ds, types as ty

import pytest


class FakeData(ds.Dataset):
    not_downloaded = auto()
    downloaded = auto()

    def is_downloaded(self) -> bool:
        if self == FakeData.not_downloaded:
            return False
        else:
            return True

    def files(self) -> Sequence[str | Path]:
        return []

    def download(self) -> None:
        return

    # To finish impl
    def data_iter(self) -> ds.DatasetIterator:
        return ds.RawDataIter(iter([]), iter([]), 0)

    def imu_T_lidar(self) -> ty.SE3:
        return ty.SE3.identity()

    def imu_T_gt(self) -> ty.SE3:
        return ty.SE3.identity()

    def imu_params(self) -> ty.ImuParams:
        return ty.ImuParams()

    def lidar_params(self) -> ty.LidarParams:
        return ty.LidarParams(
            num_rows=16, num_columns=64, min_range=0.0, max_range=100.0
        )

    def ground_truth_raw(self) -> ty.Trajectory:
        return ty.Trajectory()

    @staticmethod
    def url() -> str:
        return "fake"


ds.register_dataset(FakeData)
app.result_action = "return_value"


def test_dl_done(capsys: pytest.CaptureFixture[str]) -> None:
    app.meta(["dl", "fake_data/downloaded"])

    captured = capsys.readouterr()
    expected = """
Skipping download for fake_data/downloaded, already exists
Nothing to download, finishing
    """
    assert captured.out.strip() == expected.strip()


def test_dl_not_done(capsys: pytest.CaptureFixture[str]) -> None:
    app.meta(["dl", "fake_data/not_downloaded"])

    captured = capsys.readouterr()
    expected = """
Will download:
  fake_data/not_downloaded

---------- Beginning fake_data/not_downloaded ----------
---------- Finished fake_data/not_downloaded ----------
"""
    assert captured.out.strip() == expected.strip()


def test_rm_done(capsys: pytest.CaptureFixture[str]) -> None:
    app.meta(["rm", "fake_data/downloaded", "-y"])

    captured = capsys.readouterr()
    expected = f"""
Will remove:
  fake_data/downloaded

---------- Beginning fake_data/downloaded ----------
Removing from {ds.get_data_dir()}/fake_data/downloaded
---------- Finished fake_data/downloaded ----------
"""
    assert captured.out.strip() == expected.strip()


def test_rm_not_done(capsys: pytest.CaptureFixture[str]) -> None:
    app.meta(["rm", "fake_data/not_downloaded", "-y"])

    captured = capsys.readouterr()
    expected = f"""
Will remove:
  fake_data/not_downloaded

---------- Beginning fake_data/not_downloaded ----------
Removing from {ds.get_data_dir()}/fake_data/not_downloaded
---------- Finished fake_data/not_downloaded ----------
"""
    assert captured.out.strip() == expected.strip()

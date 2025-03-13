from evalio.types import ImuMeasurement, LidarMeasurement, Stamp
from evalio.datasets.iterators import RawDataIter
import numpy as np


def make_imus(num: int = 200) -> list[ImuMeasurement]:
    imus = []
    for i in range(num):
        imus.append(
            ImuMeasurement(
                Stamp.from_sec((i + 1e-6) / num),
                np.random.rand(3),
                np.random.rand(3),
            )
        )

    return imus


def make_lidars(num: int = 10) -> list[LidarMeasurement]:
    lidars = []
    for i in range(num):
        # offset a smidge from the imus
        lidars.append(LidarMeasurement(Stamp.from_sec((i + 1e-2) / num)))

    return lidars


# ------------------------- Make sure imu/lidars are in order and got all of them ------------------------- #
def test_iteration():
    imus = make_imus()
    lidars = make_lidars()
    iterator = RawDataIter(iter(lidars), iter(imus))

    last_stamp = Stamp.from_sec(0.0)

    count = 0
    exp_count = len(imus) + len(lidars)
    for m in iterator:
        count += 1
        assert last_stamp < m.stamp

    assert count == exp_count

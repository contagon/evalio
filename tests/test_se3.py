from dataclasses import dataclass
from evalio.types import SE3, SO3, Point, LidarMeasurement, Stamp
from scipy.spatial.transform import Rotation as R
import numpy as np
import math


@dataclass
class T:
    r: R
    t: np.ndarray


def is_close(t: T, se3: SE3):
    qx, qy, qz, qw = t.r.as_quat()
    math.isclose(se3.rot.qx, qx)
    math.isclose(se3.rot.qy, qy)
    math.isclose(se3.rot.qz, qz)
    math.isclose(se3.rot.qw, qw)


def test_constructor():
    t = np.array([1, 2, 3])
    r = R.from_rotvec([1.0, 0.2, 0.3])

    qx, qy, qz, qw = r.as_quat()
    so3 = SO3(qx=qx, qy=qy, qz=qz, qw=qw)  # type: ignore

    is_close(T(r, t), SE3(so3, t))


def test_from_matrix():
    t = np.array([1, 2, 3])
    r = R.from_rotvec([1.0, 0.2, 0.3])

    mat = np.eye(4)
    mat[:3, :3] = r.as_matrix()
    mat[:3, 3] = t

    se3 = SE3.fromMat(mat)

    is_close(T(r, t), se3)


def test_to_matrix():
    t = np.array([1, 2, 3])
    r = R.from_rotvec([1.0, 0.2, 0.3])

    qx, qy, qz, qw = r.as_quat()
    so3 = SO3(qx=qx, qy=qy, qz=qz, qw=qw)  # type: ignore
    se3 = SE3(so3, t)

    mat = np.eye(4)
    mat[:3, :3] = r.as_matrix()
    mat[:3, 3] = t

    assert np.allclose(mat, se3.toMat())


def test_transform_point():
    p = Point(x=1, y=2, z=3)

    t = np.array([1, 2, 3])
    r = R.from_rotvec([1.0, 0.2, 0.3])

    qx, qy, qz, qw = r.as_quat()
    so3 = SO3(qx=qx, qy=qy, qz=qz, qw=qw)  # type: ignore
    se3 = SE3(so3, t)

    as_np = np.array([p.x, p.y, p.z])
    p_expected = r.apply(as_np) + t
    se3.transform_in_place(p)

    assert math.isclose(p_expected[0], p.x)
    assert math.isclose(p_expected[1], p.y)
    assert math.isclose(p_expected[2], p.z)


def test_transform_scan():
    scan = LidarMeasurement(Stamp.from_sec(0.0), [Point(x=1, y=2, z=3)])

    t = np.array([1, 2, 3])
    r = R.from_rotvec([1.0, 0.2, 0.3])

    qx, qy, qz, qw = r.as_quat()
    so3 = SO3(qx=qx, qy=qy, qz=qz, qw=qw)  # type: ignore
    se3 = SE3(so3, t)

    as_np = np.array([scan.points[0].x, scan.points[0].y, scan.points[0].z])
    p_expected = r.apply(as_np) + t
    se3.transform_in_place(scan)

    assert math.isclose(p_expected[0], scan.points[0].x)
    assert math.isclose(p_expected[1], scan.points[0].y)
    assert math.isclose(p_expected[2], scan.points[0].z)

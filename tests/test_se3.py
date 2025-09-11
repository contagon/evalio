from dataclasses import dataclass
from evalio.types import SE3, SO3
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


def test_log_exp():
    xi = np.array([0.1, 0.2, 0.3, 4.0, 5.0, 6.0])
    se3 = SE3.exp(xi)
    xi_log = se3.log()
    assert np.allclose(xi, xi_log)

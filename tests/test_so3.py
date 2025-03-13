from evalio.types import SO3
from scipy.spatial.transform import Rotation as R
import numpy as np
import math


def is_close(r: R, so3: SO3):
    qx, qy, qz, qw = r.as_quat()
    math.isclose(so3.qx, qx)
    math.isclose(so3.qy, qy)
    math.isclose(so3.qz, qz)
    math.isclose(so3.qw, qw)


def test_constructor():
    r = R.from_rotvec([0.1, 0.2, 0.3])
    qx, qy, qz, qw = r.as_quat()
    so3 = SO3(qx=qx, qy=qy, qz=qz, qw=qw)
    is_close(r, so3)


def test_from_matrix():
    r = R.from_rotvec([0.1, 0.2, 0.3])
    so3 = SO3.fromMat(r.as_matrix())
    is_close(r, so3)


def test_to_matrix():
    r = R.from_rotvec([0.1, 0.2, 0.3])
    qx, qy, qz, qw = r.as_quat()
    so3 = SO3(qx=qx, qy=qy, qz=qz, qw=qw)
    assert np.allclose(so3.toMat(), r.as_matrix())


def test_exp():
    xi = np.array([0.1, 0.2, 0.3])
    r = R.from_rotvec(xi)
    xo3 = SO3.exp(xi)
    is_close(r, xo3)


def test_log():
    r = R.from_rotvec([0.1, 0.2, 0.3])
    qx, qy, qz, qw = r.as_quat()
    so3 = SO3(qx=qx, qy=qy, qz=qz, qw=qw)
    assert np.allclose(so3.log(), r.as_rotvec())


def test_inverse():
    r = R.from_rotvec([0.1, 0.2, 0.3])
    qx, qy, qz, qw = r.as_quat()
    so3 = SO3(qx=qx, qy=qy, qz=qz, qw=qw)
    is_close(r.inv(), so3.inverse())

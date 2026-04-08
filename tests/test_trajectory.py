import pytest
import numpy as np

from evalio.types.base import Trajectory
from evalio._cpp.types import SE3, SO3, Stamp  # type: ignore


@pytest.fixture
def square_trajectory():
    """Returns a trajectory moving in a square shape in the XY plane."""
    t = Trajectory()
    # Path: (0,0,0) -> (1,0,0) -> (1,1,0) -> (0,1,0)
    t.append(Stamp(sec=0, nsec=0), SE3(SO3.identity(), np.array([0.0, 0.0, 0.0])))
    t.append(Stamp(sec=1, nsec=0), SE3(SO3.identity(), np.array([1.0, 0.0, 0.0])))
    t.append(Stamp(sec=2, nsec=0), SE3(SO3.identity(), np.array([1.0, 1.0, 0.0])))
    t.append(Stamp(sec=3, nsec=0), SE3(SO3.identity(), np.array([0.0, 1.0, 0.0])))
    return t


def test_smooth_orientation_x_up(square_trajectory: Trajectory):
    t = square_trajectory.smooth_orientation(forward="x", z="up", inplace=False)

    # Pose 0: heading (1,0,0). X should point to (1,0,0), Z to (0,0,1), Y to (0,1,0)
    mat0 = t.poses[0].rot.to_mat()
    np.testing.assert_allclose(mat0[:, 0], [1.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(mat0[:, 1], [0.0, 1.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(mat0[:, 2], [0.0, 0.0, 1.0], atol=1e-6)

    # Pose 1: heading (0,1,0). X should point to (0,1,0), Z to (0,0,1), Y to (-1,0,0)
    mat1 = t.poses[1].rot.to_mat()
    np.testing.assert_allclose(mat1[:, 0], [0.0, 1.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(mat1[:, 1], [-1.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(mat1[:, 2], [0.0, 0.0, 1.0], atol=1e-6)

    # Pose 2: heading (-1,0,0). X should point to (-1,0,0), Z to (0,0,1), Y to (0,-1,0)
    mat2 = t.poses[2].rot.to_mat()
    np.testing.assert_allclose(mat2[:, 0], [-1.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(mat2[:, 1], [0.0, -1.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(mat2[:, 2], [0.0, 0.0, 1.0], atol=1e-6)

    # Pose 3: Last pose should copy the previous rotation
    mat3 = t.poses[3].rot.to_mat()
    np.testing.assert_allclose(mat3, mat2, atol=1e-6)


def test_smooth_orientation_y_up(square_trajectory: Trajectory):
    t = square_trajectory.smooth_orientation(forward="y", z="up", inplace=False)

    # Pose 0: heading (1,0,0). Y should point to (1,0,0), Z to (0,0,1), X to (0,-1,0)
    mat0 = t.poses[0].rot.to_mat()
    np.testing.assert_allclose(mat0[:, 0], [0.0, -1.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(mat0[:, 1], [1.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(mat0[:, 2], [0.0, 0.0, 1.0], atol=1e-6)

    # Pose 1: heading (0,1,0). Y should point to (0,1,0), Z to (0,0,1), X to (1,0,0)
    mat1 = t.poses[1].rot.to_mat()
    np.testing.assert_allclose(mat1[:, 0], [1.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(mat1[:, 1], [0.0, 1.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(mat1[:, 2], [0.0, 0.0, 1.0], atol=1e-6)


def test_smooth_orientation_x_down(square_trajectory: Trajectory):
    t = square_trajectory.smooth_orientation(forward="x", z="down", inplace=False)

    # Pose 0: heading (1,0,0). X should point to (1,0,0), Z to (0,0,-1), Y to (0,-1,0)
    mat0 = t.poses[0].rot.to_mat()
    np.testing.assert_allclose(mat0[:, 0], [1.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(mat0[:, 1], [0.0, -1.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(mat0[:, 2], [0.0, 0.0, -1.0], atol=1e-6)

    # Pose 1: heading (0,1,0). X should point to (0,1,0), Z to (0,0,-1), Y to (1,0,0)
    mat1 = t.poses[1].rot.to_mat()
    np.testing.assert_allclose(mat1[:, 0], [0.0, 1.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(mat1[:, 1], [1.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(mat1[:, 2], [0.0, 0.0, -1.0], atol=1e-6)


def test_smooth_orientation_y_down(square_trajectory: Trajectory):
    t = square_trajectory.smooth_orientation(forward="y", z="down", inplace=False)

    # Pose 0: heading (1,0,0). Y should point to (1,0,0), Z to (0,0,-1), X to (0,1,0)
    mat0 = t.poses[0].rot.to_mat()
    np.testing.assert_allclose(mat0[:, 0], [0.0, 1.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(mat0[:, 1], [1.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(mat0[:, 2], [0.0, 0.0, -1.0], atol=1e-6)


def test_smooth_orientation_min_dist(square_trajectory: Trajectory):
    """Test that min_dist is respected."""
    t = Trajectory()
    Id = SO3.identity()
    # Dense points that should be skipped by min_dist (default 1e-1)
    t.append(Stamp(sec=0, nsec=0), SE3(Id, np.array([0.0, 0.0, 0.0])))
    t.append(Stamp(sec=1, nsec=0), SE3(Id, np.array([0.05, 0.0, 0.0])))  # Skipped
    t.append(Stamp(sec=2, nsec=0), SE3(Id, np.array([0.14, 0.0, 0.0])))  # Used
    t.append(Stamp(sec=3, nsec=0), SE3(Id, np.array([0.15, 0.15, 0.0])))  # Used

    t_smooth = t.smooth_orientation(forward="x", z="up", inplace=False)

    # For pose 0, next valid pose is 2 (dist = 0.15 > 0.1)
    # So heading is (1,0,0)
    mat0 = t_smooth.poses[0].rot.to_mat()
    np.testing.assert_allclose(mat0[:, 0], [1.0, 0.0, 0.0], atol=1e-6)

    # For pose 1, next valid pose is 3 (dist from 1 to 3 is ~0.18 > 0.1)
    # dx=0.1, dy=0.15 -> vector is (0.1, 0.15, 0)
    v13 = np.array([0.1, 0.15, 0.0])
    v13 /= np.linalg.norm(v13)
    mat1 = t_smooth.poses[1].rot.to_mat()
    np.testing.assert_allclose(mat1[:, 0], v13, atol=1e-6)

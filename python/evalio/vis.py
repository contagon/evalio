import numpy as np
import rerun as rr

from evalio.types import SE3, LidarMeasurement, Point


def rerun(obj: LidarMeasurement | SE3 | list[Point], **kwargs):
    if isinstance(obj, LidarMeasurement):
        return lidar_to_rerun(obj, **kwargs)
    elif isinstance(obj, SE3):
        return pose_to_rerun(obj, **kwargs)
    elif isinstance(obj, list) and isinstance(obj[0], Point):
        return map_to_rerun(obj, **kwargs)
    else:
        raise ValueError(f"Cannot convert {obj} to rerun")


def map_to_rerun(map: list[Point], use_intensity=False, color=None):
    """Convert a LidarMeasurement to a string for rerun."""
    # Parse options
    if use_intensity and color is not None:
        raise ValueError("Cannot use both intensity and color at the same time")

    # Convert to numpy
    size = len(map)
    points = np.zeros((size, 3))

    for i, point in enumerate(map):
        points[i] = [point.x, point.y, point.z]

    # parse color
    if color is not None:
        colors = np.tile(color, (size, 1))
    elif use_intensity:
        colors = np.zeros((size, 3))
        for i, point in enumerate(map):
            val = point.intensity / 255
            colors[i] = [1.0 - val, val, 0]
    else:
        colors = None

    return rr.Points3D(points, colors=colors)


def lidar_to_rerun(
    lidarscan: LidarMeasurement, use_intensity=False, color=None
) -> rr.Points3D:
    """Convert a LidarMeasurement to a string for rerun."""
    return map_to_rerun(lidarscan.points, use_intensity=use_intensity, color=color)


def pose_to_rerun(pose: SE3) -> rr.Transform3D:
    """Convert a Pose to a Transform3D for rerun."""
    return rr.Transform3D(
        rotation=rr.datatypes.Quaternion(
            xyzw=[
                pose.rot.qx,
                pose.rot.qy,
                pose.rot.qz,
                pose.rot.qw,
            ]
        ),
        translation=pose.trans,
    )


def poses_to_points(poses: list[SE3], color=None) -> rr.Points3D:
    points = np.zeros((len(poses), 3))
    for i, pose in enumerate(poses):
        points[i] = pose.trans

    return rr.Points3D(points, colors=color)

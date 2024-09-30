from ._cpp import LidarMeasurement  # type: ignore
import numpy as np

import rerun as rr


def lidarscan_to_rerun(
    lidarscan: LidarMeasurement, use_intensity=False, color=None
) -> str:
    """Convert a LidarMeasurement to a string for rerun."""
    # Parse options
    if use_intensity and color is not None:
        raise ValueError("Cannot use both intensity and color at the same time")

    # Convert to numpy
    size = len(lidarscan.points)
    points = np.zeros((size, 3))

    for i, point in enumerate(lidarscan.points):
        points[i] = [point.x, point.y, point.z]

    # parse color
    if color is not None:
        colors = np.tile(color, (size, 1))
    elif use_intensity:
        colors = np.zeros((size, 3))
        for i, point in enumerate(lidarscan.points):
            val = point.intensity / 255
            colors[i] = [1.0 - val, val, 0]
    else:
        colors = None

    return rr.Points3D(points, colors=colors)

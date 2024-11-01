from dataclasses import dataclass
from enum import Enum
from typing import Optional, Sequence
from uuid import uuid4

from evalio.types import LidarParams, Trajectory
from evalio.datasets import Dataset
import numpy as np


from evalio.types import SE3, LidarMeasurement, Point

try:
    import rerun as rr
    import rerun.blueprint as rrb
except ImportError:
    pass

OverrideType = dict[rr.datatypes.EntityPath | str, list[rr.ComponentBatchLike]]


# TODO: Handle multiple trajectories runs in single recording
# TODO: Add previous part of trajectory as points
class RerunVis:
    def __init__(self, level: int, config: "RerunConfig"):
        self.level = level
        self.config = config
        overrides: OverrideType = {"imu/lidar": [rrb.components.Visible(False)]}
        self.blueprint: rr.BlueprintLike

        if self.level == 1:
            self.blueprint = rrb.Spatial3DView(overrides=overrides)
        elif self.level >= 2:
            self.blueprint = rrb.Blueprint(
                rrb.Vertical(
                    rrb.Spatial2DView(),  # image
                    rrb.BarChartView(),  # bar chart
                    # TODO: Error as well?
                    rrb.Spatial3DView(  # 3d view
                        overrides=overrides,
                        background=rrb.BackgroundKind.GradientBright,
                    ),
                    row_shares=[1, 1, 3],
                ),
                collapse_panels=True,
            )

        # To be set during new_recording
        self.lidar_params: Optional[LidarParams] = None
        self.gt: Optional[Trajectory] = None

        # To be found during log
        self.gt_o_T_imu_o: Optional[SE3] = None

    def new_recording(self, dataset: Dataset):
        if self.level == 0:
            return

        rr.new_recording(
            str(dataset),
            make_default=True,
            recording_id=uuid4(),
        )
        rr.connect(
            f"{self.config.ip}:{self.config.port}", default_blueprint=self.blueprint
        )
        self.gt = dataset.ground_truth()
        self.lidar_params = dataset.lidar_params()
        self.gt_o_T_imu_o = None

        rr.log(
            "gt",
            convert(self.gt, Vis.Points, color=[0, 0, 255]),
            static=True,
        )
        rr.log(
            "imu/lidar",
            convert(dataset.imu_T_lidar(), Vis.Pose),
            static=True,
        )

    def log(self, data: LidarMeasurement, features: Sequence[Point], pose: SE3):
        if self.level == 0:
            return

        if self.lidar_params is None or self.gt is None:
            raise ValueError("You needed to initialize the recording before stepping!")

        # Find transform between ground truth and imu origins
        if self.gt_o_T_imu_o is None:
            if data.stamp < self.gt.stamps[0]:
                pass
            else:
                imu_o_T_imu_0 = pose
                gt_o_T_imu_0 = self.gt.poses[0]
                self.gt_o_T_imu_o = gt_o_T_imu_0 * imu_o_T_imu_0.inverse()

        # If level is 1, just include the pose
        if self.level >= 1:
            rr.set_time_seconds("evalio_time", seconds=data.stamp.to_sec())
            if self.gt_o_T_imu_o is not None:
                rr.log("imu", convert(self.gt_o_T_imu_o * pose, Vis.Pose))

        # If level is 2 or greater, include the image and histogram
        if self.level >= 2:
            intensity = np.array([d.intensity for d in data.points])
            # row major order
            if data.points[0].row == data.points[1].row:
                image = intensity.reshape(
                    (self.lidar_params.num_rows, self.lidar_params.num_columns)
                )
            # column major order
            else:
                image = intensity.reshape(
                    (self.lidar_params.num_columns, self.lidar_params.num_rows)
                ).T

            rr.log("image", rr.Image(image))

            feat_intensity = np.array([d.intensity for d in features])
            rr.log("features", convert(feat_intensity, Vis.Histogram, bins=100))

        # If level is 3 or greater, include the features from the scan
        if self.level >= 3:
            rr.log("imu/lidar/frame", convert(features, Vis.Points, use_intensity=True))


@dataclass
class RerunConfig:
    ip: str = "0.0.0.0"
    port: int = 9876
    spawn: bool = False


# ------------------------- For converting to rerun types ------------------------- #
class Vis(Enum):
    """Visualization options."""

    Points = 1
    Pose = 2
    Arrows = 3
    Histogram = 4


def convert(obj: object, kind: Vis, **kwargs):
    # Short circuit if there isn't anything
    if isinstance(obj, list) and len(obj) == 0:
        match kind:
            case Vis.Points:
                return rr.Points3D([])
            case Vis.Pose:
                raise ValueError("Cannot convert empty list to Pose")
            case Vis.Arrows:
                return rr.Arrows3D(vectors=[])
            case Vis.Histogram:
                return rr.BarChart([])

    if isinstance(obj, LidarMeasurement):
        match kind:
            case Vis.Points:
                return lidar_to_rerun(obj, **kwargs)
            case _:
                raise ValueError(f"Cannot convert LidarMeasurement to {kind}")

    elif isinstance(obj, SE3):
        match kind:
            case Vis.Pose:
                return pose_to_rerun(obj, **kwargs)
            case _:
                raise ValueError(f"Cannot convert SE3 to {kind}")

    elif isinstance(obj, list) and isinstance(obj[0], SE3):
        match kind:
            case Vis.Points:
                return poses_to_points(obj, **kwargs)
            case _:
                raise ValueError(f"Cannot convert list of SE3 to {kind}")

    elif isinstance(obj, Trajectory):
        match kind:
            case Vis.Points:
                return poses_to_points(obj.poses, **kwargs)
            case _:
                raise ValueError(f"Cannot convert Trajectory to {kind}")

    elif isinstance(obj, list) and isinstance(obj[0], Point):
        match kind:
            case Vis.Points:
                return map_to_rerun(obj, **kwargs)
            case _:
                raise ValueError(f"Cannot convert list of Point to {kind}")

    elif isinstance(obj, np.ndarray):
        match kind:
            case Vis.Histogram:
                return array_to_histogram(obj, **kwargs)
            case _:
                raise ValueError(f"Cannot convert np.ndarray to {kind}")

    else:
        raise ValueError(f"Cannot convert {type(obj)} to {kind}")


# ------------------------- All the converters ------------------------- #
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


def array_to_histogram(array: np.ndarray, **kwargs) -> rr.BarChart:
    """Convert an array to a histogram."""
    heights, bins = np.histogram(array, **kwargs)
    return rr.BarChart(heights)

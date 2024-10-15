from dataclasses import dataclass
from enum import Enum
from typing import Optional
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
                    rrb.Spatial2DView(),
                    rrb.Spatial3DView(
                        overrides=overrides,
                        background=rrb.BackgroundKind.GradientBright,
                    ),
                    row_shares=[1, 3],
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

    def log(self, data: LidarMeasurement, pose: SE3):
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

        # If level is 2 or greater, include the image
        if self.level >= 2:
            # TODO: Need to handle row vs column major points here
            # image = (
            #     np.array([d.intensity for d in data.points])
            #     .reshape((self.lidar_params.num_columns, self.lidar_params.num_rows))
            #     .T
            # )
            image = np.array([d.intensity for d in data.points]).reshape(
                (self.lidar_params.num_rows, self.lidar_params.num_columns)
            )
            rr.log("image", rr.Image(image))

        # If level is 3 or greater, include the scan
        if self.level > 3:
            rr.log("imu/lidar/frame", convert(data, Vis.Points, use_intensity=True))


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


def convert(obj: object, kind: Vis, **kwargs):
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

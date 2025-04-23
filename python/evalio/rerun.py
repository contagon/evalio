from typing import Any, Optional, Sequence, overload
from uuid import uuid4

from evalio.types import LidarParams, Trajectory, Stamp
from evalio.datasets import Dataset
from evalio.pipelines import Pipeline
from evalio.utils import print_warning
import numpy as np


from evalio.types import SE3, LidarMeasurement, Point

try:
    import rerun as rr
    import rerun.blueprint as rrb

    OverrideType = dict[rr.datatypes.EntityPath | str, list[rr.ComponentBatchLike]]

    # TODO: Handle multiple trajectories runs in single recording
    # TODO: Add previous part of trajectory as points
    class RerunVis:  # type: ignore
        def __init__(self, level: int):
            self.level = level
            overrides: OverrideType = {"imu/lidar": [rrb.components.Visible(False)]}
            self.blueprint: rr.BlueprintLike

            # 3D view only
            print("Rerun visualization enabled")
            if self.level < 4:
                self.blueprint = rrb.Blueprint(
                    rrb.Spatial3DView(overrides=overrides),
                    collapse_panels=True,
                )
            # include intensity image as well
            else:
                self.blueprint = rrb.Blueprint(
                    rrb.Vertical(
                        rrb.Spatial2DView(),  # image
                        # TODO: Error as well?
                        rrb.Spatial3DView(  # 3d view
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
            rr.connect_tcp("0.0.0.0:9876", default_blueprint=self.blueprint)
            self.gt = dataset.ground_truth()
            self.lidar_params = dataset.lidar_params()
            self.gt_o_T_imu_o = None

            rr.log("gt", convert(self.gt), static=True)
            rr.log("gt", rr.Points3D.from_fields(colors=[0, 0, 255]))
            rr.log("origin/imu/lidar", convert(dataset.imu_T_lidar()), static=True)

        def log(
            self,
            data: LidarMeasurement,
            features: Sequence[Point],
            pose: SE3,
            pipe: Pipeline,
        ):
            if self.level == 0:
                return

            if self.lidar_params is None or self.gt is None:
                raise ValueError(
                    "You needed to initialize the recording before stepping!"
                )

            # Find transform between ground truth and imu origins
            if self.gt_o_T_imu_o is None:
                if data.stamp < self.gt.stamps[0]:
                    pass
                else:
                    imu_o_T_imu_0 = pose
                    gt_o_T_imu_0 = self.gt.poses[0]
                    self.gt_o_T_imu_o = gt_o_T_imu_0 * imu_o_T_imu_0.inverse()
                    rr.log("origin", convert(self.gt_o_T_imu_o), static=True)

            # If level is 1, just include the pose
            if self.level >= 1:
                rr.set_time_seconds("evalio_time", seconds=data.stamp.to_sec())
                rr.log("origin/imu", convert(pose))

            # If level is 2 or greater, include the features from the scan
            if self.level >= 2:
                if len(features) > 0:
                    rr.log("origin/imu/lidar/features", convert(list(features)))

            # If level is 3 or greater, include the current map
            if self.level >= 3:
                rr.log("origin/map", convert(pipe.map()))

            # If level is 3 or greater, include the image and original point cloud
            if self.level >= 4:
                intensity = np.array([d.intensity for d in data.points])
                # row major order
                image = intensity.reshape(
                    (self.lidar_params.num_rows, self.lidar_params.num_columns)
                )
                rr.log("image", rr.Image(image))
                rr.log("origin/imu/lidar/scan", convert(data))

    # ------------------------- For converting to rerun types ------------------------- #
    # point clouds
    @overload
    def convert(
        obj: LidarMeasurement, color: Optional[str | list[int]] = None
    ) -> rr.Points3D: ...
    @overload
    def convert(
        obj: list[Point], color: Optional[str | list[int]] = None
    ) -> rr.Points3D: ...
    @overload
    def convert(obj: np.ndarray, color: Optional[np.ndarray] = None) -> rr.Points3D: ...

    # trajectories
    @overload
    def convert(obj: list[SE3], color: Optional[list[int]] = None) -> rr.Points3D: ...
    @overload
    def convert(obj: Trajectory, color: Optional[list[int]] = None) -> rr.Points3D: ...

    # poses
    @overload
    def convert(obj: SE3) -> rr.Transform3D: ...

    def convert(
        obj: object, color: Optional[Any] = None
    ) -> rr.Transform3D | rr.Points3D:
        # Handle point clouds
        if isinstance(obj, LidarMeasurement):
            color_parsed = None
            if color == "intensity":
                max_intensity = max([p.intensity for p in obj.points])
                color_parsed = np.zeros((len(obj.points), 3))
                for i, point in enumerate(obj.points):
                    val = point.intensity / max_intensity
                    color_parsed[i] = [1.0 - val, val, 0]
            elif color == "z":
                zs = [p.z for p in obj.points]
                min_z, max_z = min(zs), max(zs)
                color_parsed = np.zeros((len(obj.points), 3))
                for i, point in enumerate(obj.points):
                    val = (point.z - min_z) / (max_z - min_z)
                    color_parsed[i] = [1.0 - val, val, 0]
            elif isinstance(color, list):
                color_parsed = np.asarray(color)
            elif color is not None:
                raise ValueError(f"Unknown color type {color}")
            return convert(np.asarray(obj.to_vec_positions()), color_parsed)

        elif isinstance(obj, list) and isinstance(obj[0], Point):
            return convert(LidarMeasurement(Stamp.from_sec(0), obj), color)

        elif isinstance(obj, np.ndarray) and len(obj.shape) == 2 and obj.shape[1] == 3:
            return rr.Points3D(obj, colors=color)

        # Handle poses
        elif isinstance(obj, SE3):
            return rr.Transform3D(
                rotation=rr.datatypes.Quaternion(
                    xyzw=[
                        obj.rot.qx,
                        obj.rot.qy,
                        obj.rot.qz,
                        obj.rot.qw,
                    ]
                ),
                translation=obj.trans,
            )
        elif isinstance(obj, Trajectory):
            return convert(obj.poses)
        elif isinstance(obj, list) and isinstance(obj[0], SE3):
            points = np.zeros((len(obj), 3))
            for i, pose in enumerate(obj):
                points[i] = pose.trans
            return rr.Points3D(points, colors=color)

        else:
            raise ValueError(f"Cannot convert {type(obj)} to rerun type")

except Exception as _:

    class RerunVis:
        def __init__(self, level: int) -> None:
            if level != 0:
                print_warning("Rerun not found, visualization disabled")

        def new_recording(self, dataset: Dataset):
            pass

        def log(self, data: LidarMeasurement, features: Sequence[Point], pose: SE3):
            pass

from typing import Any, Optional, Sequence, overload
from uuid import uuid4

from evalio.types import LidarParams, Trajectory, Stamp
from evalio.datasets import Dataset
from evalio.pipelines import Pipeline
from evalio.utils import print_warning
import numpy as np

from dataclasses import dataclass

import typer

from evalio.types import SE3, LidarMeasurement, Point


@dataclass
class VisArgs:
    show: bool
    map: bool = False
    image: bool = False
    scan: bool = False
    features: bool = False

    @staticmethod
    def parse(opts: str) -> "VisArgs":
        out = VisArgs(show=True)
        for o in opts:
            match o:
                case "m":
                    out.map = True
                case "i":
                    out.image = True
                case "s":
                    out.scan = True
                case "f":
                    out.features = True
                case _:
                    raise typer.BadParameter(f"Unknown visualization option {o}")

        return out


try:
    import rerun as rr
    import rerun.blueprint as rrb

    OverrideType = dict[rr.datatypes.EntityPath | str, list[rr.ComponentBatchLike]]

    class RerunVis:  # type: ignore
        def __init__(self, args: VisArgs):
            self.args = args
            self.blueprint: rr.BlueprintLike

            # include intensity image as well
            if args.image:
                self.blueprint = rrb.Blueprint(
                    rrb.Vertical(
                        rrb.Spatial2DView(),  # image
                        rrb.Spatial3DView(),
                        row_shares=[1, 3],
                    ),
                )
            else:
                self.blueprint = rrb.Blueprint(rrb.Spatial3DView())

            # To be set during new_recording
            self.lidar_params: Optional[LidarParams] = None
            self.gt: Optional[Trajectory] = None

            # To be found during log
            self.gt_o_T_imu_o: Optional[SE3] = None
            self.trajectory: Optional[Trajectory] = None
            self.imu_T_lidar: Optional[SE3] = None
            self.pn: Optional[str] = None

        def new_recording(self, dataset: Dataset):
            if not self.args.show:
                return

            rr.new_recording(
                str(dataset),
                make_default=True,
                recording_id=uuid4(),
            )
            rr.connect_tcp("0.0.0.0:9876", default_blueprint=self.blueprint)
            self.gt = dataset.ground_truth()
            self.lidar_params = dataset.lidar_params()
            self.imu_T_lidar = dataset.imu_T_lidar()

            rr.log("gt", convert(self.gt), static=True)
            rr.log("gt", rr.Points3D.from_fields(colors=[0, 0, 255]), static=True)

        def new_pipe(self, pipe_name: str):
            if not self.args.show:
                return

            if self.imu_T_lidar is None:
                raise ValueError(
                    "You needed to initialize the recording before adding a pipeline!"
                )

            self.pn = pipe_name
            self.gt_o_T_imu_o = None
            self.trajectory = Trajectory(stamps=[], poses=[])
            rr.log(f"{self.pn}/imu/lidar", convert(self.imu_T_lidar), static=True)

        def log(
            self,
            data: LidarMeasurement,
            features: Sequence[Point],
            pose: SE3,
            pipe: Pipeline,
        ):
            if not self.args.show:
                return

            if self.lidar_params is None or self.gt is None:
                raise ValueError(
                    "You needed to initialize the recording before stepping!"
                )
            if self.pn is None or self.trajectory is None:
                raise ValueError("You needed to add a pipeline before stepping!")

            # Find transform between ground truth and imu origins
            if self.gt_o_T_imu_o is None:
                if data.stamp < self.gt.stamps[0]:
                    pass
                else:
                    imu_o_T_imu_0 = pose
                    gt_o_T_imu_0 = self.gt.poses[0]
                    self.gt_o_T_imu_o = gt_o_T_imu_0 * imu_o_T_imu_0.inverse()
                    rr.log(self.pn, convert(self.gt_o_T_imu_o), static=True)

            # Always include the pose
            rr.set_time_seconds("evalio_time", seconds=data.stamp.to_sec())
            rr.log(f"{self.pn}/imu", convert(pose))
            self.trajectory.append(data.stamp, pose)
            rr.log(f"{self.pn}/trajectory", convert(self.trajectory))

            # Features from the scan
            if self.args.features:
                if len(features) > 0:
                    rr.log(f"{self.pn}/imu/lidar/features", convert(list(features)))

            # Include the current map
            if self.args.map:
                rr.log(f"{self.pn}/map", convert(pipe.map()))

            # Include the original point cloud
            if self.args.scan:
                rr.log(f"{self.pn}/imu/lidar/scan", convert(data))

            # Include the intensity image
            if self.args.image:
                intensity = np.array([d.intensity for d in data.points])
                # row major order
                image = intensity.reshape(
                    (self.lidar_params.num_rows, self.lidar_params.num_columns)
                )
                rr.log("image", rr.Image(image))

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
        def __init__(self, args: VisArgs) -> None:
            if args.show:
                print_warning("Rerun not found, visualization disabled")

        def new_recording(self, dataset: Dataset):
            pass

        def log(
            self,
            data: LidarMeasurement,
            features: Sequence[Point],
            pose: SE3,
            pipe: Pipeline,
        ):
            pass

        def new_pipe(self, pipe_name: str):
            pass

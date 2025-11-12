from typing import Any, Literal, Optional, TypedDict, cast, overload
from typing_extensions import TypeVar
from uuid import UUID, uuid4

import distinctipy
import numpy as np
from numpy.typing import NDArray

from evalio.datasets import Dataset
from evalio.pipelines import Pipeline
from evalio.types import (
    SE3,
    GroundTruth,
    LidarMeasurement,
    LidarParams,
    Metadata,
    Point,
    Stamp,
    Trajectory,
)
from evalio.utils import print_warning
from evalio._cpp.helpers import closest  # type: ignore
from evalio._cpp.types import VisOption  # type: ignore


# These colors are pulled directly from the rerun skybox colors
# https://github.com/rerun-io/rerun/blob/main/crates/viewer/re_renderer/shader/generic_skybox.wgsl#L19
# We avoid them to make sure our colors are distinct from viewer colors
def skybox_dark_rgb(dir: NDArray[np.float64]) -> tuple[float, float, float]:
    rgb = dir * 0.5 + np.full(3, 0.5)
    rgb = np.full(3, 0.05) + 0.20 * rgb
    return (float(rgb[0]), float(rgb[1]), float(rgb[2]))


def skybox_light_rgb(dir: NDArray[np.float64]) -> tuple[float, float, float]:
    rgb = dir * 0.5 + np.full(3, 0.5)
    rgb = np.full(3, 0.7) + 0.20 * rgb
    return (float(rgb[0]), float(rgb[1]), float(rgb[2]))


GT_COLOR = (
    144.0 / 255.0,
    144.0 / 255.0,
    144.0 / 255.0,
)  # Color for ground truth in rerun


try:
    import rerun as rr
    import rerun.blueprint as rrb

    OverrideType = dict[rr.datatypes.EntityPathLike, list[rr.AsComponents]]
    RerunArgs = TypedDict(
        "RerunArgs", {"application_id": str, "recording_id": UUID, "make_default": bool}
    )

    class RerunVis:  # type: ignore
        def __init__(self, args: Optional[set[VisOption]], pipeline_names: list[str]):
            self.args = args
            self.pipeline_names = pipeline_names

            # To be set during new_dataset
            self.lidar_params: Optional[LidarParams] = None
            self.imu_T_lidar: Optional[SE3] = None
            self.gt: Optional[Trajectory[GroundTruth]] = None

            # To be set during new_pipe
            self.trajectory: Optional[Trajectory] = None
            self.pn: Optional[str] = None
            self.colors: Optional[list[tuple[float, float, float]]] = None

            # To be set during log
            self.gt_o_T_imu_o: Optional[SE3] = None

            directions = np.array(
                [
                    [1, 0, 0],
                    [-1, 0, 0],
                    [0, 1, 0],
                    [0, -1, 0],
                    [0, 0, 1],
                    [0, 0, -1],
                ]
            )
            self.used_colors: list[tuple[float, float, float]] = (
                [
                    GT_COLOR,
                    distinctipy.BLACK,
                    distinctipy.WHITE,
                ]
                + [skybox_dark_rgb(dir) for dir in directions]
                + [skybox_light_rgb(dir) for dir in directions]
            )

        def _blueprint(self) -> rrb.BlueprintLike:
            # Eventually we'll be able to glob these, but for now, just take in the names beforehand
            # https://github.com/rerun-io/rerun/issues/6673
            # Once this is closed, we'll be able to remove pipelines as a parameter here and in new_recording
            overrides: OverrideType = {
                f"{n}/imu": [rrb.VisualizerOverrides(rrb.visualizers.Transform3DArrows)]
                for n in self.pipeline_names
            }

            if self.args is not None and VisOption.IMAGE in self.args:
                return rrb.Blueprint(
                    rrb.Vertical(
                        rrb.Spatial2DView(),  # image
                        rrb.Spatial3DView(overrides=overrides),
                        row_shares=[1, 3],
                    ),
                )
            else:
                return rrb.Blueprint(rrb.Spatial3DView(overrides=overrides))

        def new_dataset(self, dataset: Dataset):
            if self.args is None:
                return

            self.recording_params: RerunArgs = {
                "application_id": str(dataset),
                "recording_id": uuid4(),
                "make_default": True,
            }
            self.rec = rr.RecordingStream(**self.recording_params)
            self.rec.connect_grpc()
            self.rec.send_blueprint(self._blueprint())

            self.gt = dataset.ground_truth()
            self.lidar_params = dataset.lidar_params()
            self.imu_T_lidar = dataset.imu_T_lidar()

            self.rec.log("gt", convert(self.gt, color=GT_COLOR), static=True)

            # reset other variables
            self.pn = None
            self.gt_o_T_imu_o = None
            self.trajectory = None
            self.colors = None

        def new_pipe(self, pipe_name: str, feat_num: int):
            if self.args is None:
                return

            if self.imu_T_lidar is None:
                raise ValueError("You needed to add a dataset before a pipeline!")

            # Define colors for this pipeline
            # features/map colors + trajectory + scan
            self.colors = distinctipy.get_colors(
                feat_num + 2,
                exclude_colors=self.used_colors,
                rng=0,
            )
            self.used_colors.extend(self.colors)

            # First reconnect to make sure we're connected (happens b/c of multithread passing)
            self.rec = rr.RecordingStream(**self.recording_params)
            self.rec.connect_grpc()

            self.pn = pipe_name
            self.trajectory = Trajectory(stamps=[], poses=[])
            self.rec.log(f"{self.pn}/imu/lidar", convert(self.imu_T_lidar), static=True)

            self.gt_o_T_imu_o = None

        def log_scan(self, data: LidarMeasurement):
            if self.args is None:
                return

            if self.lidar_params is None or self.gt is None:
                raise ValueError("You needed to add a dataset before stepping!")
            if self.pn is None or self.trajectory is None or self.colors is None:
                raise ValueError("You needed to add a pipeline before stepping!")

            self.rec.set_time("evalio_time", timestamp=data.stamp.to_sec())

            # Include the original point cloud
            if VisOption.SCAN in self.args:
                self.rec.log(
                    f"{self.pn}/imu/lidar/scan",
                    convert(data, color=self.colors[-2], radii=0.08),
                )

            # Include the intensity image
            if VisOption.IMAGE in self.args:
                intensity = np.array([d.intensity for d in data.points])
                # row major order
                image = intensity.reshape(
                    (self.lidar_params.num_rows, self.lidar_params.num_columns)
                )
                self.rec.log("image", rr.Image(image))

        def log_pose(self, stamp: Stamp, pose: SE3):
            if self.args is None:
                return

            if self.lidar_params is None or self.gt is None:
                raise ValueError("You needed to add a dataset before stepping!")
            if self.pn is None or self.trajectory is None or self.colors is None:
                raise ValueError("You needed to add a pipeline before stepping!")

            # Find transform between ground truth and imu origins
            if self.gt_o_T_imu_o is None:
                if stamp < self.gt.stamps[0]:
                    pass
                else:
                    imu_o_T_imu_0 = pose
                    # find the ground truth pose that is temporally closest to the imu pose
                    gt_index = 0
                    while self.gt.stamps[gt_index] < stamp:
                        gt_index += 1
                    if not closest(
                        stamp,
                        self.gt.stamps[gt_index - 1],
                        self.gt.stamps[gt_index],
                    ):
                        gt_index -= 1
                    gt_o_T_imu_0 = self.gt.poses[gt_index]
                    self.gt_o_T_imu_o = gt_o_T_imu_0 * imu_o_T_imu_0.inverse()
                    self.rec.log(self.pn, convert(self.gt_o_T_imu_o), static=True)

            # Always include the pose
            self.rec.set_time("evalio_time", timestamp=stamp.to_sec())
            self.rec.log(f"{self.pn}/imu", convert(pose))
            self.trajectory.append(stamp, pose)
            self.rec.log(
                f"{self.pn}/trajectory",
                convert(self.trajectory, color=self.colors[-1]),
            )

        def log_map(self, stamp: Stamp, map: dict[str, NDArray[np.float64]]):
            if self.args is None:
                return

            if self.lidar_params is None or self.gt is None:
                raise ValueError("You needed to add a pipeline before stepping!")
            if self.pn is None or self.trajectory is None or self.colors is None:
                raise ValueError("You needed to add a pipeline before stepping!")

            self.rec.set_time("evalio_time", timestamp=stamp.to_sec())

            # Include the current map
            if VisOption.MAP in self.args:
                for (k, p), c in zip(map.items(), self.colors):
                    self.rec.log(f"{self.pn}/map/{k}", convert(p, color=c, radii=0.03))

        def log_features(self, stamp: Stamp, features: dict[str, NDArray[np.float64]]):
            if self.args is None:
                return

            if self.lidar_params is None or self.gt is None:
                raise ValueError("You needed to add a pipeline before stepping!")
            if self.pn is None or self.trajectory is None or self.colors is None:
                raise ValueError("You needed to add a pipeline before stepping!")

            self.rec.set_time("evalio_time", timestamp=stamp.to_sec())

            # Features from the scan
            if VisOption.FEATURES in self.args:
                if len(features) > 0:
                    for (k, p), c in zip(features.items(), self.colors):
                        self.rec.log(
                            f"{self.pn}/imu/lidar/{k}",
                            convert(p, color=c, radii=0.12),
                        )

    # ------------------------- For converting to rerun types ------------------------- #
    # point clouds
    @overload
    def convert(
        obj: LidarMeasurement,
        color: Optional[
            Literal["z", "intensity"]
            | tuple[int, int, int]
            | tuple[float, float, float]
        ] = None,
        radii: Optional[float] = None,
    ) -> rr.Points3D:
        """Convert a LidarMeasurement to a rerun Points3D.

        Args:
            obj (LidarMeasurement): LidarMeasurement to convert.
            color (Optional[str  |  tuple[int, int, int] | tuple[float, float, float]], optional): Optional color for points. Can be a list of colors, e.g. `[255, 0, 0]` for red, or one of `z` or `intensity`. Defaults to None.

        Returns:
            rr.Points3D: LidarMeasurement converted to rerun Points3D.
        """

        ...

    @overload
    def convert(
        obj: list[Point],
        color: Optional[
            Literal["z", "intensity"]
            | tuple[int, int, int]
            | tuple[float, float, float]
        ] = None,
        radii: Optional[float] = None,
    ) -> rr.Points3D:
        """Convert a list of Points to a rerun Points3D.

        Args:
            obj (list[Points]): Points to convert.
            color (Optional[str  |  tuple[int, int, int] | tuple[float, float, float]], optional): Optional color for points. Can be a list of colors, e.g. `[255, 0, 0]` for red, or one of `z` or `intensity`. Defaults to None.

        Returns:
            rr.Points3D: Points converted to rerun Points3D.
        """
        ...

    @overload
    def convert(
        obj: NDArray[np.float64],
        color: Optional[
            Literal["z"]
            | NDArray[np.float64]
            | tuple[int, int, int]
            | tuple[float, float, float]
        ] = None,
        radii: Optional[float] = None,
    ) -> rr.Points3D:
        """Convert an (n, 3) numpy array to a rerun Points3D.

        Args:
            obj (np.ndarray): LidarMeasurement to convert.
            color (Optional[str  |  tuple[int, int, int] | tuple[float, float, float]], optional): Optional color for points. Can be a list of colors, e.g. `[255, 0, 0]` for red, or one of `z` or `intensity`. Defaults to None.

        Returns:
            rr.Points3D: numpy array converted to rerun Points3D.
        """
        ...

    # trajectories
    @overload
    def convert(
        obj: list[SE3],
        color: Optional[tuple[int, int, int] | tuple[float, float, float]] = None,
    ) -> rr.Points3D:
        """Convert a list of SE3 poses to a rerun Points3D.

        Args:
            obj (list[SE3]): List of SE3 poses to convert.
            color (Optional[tuple[int, int, int] | tuple[float, float, float]], optional): Optional color for points, as a list of colors, e.g. `[255, 0, 0]` for red. Defaults to None.

        Returns:
            rr.Points3D: List of SE3 poses converted to rerun Points3D.
        """
        ...

    M = TypeVar("M", bound=Metadata | None)

    @overload
    def convert(
        obj: Trajectory[M],
        color: Optional[tuple[int, int, int] | tuple[float, float, float]] = None,
    ) -> rr.Points3D:
        """Convert a Trajectory a rerun Points3D.

        Args:
            obj (Trajectory): Trajectory to convert.
            color (Optional[list[int]], optional): Optional color for points, as a list of colors, e.g. `[255, 0, 0]` for red. Defaults to None.

        Returns:
            rr.Points3D: Trajectory converted to rerun Points3D.
        """
        ...

    # poses
    @overload
    def convert(obj: SE3) -> rr.Transform3D:
        """Convert a SE3 pose to a rerun Transform3D.

        Args:
            obj (SE3): SE3 pose to convert.

        Returns:
            rr.Transform3D: SE3 pose converted to rerun Transform3D.
        """
        ...

    def convert(
        obj: Any,
        color: Optional[Any] = None,
        radii: Optional[float] = None,
    ) -> rr.Transform3D | rr.Points3D:
        """Convert a variety of objects to rerun types.

        Args:
            obj (object): Object to convert. Can be a LidarMeasurement, list of Points, numpy array, SE3, or Trajectory.
            color (Optional[Any], optional): Optional color to set. See overloads for additional literal options. Defaults to None.

        Raises:
            ValueError: If the color pass is invalid.
            ValueError: If the object is not an implemented type for conversion.

        Returns:
            Rerun type.
        """
        # If we have an empty list, assume it's a point cloud with no points
        if isinstance(obj, list) and len(obj) == 0:  # type: ignore
            return rr.Points3D(np.zeros((0, 3)), colors=color, radii=radii)

        # Handle point clouds
        if isinstance(obj, LidarMeasurement):
            color_parsed = None
            if isinstance(color, tuple):
                color = cast(tuple[int, int, int], color)
                color_parsed = np.asarray(color)
            elif color == "intensity":
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
            elif color is not None:
                raise ValueError(f"Unknown color type {color}")

            return convert(
                np.asarray(obj.to_vec_positions()), color=color_parsed, radii=radii
            )

        elif isinstance(obj, list) and isinstance(obj[0], Point):
            obj = cast(list[Point], obj)
            return convert(
                LidarMeasurement(Stamp.from_sec(0), obj), color=color, radii=radii
            )

        elif isinstance(obj, np.ndarray) and len(obj.shape) == 2 and obj.shape[1] == 3:  # type: ignore
            obj = cast(NDArray[np.float64], obj)
            if isinstance(color, str) and color == "z":
                zs = obj[:, 2]
                min_z, max_z = min(zs), max(zs)
                color = np.zeros_like(obj)
                color = cast(NDArray[np.float64], color)

                val = (zs - min_z) / (max_z - min_z)
                color[:, 0] = 1.0 - val
                color[:, 1] = val

            return rr.Points3D(obj, colors=color, radii=radii)

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
            return convert(obj.poses, color=color)
        elif isinstance(obj, list) and isinstance(obj[0], SE3):
            obj = cast(list[SE3], obj)
            points = np.zeros((len(obj), 3))
            for i, pose in enumerate(obj):
                points[i] = pose.trans
            return rr.Points3D(points, colors=color)

        else:
            raise ValueError(f"Cannot convert {type(obj)} to rerun type")  # type: ignore

except Exception:

    class RerunVis:
        def __init__(
            self, args: Optional[set[VisOption]], pipeline_names: list[str]
        ) -> None:
            if args is not None:
                print_warning("Rerun not found, visualization disabled")

        def new_dataset(self, dataset: Dataset):
            pass

        def new_pipe(self, pipe_name: str, feat_num: int):
            pass

        def log_scan(self, data: LidarMeasurement):
            pass

        def log_pose(self, stamp: Stamp, pose: SE3):
            pass

        def log_map(self, stamp: Stamp, map: dict[str, NDArray[np.float64]]):
            pass

        def log_features(self, stamp: Stamp, features: dict[str, NDArray[np.float64]]):
            pass

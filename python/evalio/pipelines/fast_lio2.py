from typing import Dict, List, Optional
import numpy as np

from evalio.pipelines import Pipeline
from evalio.types import (
	SE3,
	SO3,
	Point,
	ImuParams,
	LidarParams,
	ImuMeasurement,
	LidarMeasurement,
	Stamp,
	Duration,
)


class FastLIO2(Pipeline):
	"""
	FastLIO2: Fast LiDAR-Inertial Odometry pipeline.
	This is a placeholder Python implementation for integration with evalio.
	"""

	def __init__(self, **kwargs):
		super().__init__()
		identity_rot = SO3(qx=0.0, qy=0.0, qz=0.0, qw=1.0)
		identity_trans = np.array([0.0, 0.0, 0.0])
		self._current_pose = SE3(identity_rot, identity_trans)
		self._current_map: Dict[str, List[Point]] = {"points": []}
		self._imu_params: Optional[ImuParams] = None
		self._lidar_params: Optional[LidarParams] = None
		self._imu_T_lidar: SE3 = SE3(identity_rot, identity_trans)
		self._initialized = False

		# FastLIO2-specific parameters (example, can be expanded)
		self._deskew = kwargs.get('deskew', True)
		self._max_iterations = kwargs.get('max_iterations', 20)
		self._voxel_size = kwargs.get('voxel_size', 0.5)
		self._gravity_align = kwargs.get('gravity_align', True)

		# Internal state
		self._last_stamp: Optional[Stamp] = None
		self._trajectory: List[SE3] = []
		self._imu_buffer: List[ImuMeasurement] = []

	# Info methods
	@staticmethod
	def version() -> str:
		return "1.0.0"

	@staticmethod
	def url() -> str:
		return "https://github.com/hku-mars/FAST_LIO"

	@staticmethod
	def name() -> str:
		return "fast_lio2"

	@staticmethod
	def default_params() -> Dict[str, bool | int | float | str]:
		return {
			"deskew": True,
			"max_iterations": 20,
			"voxel_size": 0.5,
			"gravity_align": True,
		}

	# Getters
	def pose(self) -> SE3:
		return self._current_pose

	def map(self) -> Dict[str, List[Point]]:
		return self._current_map

	# Setters
	def set_imu_params(self, params: ImuParams) -> None:
		self._imu_params = params

	def set_lidar_params(self, params: LidarParams) -> None:
		self._lidar_params = params

	def set_imu_T_lidar(self, T: SE3) -> None:
		self._imu_T_lidar = T

	def set_params(self, params: Dict[str, bool | int | float | str]) -> Dict[str, bool | int | float | str]:
		default_params = self.default_params()
		for key, value in params.items():
			if key in default_params:
				setattr(self, f"_{key}", value)
				default_params[key] = value
		return default_params

	# Doers
	def initialize(self) -> None:
		self._initialized = True
		identity_rot = SO3(qx=0.0, qy=0.0, qz=0.0, qw=1.0)
		identity_trans = np.array([0.0, 0.0, 0.0])
		self._current_pose = SE3(identity_rot, identity_trans)
		self._trajectory = []
		self._imu_buffer = []

	def add_imu(self, mm: ImuMeasurement) -> None:
		if not self._initialized:
			return
		self._imu_buffer.append(mm)
		if len(self._imu_buffer) > 1000:
			self._imu_buffer = self._imu_buffer[-1000:]

	def add_lidar(self, mm: LidarMeasurement) -> Dict[str, List[Point]]:
		if not self._initialized:
			return {"points": []}
		points = self._preprocess_lidar(mm)
		if self._deskew:
			points = self._motion_correct_points(points, mm)
		features = self._extract_features(points)
		self._update_pose(features, mm.stamp)
		self._update_map(features)
		viz_points = []
		for point in features[:1000]:
			viz_points.append(Point(
				x=float(point[0]),
				y=float(point[1]),
				z=float(point[2]),
				intensity=1.0,
				t=Duration.from_sec(mm.stamp.to_sec()),
				row=0,
				col=0
			))
		return {"points": viz_points}

	def _preprocess_lidar(self, mm: LidarMeasurement) -> np.ndarray:
		points = []
		for p in mm.points:
			range_sq = p.x**2 + p.y**2 + p.z**2
			if self._lidar_params and self._lidar_params.min_range**2 < range_sq < self._lidar_params.max_range**2:
				points.append([p.x, p.y, p.z])
		return np.array(points) if points else np.zeros((0, 3))

	def _motion_correct_points(self, points: np.ndarray, mm: LidarMeasurement) -> np.ndarray:
		if len(self._imu_buffer) < 2 or len(points) == 0:
			return points
		# Placeholder for motion correction logic
		return points

	def _extract_features(self, points: np.ndarray) -> np.ndarray:
		if len(points) == 0:
			return points
		voxel_size = getattr(self, '_voxel_size', 0.5)
		if len(points) > 1000:
			indices = np.random.choice(len(points), 1000, replace=False)
			return points[indices]
		return points

	def _update_pose(self, features: np.ndarray, stamp: Stamp) -> None:
		if len(features) == 0:
			return
		if len(self._trajectory) == 0:
			if self._gravity_align and len(self._imu_buffer) > 0:
				pass
		else:
			if self._last_stamp is not None:
				dt = (stamp - self._last_stamp).to_sec()
				self._current_pose = self._current_pose
		self._trajectory.append(self._current_pose)
		self._last_stamp = stamp

	def _update_map(self, features: np.ndarray) -> None:
		if len(features) == 0:
			return
		new_points = []
		for point in features:
			time_duration = Duration() if self._last_stamp is None else Duration.from_sec(self._last_stamp.to_sec())
			new_points.append(Point(
				x=float(point[0]),
				y=float(point[1]),
				z=float(point[2]),
				intensity=1.0,
				t=time_duration,
				row=0,
				col=0
			))
		self._current_map["points"].extend(new_points)
		if len(self._current_map["points"]) > 10000:
			self._current_map["points"] = self._current_map["points"][-10000:]

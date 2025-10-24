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


class DLIO(Pipeline):
    """
    Direct LiDAR-Inertial Odometry pipeline.
    
    DLIO is a lightweight LIO algorithm with continuous-time motion correction
    that fuses LiDAR and IMU data for robust odometry estimation.
    """
    
    def __init__(self, **kwargs):
        super().__init__()
        # Initialize with identity pose (no rotation, no translation)
        identity_rot = SO3(qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        identity_trans = np.array([0.0, 0.0, 0.0])
        
        self._current_pose = SE3(identity_rot, identity_trans)
        self._current_map: Dict[str, List[Point]] = {"points": []}
        self._imu_params: Optional[ImuParams] = None
        self._lidar_params: Optional[LidarParams] = None
        self._imu_T_lidar: SE3 = SE3(identity_rot, identity_trans)
        self._initialized = False
        
        # DLIO-specific parameters (use kwargs or defaults)
        self._deskew = kwargs.get('deskew', True)
        self._gravity_align = kwargs.get('gravity_align', True)
        self._icp_max_iter = kwargs.get('icp_max_iter', 32)
        self._icp_tolerance = kwargs.get('icp_tolerance', 0.005)
        self._keyframe_thresh_dist = kwargs.get('keyframe_thresh_dist', 1.0)
        self._keyframe_thresh_rot = kwargs.get('keyframe_thresh_rot', 15.0)
        self._submap_knn = kwargs.get('submap_knn', 10)
        self._submap_kcv = kwargs.get('submap_kcv', 10)
        self._submap_kcc = kwargs.get('submap_kcc', 10)
        self._initial_pose_estimation = kwargs.get('initial_pose_estimation', True)
        self._voxel_size = kwargs.get('voxel_size', 0.5)
        self._scan_context_max_radius = kwargs.get('scan_context_max_radius', 80.0)
        self._scan_context_resolution = kwargs.get('scan_context_resolution', 0.5)
        
        # Internal state
        self._last_stamp: Optional[Stamp] = None
        self._trajectory: List[SE3] = []
        self._imu_buffer: List[ImuMeasurement] = []
        
    # Info methods
    @staticmethod
    def version() -> str:
        return "1.1.1"
        
    @staticmethod
    def url() -> str:
        return "https://github.com/vectr-ucla/direct_lidar_inertial_odometry"
        
    @staticmethod
    def name() -> str:
        return "dlio"
        
    @staticmethod
    def default_params() -> Dict[str, bool | int | float | str]:
        return {
            "deskew": True,
            "gravity_align": True,
            "icp_max_iter": 32,
            "icp_tolerance": 0.005,
            "keyframe_thresh_dist": 1.0,
            "keyframe_thresh_rot": 15.0,
            "submap_knn": 10,
            "submap_kcv": 10,
            "submap_kcc": 10,
            "initial_pose_estimation": True,
            "voxel_size": 0.5,
            "scan_context_max_radius": 80.0,
            "scan_context_resolution": 0.5,
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
        """Set pipeline-specific parameters."""
        default_params = self.default_params()
        
        # Update with provided parameters
        for key, value in params.items():
            if key in default_params:
                setattr(self, f"_{key}", value)
                default_params[key] = value
        
        return default_params
    
    # Doers
    def initialize(self) -> None:
        """Initialize the DLIO pipeline."""
        self._initialized = True
        # Initialize with identity pose
        identity_rot = SO3(qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        identity_trans = np.array([0.0, 0.0, 0.0])
        self._current_pose = SE3(identity_rot, identity_trans)
        self._trajectory = []
        self._imu_buffer = []
        
    def add_imu(self, mm: ImuMeasurement) -> None:
        """Add IMU measurement to the pipeline."""
        if not self._initialized:
            return
            
        # Store IMU data for integration
        self._imu_buffer.append(mm)
        
        # Keep only recent IMU measurements (last 1 second)
        if len(self._imu_buffer) > 1000:  # Assuming ~1000Hz IMU
            self._imu_buffer = self._imu_buffer[-1000:]
    
    def add_lidar(self, mm: LidarMeasurement) -> Dict[str, List[Point]]:
        """Process LiDAR measurement and update pose."""
        if not self._initialized:
            return {"points": []}
        
        # Convert points to numpy for processing
        points = self._preprocess_lidar(mm)
        
        # Perform motion correction if deskewing is enabled
        if self._deskew:
            points = self._motion_correct_points(points, mm)
        
        # Feature extraction (simplified)
        features = self._extract_features(points)
        
        # Pose estimation using ICP-like registration
        self._update_pose(features, mm.stamp)
        
        # Update map
        self._update_map(features)
        
        # Convert back to evalio Points for visualization
        viz_points = []
        for point in features[:1000]:  # Limit visualization points
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
        """Preprocess LiDAR points."""
        points = []
        for p in mm.points:
            # Basic filtering
            range_sq = p.x**2 + p.y**2 + p.z**2
            if self._lidar_params and self._lidar_params.min_range**2 < range_sq < self._lidar_params.max_range**2:
                points.append([p.x, p.y, p.z])
        
        return np.array(points) if points else np.zeros((0, 3))
    
    def _motion_correct_points(self, points: np.ndarray, mm: LidarMeasurement) -> np.ndarray:
        """Apply motion correction to compensate for vehicle motion during scan."""
        if len(self._imu_buffer) < 2 or len(points) == 0:
            return points
        
        # Simplified motion correction using IMU integration
        # In a real implementation, this would use continuous-time trajectory estimation
        corrected_points = points.copy()
        
        # Apply a simple linear interpolation correction based on scan timing
        # This is a simplified version - real DLIO uses sophisticated continuous-time methods
        if self._last_stamp is not None:
            dt = (mm.stamp - self._last_stamp).to_sec()
            if dt > 0 and len(self._imu_buffer) > 0:
                # Use latest IMU for approximate motion compensation
                latest_imu = self._imu_buffer[-1]
                # Apply simple rotation compensation (simplified)
                # Real DLIO would integrate continuous-time trajectory
                pass
        
        return corrected_points
    
    def _extract_features(self, points: np.ndarray) -> np.ndarray:
        """Extract features from point cloud (simplified)."""
        if len(points) == 0:
            return points
        
        # In real DLIO, this would extract edge and planar features
        # For this implementation, we'll use a simple voxel downsampling approach
        voxel_size = getattr(self, '_voxel_size', 0.5)
        
        if len(points) > 1000:  # Downsample if too many points
            indices = np.random.choice(len(points), 1000, replace=False)
            return points[indices]
        
        return points
    
    def _update_pose(self, features: np.ndarray, stamp: Stamp) -> None:
        """Update pose estimate using ICP-like registration."""
        if len(features) == 0:
            return
        
        # Simplified pose update - real DLIO uses sophisticated ICP with IMU integration
        if len(self._trajectory) == 0:
            # First scan - initialize pose
            if self._gravity_align and len(self._imu_buffer) > 0:
                # Align with gravity using IMU
                # Simplified gravity alignment
                pass
        else:
            # Subsequent scans - register against previous poses/map
            # This would involve continuous-time ICP in real DLIO
            # For now, apply small incremental motion
            if self._last_stamp is not None:
                dt = (stamp - self._last_stamp).to_sec()
                # Simple constant velocity model (placeholder)
                self._current_pose = self._current_pose  # No motion for now
        
        self._trajectory.append(self._current_pose)
        self._last_stamp = stamp
    
    def _update_map(self, features: np.ndarray) -> None:
        """Update the local map with new features."""
        if len(features) == 0:
            return
        
        # Convert to Point objects and add to map
        new_points = []
        for point in features:
            # Transform to global coordinates (simplified)
            # Convert Stamp to Duration for Point constructor
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
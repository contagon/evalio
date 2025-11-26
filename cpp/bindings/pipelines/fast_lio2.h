#pragma once

#include <deque>
#include <memory>
#include <stdexcept>

#include "evalio/pipeline.h"
#include "evalio/types.h"

// Fast-LIO2 headers
#include "IKFoM_toolkit/esekfom/esekfom.hpp"
#include "common_lib.h"
#include "ikd-Tree/ikd_Tree.h"
#include "preprocess.h"

// Type aliases matching Fast-LIO2
typedef pcl::PointXYZINormal PointType;
typedef KD_TREE<PointType> ikdtree;

inline evalio::Point to_evalio_point(const PointType& pt) {
  return {
    .x = pt.x,
    .y = pt.y,
    .z = pt.z,
    .intensity = pt.intensity,
    .t = evalio::Duration::from_sec(0),
    .range = 0.0,
    .row = 0,
    .col = 0
  };
}

inline PointType to_pcl_point(const evalio::Point& pt) {
  PointType pcl_pt;
  pcl_pt.x = pt.x;
  pcl_pt.y = pt.y;
  pcl_pt.z = pt.z;
  pcl_pt.intensity = pt.intensity;
  return pcl_pt;
}

class FastLIO2: public evalio::Pipeline {
public:
  FastLIO2() : lidar_T_imu_(evalio::SE3::identity()) {}

  // Info
  static std::string version() {
    return XSTR(EVALIO_FAST_LIO2);
  }

  static std::string name() {
    return "fast_lio2";
  }

  static std::string url() {
    return "https://github.com/hku-mars/FAST_LIO";
  }

  // clang-format off
  EVALIO_SETUP_PARAMS(
    (double, filter_size_surf, 0.5, filter_size_surf_),
    (double, filter_size_map, 0.5, filter_size_map_),
    (double, cube_side_length, 200.0, cube_len_),
    (double, det_range, 300.0, det_range_),
    (int, max_iteration, 4, max_iteration_),
    (double, epsi, 0.001, epsi_),
    (bool, pcd_save_en, false, pcd_save_en_),
    (double, lidar_max_range, 100.0, lidar_max_range_),
    (double, lidar_min_range, 0.5, lidar_min_range_)
  );
  // clang-format on

  // Getters
  const evalio::SE3 pose() override {
    if (!esekf_initialized_) {
      return evalio::SE3::identity();
    }
    // Extract pose from state
    Eigen::Vector3d pos = state_.pos;
    Eigen::Matrix3d rot = state_.rot.toRotationMatrix();
    Eigen::Quaterniond q(rot);
    evalio::SO3 so3 = {.qx = q.x(), .qy = q.y(), .qz = q.z(), .qw = q.w()};
    return evalio::SE3(so3, pos) * lidar_T_imu_;
  }

  const std::map<std::string, std::vector<evalio::Point>> map() override {
    std::vector<evalio::Point> evalio_map;
    if (ikdtree_) {
      PointVector points_near;
      ikdtree_->flatten(ikdtree_->Root_Node, points_near, NOT_RECORD);
      evalio_map.reserve(points_near.size());
      for (const auto& pt : points_near) {
        evalio_map.push_back(to_evalio_point(pt));
      }
    }
    return {{"point", evalio_map}};
  }

  // Setters
  void set_imu_params(evalio::ImuParams params) override {
    // Fast-LIO2 uses ESEKF which has different noise modeling
    // These would need to be converted to Q and R matrices
  }

  void set_lidar_params(evalio::LidarParams params) override {
    lidar_max_range_ = params.max_range;
    lidar_min_range_ = params.min_range;
    num_scan_lines_ = params.num_rows;
  }

  void set_imu_T_lidar(evalio::SE3 T) override {
    lidar_T_imu_ = T.inverse();
  }

  // Doers
  void initialize() override {
    // Initialize ikd-tree for map
    ikdtree_ = std::make_shared<ikdtree>(0.5, 0.6, 0.2);
    
    // Initialize preprocessor
    p_pre_ = std::make_shared<Preprocess>();
    p_pre_->lidar_type = AVIA; // Default, should be configurable
    
    // Initialize state
    state_ = state_ikfom();
    esekf_initialized_ = false;
    
    // Initialize voxel grid filters
    downSizeFilterSurf_.setLeafSize(filter_size_surf_, filter_size_surf_, filter_size_surf_);
    downSizeFilterMap_.setLeafSize(filter_size_map_, filter_size_map_, filter_size_map_);
  }

  void add_imu(evalio::ImuMeasurement mm) override {
    // Queue IMU measurement for integration
    ImuMeas imu_meas;
    imu_meas.timestamp = mm.stamp.to_sec();
    imu_meas.linear_acceleration = mm.accel;
    imu_meas.angular_velocity = mm.gyro;
    imu_buffer_.push_back(imu_meas);
  }

  std::map<std::string, std::vector<evalio::Point>>
  add_lidar(evalio::LidarMeasurement mm) override {
    // Convert to PCL point cloud
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud->points.reserve(mm.points.size());
    for (const auto& pt : mm.points) {
      cloud->points.push_back(to_pcl_point(pt));
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // This is where the main processing would happen:
    // 1. Preprocess point cloud (feature extraction)
    // 2. Integrate IMU measurements
    // 3. Run ESEKF update with point-to-plane residuals
    // 4. Update map with new points
    
    // For now, return the input points
    std::vector<evalio::Point> result;
    result.reserve(cloud->points.size());
    for (const auto& pt : cloud->points) {
      result.push_back(to_evalio_point(pt));
    }
    return {{"point", result}};
  }

private:
  // State and filter
  state_ikfom state_;
  bool esekf_initialized_ = false;
  
  // Map
  std::shared_ptr<ikdtree> ikdtree_;
  
  // Preprocessing
  std::shared_ptr<Preprocess> p_pre_;
  
  // IMU buffer
  struct ImuMeas {
    double timestamp;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
  };
  std::deque<ImuMeas> imu_buffer_;
  
  // Filters
  pcl::VoxelGrid<PointType> downSizeFilterSurf_;
  pcl::VoxelGrid<PointType> downSizeFilterMap_;
  
  // Parameters
  double filter_size_surf_ = 0.5;
  double filter_size_map_ = 0.5;
  double cube_len_ = 200.0;
  double det_range_ = 300.0;
  int max_iteration_ = 4;
  double epsi_ = 0.001;
  bool pcd_save_en_ = false;
  double lidar_max_range_ = 100.0;
  double lidar_min_range_ = 0.5;
  int num_scan_lines_ = 16;
  
  // Transforms
  evalio::SE3 lidar_T_imu_;
};

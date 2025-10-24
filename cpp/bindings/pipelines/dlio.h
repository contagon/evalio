#pragma once

#include <memory>
#include <stdexcept>

#include "evalio/pipeline.h"
#include "evalio/types.h"

// This is a placeholder header for DLIO C++ implementation
// In a full implementation, this would include DLIO's headers
// #include "dlio/pipeline/DLIO.hpp"

class DLIO: public evalio::Pipeline {
public:
  DLIO() {
    // Initialize DLIO configuration
  }

  // Info
  static std::string version() {
    return "1.1.1";
  }

  static std::string name() {
    return "dlio";
  }

  static std::string url() {
    return "https://github.com/vectr-ucla/direct_lidar_inertial_odometry";
  }

  static std::map<std::string, evalio::Param> default_params() {
    return {
      {"deskew", true},
      {"gravity_align", true},
      {"icp_max_iter", 32},
      {"icp_tolerance", 0.005},
      {"keyframe_thresh_dist", 1.0},
      {"keyframe_thresh_rot", 15.0},
      {"submap_knn", 10},
      {"submap_kcv", 10},
      {"submap_kcc", 10},
      {"initial_pose_estimation", true},
      {"voxel_size", 0.5},
      {"scan_context_max_radius", 80.0},
      {"scan_context_resolution", 0.5},
    };
  }

  // Getters
  const evalio::SE3 pose() override {
    // Return current pose estimate
    return evalio::SE3();
  }

  const std::map<std::string, std::vector<evalio::Point>> map() override {
    // Return current map for visualization
    return {{"points", {}}};
  }

  // Setters
  void set_imu_params(evalio::ImuParams params) override {
    // Set IMU parameters
  }

  void set_lidar_params(evalio::LidarParams params) override {
    // Set LiDAR parameters
  }

  void set_imu_T_lidar(evalio::SE3 T) override {
    // Set IMU to LiDAR transformation
  }

  std::map<std::string, evalio::Param> 
  set_params(std::map<std::string, evalio::Param> params) override {
    // Set pipeline-specific parameters
    auto default_params_map = default_params();
    
    // Update with provided parameters
    for (const auto& [key, value] : params) {
      if (default_params_map.find(key) != default_params_map.end()) {
        default_params_map[key] = value;
      }
    }
    
    return default_params_map;
  }

  // Doers
  void initialize() override {
    // Initialize DLIO pipeline
  }

  void add_imu(evalio::ImuMeasurement mm) override {
    // Process IMU measurement
  }

  std::map<std::string, std::vector<evalio::Point>>
  add_lidar(evalio::LidarMeasurement mm) override {
    // Process LiDAR measurement and return features for visualization
    return {{"points", {}}};
  }

private:
  // DLIO-specific members would go here
  // std::unique_ptr<dlio::DLIO> dlio_pipeline_;
  // dlio::DLIOConfig config_;
};
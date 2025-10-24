#pragma once

#include <memory>
#include <stdexcept>

#include "evalio/pipeline.h"
#include "evalio/types.h"

// This is a placeholder header for FastLIO2 C++ implementation
// In a full implementation, this would include FastLIO2's headers
// #include "fastlio2/fastlio2.h"

class FastLIO2: public evalio::Pipeline {
public:
  FastLIO2() {
    // Initialize FastLIO2 configuration
  }

  // Info
  static std::string version() {
    return "1.0.0";
  }

  static std::string name() {
    return "fast_lio2";
  }

  static std::string url() {
    return "https://github.com/hku-mars/FAST_LIO";
  }

  static std::map<std::string, evalio::Param> default_params() {
    return {
      {"deskew", true},
      {"max_iterations", 20},
      {"voxel_size", 0.5},
      {"gravity_align", true},
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
    for (const auto& [key, value] : params) {
      if (default_params_map.find(key) != default_params_map.end()) {
        default_params_map[key] = value;
      }
    }
    return default_params_map;
  }

  // Doers
  void initialize() override {
    // Initialize FastLIO2 pipeline
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
  // FastLIO2-specific members would go here
  // std::unique_ptr<fastlio2::FastLIO2> fastlio2_pipeline_;
  // fastlio2::Config config_;
};

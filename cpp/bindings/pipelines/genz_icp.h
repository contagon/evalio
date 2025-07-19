#pragma once

#include <memory>
#include <stdexcept>

#include "evalio/pipeline.h"
#include "evalio/types.h"
#include "genz_icp/pipeline/GenZICP.hpp"

class GenZICP: public evalio::Pipeline {
public:
  GenZICP() : config_() {}

  // Info
  static std::string version() {
    return XSTR(EVALIO_GENZ_ICP);
  }

  static std::string name() {
    return "genz";
  }

  static std::string url() {
    return "https://github.com/cocel-postech/genz-icp";
  }

  static std::map<std::string, evalio::Param> default_params() {
    return {
      // map params
      {"map_cleanup_radius", 400.0},
      {"max_points_per_voxel", 1},
      // voxelize params
      {"voxel_size", 0.25},
      {"desired_num_voxelized_points", 2000},
      // th params
      {"min_motion_th", 0.1},
      {"initial_threshold", 2.0},
      {"planarity_threshold", 0.1},
      // motion compensation
      {"deskew", false},
      // registration params
      {"max_num_iterations", 150},
      {"convergence_criterion", 0.0001},
    };
  }

  // Getters
  const evalio::SE3 pose() override {
    const auto pose =
      !genz_icp_->poses().empty() ? genz_icp_->poses().back() : Sophus::SE3d();
    return to_evalio_se3(pose);
  }

  const std::map<std::string, std::vector<evalio::Point>> map() override {
    std::vector<Eigen::Vector3d> map = genz_icp_->LocalMap();
    std::vector<evalio::Point> evalio_map;
    evalio_map.reserve(map.size());
    for (auto point : map) {
      evalio_map.push_back(to_evalio_point(point));
    }
    return {{"point", evalio_map}};
  }

  // Setters
  void set_imu_params(evalio::ImuParams params) override {}

  void set_lidar_params(evalio::LidarParams params) override {
    config_.max_range = params.max_range;
    config_.min_range = params.min_range;
  }

  void set_imu_T_lidar(evalio::SE3 T) override {
    lidar_T_imu_ = to_sophus_se3(T).inverse();
  }

  void set_params(std::map<std::string, evalio::Param> params) override {
    for (auto& [key, value] : params) {
      // map params
      if (key == "map_cleanup_radius") {
        config_.map_cleanup_radius = std::get<double>(value);
      } else if (key == "max_points_per_voxel") {
        config_.max_points_per_voxel = std::get<int>(value);
      }
      // voxelize params
      else if (key == "voxel_size") {
        config_.voxel_size = std::get<double>(value);
      } else if (key == "desired_num_voxelized_points") {
        config_.desired_num_voxelized_points = std::get<int>(value);
      }
      // th params
      else if (key == "min_motion_th") {
        config_.min_motion_th = std::get<double>(value);
      } else if (key == "initial_threshold") {
        config_.initial_threshold = std::get<double>(value);
      } else if (key == "planarity_threshold") {
        config_.planarity_threshold = std::get<double>(value);
      }
      // motion compensation
      else if (key == "deskew") {
        config_.deskew = std::get<bool>(value);
      }
      // registration params
      else if (key == "max_num_iterations") {
        config_.max_num_iterations = std::get<int>(value);
      } else if (key == "convergence_criterion") {
        config_.convergence_criterion = std::get<double>(value);
      } else {
        throw std::invalid_argument("Unknown parameter: " + key);
      }
    }
  }

  // Doers
  void initialize() override {
    genz_icp_ = std::make_unique<genz_icp::pipeline::GenZICP>(config_);
  }

  void add_imu(evalio::ImuMeasurement mm) override {}

  std::map<std::string, std::vector<evalio::Point>>
  add_lidar(evalio::LidarMeasurement mm) override {
    // Set everything up
    std::vector<Eigen::Vector3d> points;
    points.reserve(mm.points.size());
    std::vector<double> timestamps;
    timestamps.reserve(mm.points.size());

    // Copy
    for (auto point : mm.points) {
      points.push_back(to_eigen_point(point));
      timestamps.push_back(point.t.to_sec());
    }

    // Run through pipeline
    const auto& [_, used_points] = genz_icp_->RegisterFrame(points, timestamps);
    std::vector<evalio::Point> result;
    result.reserve(used_points.size());
    for (auto point : used_points) {
      result.push_back(to_evalio_point(point));
    }
    return {{"point", result}};
  }

private:
  std::unique_ptr<genz_icp::pipeline::GenZICP> genz_icp_;
  genz_icp::pipeline::GenZConfig config_;
  Sophus::SE3d lidar_T_imu_;

  // Misc helpers
  inline evalio::Point to_evalio_point(Eigen::Vector3d point) {
    return {
      .x = point[0],
      .y = point[1],
      .z = point[2],
      .intensity = 0.0,
      .t = evalio::Duration::from_sec(0),
      .row = 0,
      .col = 0
    };
  }

  inline Eigen::Vector3d to_eigen_point(evalio::Point point) {
    return {point.x, point.y, point.z};
  }

  inline evalio::SE3 to_evalio_se3(Sophus::SE3d pose) {
    const auto t = pose.translation();
    const auto q = pose.unit_quaternion();
    const auto rot =
      evalio::SO3 {.qx = q.x(), .qy = q.y(), .qz = q.z(), .qw = q.w()};
    return evalio::SE3(rot, t);
  }

  inline Sophus::SE3d to_sophus_se3(evalio::SE3 pose) {
    return Sophus::SE3d(Sophus::SO3d(pose.rot.toEigen()), pose.trans);
  }
};

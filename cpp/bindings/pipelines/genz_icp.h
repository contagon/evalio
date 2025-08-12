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

  // clang-format off
  EVALIO_SETUP_PARAMS(
    // map params
    // Make map_cleanup_radius = max_range
    (int, max_points_per_voxel, 1, config_.max_points_per_voxel),
    // voxelize params
    (double, voxel_size, 0.3, config_.voxel_size),
    (int, desired_num_voxelized_points, 2000, config_.desired_num_voxelized_points),
    // th params
    (double, min_motion_th, 0.1, config_.min_motion_th),
    (double, initial_threshold, 2.0, config_.initial_threshold),
    (double, planarity_threshold, 0.2, config_.planarity_threshold),
    // motion compensation
    (bool, deskew, false, config_.deskew),
    // registration params
    (int, max_num_iterations, 100, config_.max_num_iterations),
    (double, convergence_criterion, 0.0001, config_.convergence_criterion)
  );
  // clang-format on

  // Getters
  const evalio::SE3 pose() override {
    const auto pose =
      !genz_icp_->poses().empty() ? genz_icp_->poses().back() : Sophus::SE3d();
    return to_evalio_se3(pose * lidar_T_imu_);
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
    config_.min_range = params.min_range;
    config_.max_range = params.max_range;
    config_.map_cleanup_radius = params.max_range;
  }

  void set_imu_T_lidar(evalio::SE3 T) override {
    lidar_T_imu_ = to_sophus_se3(T).inverse();
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
    const auto& [planar_points, nonplanar_points] =
      genz_icp_->RegisterFrame(points, timestamps);
    const auto lidar_T_world = genz_icp_->poses().back().inverse();

    // Return the used points
    // These are all in the global frame, so we need to convert them
    std::vector<evalio::Point> ev_planar_points;
    ev_planar_points.reserve(planar_points.size());
    for (auto point : planar_points) {
      ev_planar_points.push_back(to_evalio_point(lidar_T_world * point));
    }

    std::vector<evalio::Point> ev_nonplanar_points;
    ev_nonplanar_points.reserve(nonplanar_points.size());
    for (auto point : nonplanar_points) {
      ev_nonplanar_points.push_back(to_evalio_point(lidar_T_world * point));
    }

    return {{"nonplanar", ev_nonplanar_points}, {"planar", ev_planar_points}};
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

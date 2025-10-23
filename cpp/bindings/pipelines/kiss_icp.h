#pragma once

#include <memory>

#include "evalio/pipeline.h"
#include "evalio/types.h"
#include "kiss_icp/pipeline/KissICP.hpp"

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

class KissICP: public evalio::Pipeline {
public:
  KissICP() : config_() {}

  // Info
  static std::string version() {
    return XSTR(EVALIO_KISS_ICP);
  }

  static std::string name() {
    return "kiss";
  }

  static std::string url() {
    return "https://github.com/PRBonn/kiss-icp";
  }

  EVALIO_SETUP_PARAMS(
    (double, voxel_size, 1.0, config_.voxel_size),
    (double, min_motion_th, 0.1, config_.min_motion_th),
    (double, initial_threshold, 2.0, config_.initial_threshold),
    (double, convergence_criterion, 0.0001, config_.convergence_criterion),
    (int, max_num_iterations, 500, config_.max_num_iterations),
    (int, max_num_threads, 0, config_.max_num_threads),
    (int, max_points_per_voxel, 20, config_.max_points_per_voxel),
    (bool, deskew, false, config_.deskew)
  );

  // Getters
  const std::map<std::string, std::vector<evalio::Point>> map() override {
    std::vector<Eigen::Vector3d> map = kiss_icp_->LocalMap();
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

  // Doers
  void initialize() override {
    kiss_icp_ = std::make_unique<kiss_icp::pipeline::KissICP>(config_);
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
    const auto& [_, used_points] = kiss_icp_->RegisterFrame(points, timestamps);
    std::vector<evalio::Point> result;
    result.reserve(used_points.size());
    for (auto point : used_points) {
      result.push_back(to_evalio_point(point));
    }

    // Save the estimate
    const Sophus::SE3d pose = kiss_icp_->pose() * lidar_T_imu_;
    this->push_back_estimate(mm.stamp, to_evalio_se3(pose));

    return {{"point", result}};
  }

private:
  std::unique_ptr<kiss_icp::pipeline::KissICP> kiss_icp_;
  kiss_icp::pipeline::KISSConfig config_;
  Sophus::SE3d lidar_T_imu_;
};

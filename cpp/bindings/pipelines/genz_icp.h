#pragma once

#include <algorithm>
#include <memory>

#include "evalio/convert/eigen.h"
#include "evalio/convert/sophus.h"
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

  // These parameters are pulled from the default ros node parameters
  // https://github.com/cocel-postech/genz-icp/blob/master/ros/launch/odometry.launch#L11-L24
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
    return evalio::convert<evalio::SE3>(pose * lidar_T_imu_);
  }

  const std::map<std::string, std::vector<evalio::Point>> map() override {
    std::vector<Eigen::Vector3d> map = genz_icp_->LocalMap();
    std::vector<evalio::Point> evalio_map;
    evalio_map.reserve(map.size());
    for (auto point : map) {
      evalio_map.push_back(evalio::convert<evalio::Point>(point));
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
    lidar_T_imu_ = evalio::convert<Sophus::SE3d>(T).inverse();
  }

  // Doers
  void initialize() override {
    genz_icp_ = std::make_unique<genz_icp::pipeline::GenZICP>(config_);
  }

  void add_imu(evalio::ImuMeasurement mm) override {}

  std::map<std::string, std::vector<evalio::Point>>
  add_lidar(evalio::LidarMeasurement mm) override {
    // Set everything up
    auto points = evalio::convert<Eigen::Vector3d>(mm.points);
    auto timestamps = evalio::convert<double>(mm.points);

    // Run through pipeline
    auto [planar_points, nonplanar_points] =
      genz_icp_->RegisterFrame(points, timestamps);
    auto lidar_T_world = genz_icp_->poses().back().inverse();

    // These are all in the global frame, so we need to convert them
    std::transform(
      planar_points.begin(),
      planar_points.end(),
      planar_points.begin(),
      [&](auto point) { return lidar_T_imu_ * point; }
    );
    std::transform(
      nonplanar_points.begin(),
      nonplanar_points.end(),
      nonplanar_points.begin(),
      [&](auto point) { return lidar_T_imu_ * point; }
    );

    // Return the used points
    return {
      {"nonplanar", evalio::convert<evalio::Point>(nonplanar_points)},
      {"planar", evalio::convert<evalio::Point>(planar_points)}
    };
  }

private:
  std::unique_ptr<genz_icp::pipeline::GenZICP> genz_icp_;
  genz_icp::pipeline::GenZConfig config_;
  Sophus::SE3d lidar_T_imu_;
};

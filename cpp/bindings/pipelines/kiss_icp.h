#pragma once

#include <memory>

#include "evalio/convert/eigen.h"
#include "evalio/convert/sophus.h"
#include "evalio/pipeline.h"
#include "evalio/types.h"
#include "kiss_icp/pipeline/KissICP.hpp"

namespace ev = evalio;

class KissICP: public ev::Pipeline {
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
  const std::map<std::string, std::vector<ev::Point>> map() override {
    return ev::make_map("point", kiss_icp_->LocalMap());
  }

  // Setters
  void set_imu_params(ev::ImuParams params) override {}

  void set_lidar_params(ev::LidarParams params) override {
    config_.max_range = params.max_range;
    config_.min_range = params.min_range;
  }

  void set_imu_T_lidar(ev::SE3 T) override {
    lidar_T_imu_ = ev::convert<Sophus::SE3d>(T).inverse();
  }

  // Doers
  void initialize() override {
    kiss_icp_ = std::make_unique<kiss_icp::pipeline::KissICP>(config_);
  }

  void add_imu(evalio::ImuMeasurement mm) override {}

  void add_lidar(ev::LidarMeasurement mm) override {
    // Convert inputs
    auto points = ev::convert_iter<std::vector<Eigen::Vector3d>>(mm.points);
    auto timestamps = ev::convert_iter<std::vector<double>>(mm.points);

    // Run through pipeline
    const auto& [_, used_points] = kiss_icp_->RegisterFrame(points, timestamps);

    // Save the estimate
    this->save(mm.stamp, kiss_icp_->pose() * lidar_T_imu_);

    // Save features
    this->save(mm.stamp, "point", used_points);
  }

private:
  std::unique_ptr<kiss_icp::pipeline::KissICP> kiss_icp_;
  kiss_icp::pipeline::KISSConfig config_;
  Sophus::SE3d lidar_T_imu_;
};

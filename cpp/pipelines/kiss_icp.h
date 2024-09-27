#pragma once

#include <memory>
#include <stdexcept>

#include "base.h"
#include "kiss_icp/pipeline/KissICP.hpp"
#include "types.h"

evalio::Point to_evalio_point(Eigen::Vector4d point) {
  return {.x = point[0],
          .y = point[1],
          .z = point[2],
          .intensity = point[3],
          .stamp_offset = 0,
          .row = 0,
          .column = 0};
}

Eigen::Vector4d to_eigen_point(evalio::Point point) {
  return {point.x, point.y, point.z, point.intensity};
}

class KissICP : public evalio::Pipeline {
 public:
  KissICP() : config_() {};

  // Getters
  const evalio::SE3 pose() override {
    const Sophus::SE3d pose = kiss_icp_->pose();
    const auto t = pose.translation();
    const auto q = pose.unit_quaternion();
    const auto rot =
        evalio::SO3{.qx = q.x(), .qy = q.y(), .qz = q.z(), .qw = q.w()};
    return evalio::SE3(rot, t);
  }

  const std::vector<evalio::Point> map() override {
    std::vector<Eigen::Vector4d> map = kiss_icp_->LocalMap();
    std::vector<evalio::Point> evalio_map(map.size());
    for (auto point : map) {
      evalio_map.push_back(to_evalio_point(point));
    }
    return evalio_map;
  }

  // Setters
  void set_imu_params(evalio::ImuParams params) override {};
  void set_lidar_params(evalio::LidarParams params) override {};
  void set_imu_T_lidar(evalio::SE3 T) override {};

  void set_param(std::string key, std::string value) override {
    throw std::invalid_argument(
        "Invalid parameter, KissICP doesn't have string param " + key);
  }

  void set_param(std::string key, double value) override {
    if (key == "voxel_size") {
      config_.voxel_size = value;
    } else if (key == "max_range") {
      config_.max_range = value;
    } else if (key == "min_range") {
      config_.min_range = value;
    } else if (key == "min_motion_th") {
      config_.min_motion_th = value;
    } else if (key == "initial_threshold") {
      config_.initial_threshold = value;
    } else if (key == "max_num_iterations") {
      config_.max_num_iterations = value;
    } else if (key == "convergence_criterion") {
      config_.convergence_criterion = value;
    } else if (key == "max_num_threads") {
      config_.max_num_threads = value;
    } else {
      throw std::invalid_argument(
          "Invalid parameter, KissICP doesn't have double param " + key);
    }
  }

  void set_param(std::string key, int value) override {
    if (key == "max_points_per_voxel") {
      config_.max_points_per_voxel = value;
    } else {
      throw std::invalid_argument(
          "Invalid parameter, KissICP doesn't have int param " + key);
    }
  }

  void set_param(std::string key, bool value) override {
    if (key == "deskew") {
      config_.deskew = value;
    } else {
      throw std::invalid_argument(
          "Invalid parameter, KissICP doesn't have bool param " + key);
    }
  }

  void initialize() override {
    kiss_icp_ = std::make_unique<kiss_icp::pipeline::KissICP>(config_);
  }

  void add_imu(evalio::ImuMeasurement mm) override {};

  void add_lidar(evalio::LidarMeasurement mm) override {
    std::vector<Eigen::Vector4d> points(mm.points.size());
    for (auto point : mm.points) {
      points.push_back(to_eigen_point(point));
    }
    kiss_icp_->RegisterFrame(points);
  }

 private:
  std::unique_ptr<kiss_icp::pipeline::KissICP> kiss_icp_;
  kiss_icp::pipeline::KISSConfig config_;
};
#pragma once

#include <memory>
#include <stdexcept>

#include "evalio/pipeline.h"
#include "evalio/types.h"
#include "kiss_icp/pipeline/KissICP.hpp"

inline evalio::Point to_evalio_point(Eigen::Vector3d point) {
  return {.x = point[0],
          .y = point[1],
          .z = point[2],
          .intensity = 0.0,
          .t = evalio::Duration::from_sec(0),
          .row = 0,
          .col = 0};
}

inline Eigen::Vector3d to_eigen_point(evalio::Point point) {
  return {point.x, point.y, point.z};
}

inline evalio::SE3 to_evalio_se3(Sophus::SE3d pose) {
  const auto t = pose.translation();
  const auto q = pose.unit_quaternion();
  const auto rot =
      evalio::SO3{.qx = q.x(), .qy = q.y(), .qz = q.z(), .qw = q.w()};
  return evalio::SE3(rot, t);
}

inline Sophus::SE3d to_sophus_se3(evalio::SE3 pose) {
  return Sophus::SE3d(Sophus::SO3d(pose.rot.toEigen()), pose.trans);
}

class KissICP : public evalio::Pipeline {
public:
  KissICP() : config_() {};

  // Info
  static std::string name() { return "kiss"; }
  static std::string url() { return "https://github.com/PRBonn/kiss-icp"; }
  static std::map<std::string, evalio::Param> default_params() {
    return {
        {"voxel_size", 1.0},          {"max_range", 100.0},
        {"min_range", 5.0},           {"min_motion_th", 0.1},
        {"initial_threshold", 2.0},   {"convergence_criterion", 0.0001},
        {"max_num_iterations", 500},  {"max_num_threads", 0},
        {"max_points_per_voxel", 20}, {"deskew", false},
    };
  }

  // Getters
  const evalio::SE3 pose() override {
    const Sophus::SE3d pose = kiss_icp_->pose() * lidar_T_imu_;
    return to_evalio_se3(pose);
  }

  const std::vector<evalio::Point> map() override {
    std::vector<Eigen::Vector3d> map = kiss_icp_->LocalMap();
    std::vector<evalio::Point> evalio_map;
    evalio_map.reserve(map.size());
    for (auto point : map) {
      evalio_map.push_back(to_evalio_point(point));
    }
    return evalio_map;
  }

  // Setters
  void set_imu_params(evalio::ImuParams params) override {};
  void set_lidar_params(evalio::LidarParams params) override {};
  void set_imu_T_lidar(evalio::SE3 T) override {
    lidar_T_imu_ = to_sophus_se3(T).inverse();
  };

  void set_params(std::map<std::string, evalio::Param> params) override {
    for (auto &[key, value] : params) {
      if (std::holds_alternative<bool>(value)) {
        if (key == "deskew") {
          config_.deskew = std::get<bool>(value);
        } else {
          throw std::invalid_argument(
              "Invalid parameter, KissICP doesn't have bool param " + key);
        }
      } else if (std::holds_alternative<int>(value)) {
        if (key == "max_points_per_voxel") {
          config_.max_points_per_voxel = std::get<int>(value);
        } else if (key == "max_num_iterations") {
          config_.max_num_iterations = std::get<int>(value);
        } else if (key == "max_num_threads") {
          config_.max_num_threads = std::get<int>(value);
        } else {
          throw std::invalid_argument(
              "Invalid parameter, KissICP doesn't have int param " + key);
        }
      } else if (std::holds_alternative<double>(value)) {
        if (key == "voxel_size") {
          config_.voxel_size = std::get<double>(value);
        } else if (key == "max_range") {
          config_.max_range = std::get<double>(value);
        } else if (key == "min_range") {
          config_.min_range = std::get<double>(value);
        } else if (key == "min_motion_th") {
          config_.min_motion_th = std::get<double>(value);
        } else if (key == "initial_threshold") {
          config_.initial_threshold = std::get<double>(value);
        } else if (key == "convergence_criterion") {
          config_.convergence_criterion = std::get<double>(value);
        } else {
          throw std::invalid_argument(
              "Invalid parameter, KissICP doesn't have double param " + key);
        }
      } else if (std::holds_alternative<std::string>(value)) {
        throw std::invalid_argument(
            "Invalid parameter, KissICP doesn't have string param " + key);
      } else {
        throw std::invalid_argument("Invalid parameter type");
      }
    }
  }

  // Doers
  void initialize() override {
    kiss_icp_ = std::make_unique<kiss_icp::pipeline::KissICP>(config_);
  }

  void add_imu(evalio::ImuMeasurement mm) override {};

  std::vector<evalio::Point> add_lidar(evalio::LidarMeasurement mm) override {
    // Set everything up
    std::vector<Eigen::Vector3d> points;
    points.reserve(mm.points.size());
    std::vector<double> timestamps;
    timestamps.reserve(mm.points.size());

    // Assuming our mm stamp from middle of scan seems to work well
    // => kiss expects timestamps in range [0, 1] and mid_pose_timestamp = 0.5
    auto min_timestamp = *std::min_element(
        mm.points.begin(), mm.points.end(),
        [](evalio::Point a, evalio::Point b) { return a.t < b.t; });
    auto max_timestamp = *std::max_element(
        mm.points.begin(), mm.points.end(),
        [](evalio::Point a, evalio::Point b) { return a.t < b.t; });

    double diff = max_timestamp.t.to_sec() - min_timestamp.t.to_sec();
    double scale = 1.0 / diff;
    double offset = min_timestamp.t.to_sec();

    for (auto point : mm.points) {
      points.push_back(to_eigen_point(point));
      double sec = point.t.to_sec();
      sec = (sec + offset) * scale;
      timestamps.push_back(sec);
    }

    const auto &[_, used_points] = kiss_icp_->RegisterFrame(points, timestamps);
    std::vector<evalio::Point> result;
    result.reserve(used_points.size());
    for (auto point : used_points) {
      result.push_back(to_evalio_point(point));
    }
    return result;
  }

private:
  std::unique_ptr<kiss_icp::pipeline::KissICP> kiss_icp_;
  kiss_icp::pipeline::KISSConfig config_;
  Sophus::SE3d lidar_T_imu_;
};
#pragma once

#include <Eigen/Geometry>
#include <memory>
#include <thread>

#include "evalio/pipeline.h"
#include "evalio/types.h"

// mad-icp stuff
#include "odometry/pipeline.h"

// From here
// https://github.com/rvp-group/mad-icp/blob/main/mad_icp/configurations/mad_params.py#L31
struct MadICPConfig {
  bool deskew = false;
  double b_max = 0.2;
  double b_min = 0.1;
  double b_ratio = 0.02;
  double rho_ker = 0.1;
  double p_th = 0.8;
  int num_keyframes = 8;
  int num_threads = 0;
  bool realtime = true;
  // lidar params
  double max_range = 100.0;
  double min_range = 0.0;
  double sensor_hz = 10.0;
};

class MadICP: public evalio::Pipeline {
public:
  MadICP() {}

  // Info
  static std::string version() {
    return XSTR(EVALIO_MAD_ICP);
  }

  static std::string name() {
    return "mad";
  }

  static std::string url() {
    return "https://github.com/rvp-group/mad-icp";
  }

  EVALIO_SETUP_PARAMS(
    (bool, deskew, false, config_.deskew),
    (double, b_max, 0.2, config_.b_max),
    (double, b_min, 0.1, config_.b_min),
    (double, b_ratio, 0.02, config_.b_ratio),
    (double, rho_ker, 0.1, config_.rho_ker),
    (double, p_th, 0.8, config_.p_th),
    (int, num_keyframes, 8, config_.num_keyframes),
    (int, num_threads, 0, config_.num_threads),
    (bool, realtime, true, config_.realtime)
  );

  // Getters
  const std::map<std::string, std::vector<evalio::Point>> map() override {
    auto leaves = mad_icp_->modelLeaves();
    std::vector<evalio::Point> output_points;
    output_points.reserve(leaves.size());
    for (auto point : leaves) {
      output_points.push_back(to_evalio_point(point));
    }

    return {{"planar", output_points}};
  }

  // Setters
  void set_imu_params(evalio::ImuParams params) override {}

  void set_lidar_params(evalio::LidarParams params) override {
    config_.max_range = params.max_range;
    config_.min_range = params.min_range;
    config_.sensor_hz = params.rate;
  }

  void set_imu_T_lidar(evalio::SE3 T) override {
    lidar_T_imu_ = T.inverse();
  }

  // Doers
  void initialize() override {
    if (config_.num_threads <= 0) {
      config_.num_threads = std::min(
        config_.num_keyframes,
        static_cast<int>(std::thread::hardware_concurrency())
      );
    }

    mad_icp_ = std::make_unique<mad::Pipeline>(
      config_.sensor_hz,
      config_.deskew,
      config_.b_max,
      config_.rho_ker,
      config_.p_th,
      config_.b_min,
      config_.b_ratio,
      config_.num_keyframes,
      config_.num_threads,
      config_.realtime
    );
  }

  void add_imu(evalio::ImuMeasurement mm) override {}

  std::map<std::string, std::vector<evalio::Point>>
  add_lidar(evalio::LidarMeasurement mm) override {
    // filter out points that are out of range
    mm.points.erase(
      // remove_if puts the elements to be removed at the end of the vector
      std::remove_if(
        mm.points.begin(),
        mm.points.end(),
        [this](const evalio::Point& point) {
          double range = std::sqrt(
            point.x * point.x + point.y * point.y + point.z * point.z
          );
          return range < config_.min_range || range > config_.max_range;
        }
      ),
      mm.points.end()
    );

    // Copy
    std::vector<Eigen::Vector3d> points;
    points.reserve(mm.points.size());
    for (auto point : mm.points) {
      points.push_back(to_eigen_point(point));
    }

    // Run through pipeline
    mad_icp_->compute(mm.stamp.to_sec(), points);

    // Save the current estimate
    const auto pose = to_evalio_se3(mad_icp_->currentPose()) * lidar_T_imu_;
    this->save_estimate(mm.stamp, pose);

    auto leaves = mad_icp_->currentLeaves();
    std::vector<evalio::Point> output_points;
    output_points.reserve(leaves.size());
    for (const auto& point : leaves) {
      output_points.push_back(to_evalio_point(point));
    }

    return {{"planar", output_points}};
  }

private:
  std::unique_ptr<mad::Pipeline> mad_icp_;
  MadICPConfig config_;
  evalio::SE3 lidar_T_imu_ = evalio::SE3::identity();

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

  inline evalio::SE3 to_evalio_se3(Eigen::Matrix4d pose) {
    const auto iso = Eigen::Isometry3d(pose);
    const auto t = iso.translation();
    const auto q = Eigen::Quaterniond(iso.linear());
    const auto rot =
      evalio::SO3 {.qx = q.x(), .qy = q.y(), .qz = q.z(), .qw = q.w()};
    return evalio::SE3(rot, t);
  }
};

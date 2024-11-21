#pragma once

#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <string>

#include "evalio/pipelines/base.h"
#include "evalio/types.h"
#include "point_types/point_types.hpp"
#include "tclio/imu.hpp"
#include "tclio/tclio.hpp"
#include "timing/timing.hpp"

// ------------------------- Conversions ------------------------- //
inline Eigen::Matrix3d std_to_cov3(double std) {
  return Eigen::Matrix3d::Identity() * std * std;
}

inline Eigen::Matrix<double, 6, 6> std_to_cov6(double std) {
  return Eigen::Matrix<double, 6, 6>::Identity() * std * std;
}

inline evalio::SE3 to_evalio_se3(const gtsam::Pose3 &pose) {
  const auto t = pose.translation();
  const auto q = pose.rotation().toQuaternion();
  const auto rot =
      evalio::SO3{.qx = q.x(), .qy = q.y(), .qz = q.z(), .qw = q.w()};
  return evalio::SE3(rot, t);
}

inline gtsam::Pose3 to_gtsam_se3(const evalio::SE3 &pose) {
  const auto t = pose.trans;
  const auto q = pose.rot.toEigen();
  return gtsam::Pose3(gtsam::Rot3(q), t);
}

inline tclio::Imu to_tclio_imu(const evalio::ImuMeasurement &imu) {
  tclio::timing::Time stamp;
  stamp.setFromNanoseconds(imu.stamp.to_nsec());
  return tclio::Imu{
      .stamp = stamp,
      .gyro = imu.gyro,
      .acc = imu.accel,
  };
}

inline tclio::point_types::PointXYZICD<float>
to_tclio_point(const evalio::Point &point) {
  return tclio::point_types::PointXYZICD<float>{
      .x = static_cast<float>(point.x),
      .y = static_cast<float>(point.y),
      .z = static_cast<float>(point.z),
      ._ = 0.0,
      .intensity = static_cast<short unsigned int>(point.intensity),
      .channel = point.row,
      .timeOffset = tclio::timing::Duration::Nanoseconds(point.t.to_nsec()),
  };
}

inline evalio::Point
to_evalio_pointXYZSL(const tclio::point_types::PointXYZSL<double> &point) {
  return evalio::Point{
      .x = point.x,
      .y = point.y,
      .z = point.z,
      .intensity = 0.0,
      .t = evalio::Stamp::from_nsec(0),
      .range = 0,
      .row = static_cast<uint8_t>(point.line),
      .col = 0,
  };
}

inline evalio::Point
to_evalio_pointXYZICD(const tclio::point_types::PointXYZICD<float> &point,
                      tclio::timing::Time stamp) {
  return evalio::Point{
      .x = point.x,
      .y = point.y,
      .z = point.z,
      .intensity = static_cast<double>(point.intensity),
      .t = evalio::Stamp::from_nsec((stamp + point.timeOffset).toNanoseconds()),
      .range = 0,
      .row = static_cast<uint8_t>(point.channel),
      .col = 0,
  };
}

// ------------------------- Pipeline ------------------------- //
class Tclio : public evalio::Pipeline {
public:
  Tclio() : params_() {}

  // Info
  static std::string name() { return "tclio"; }
  static std::string url() { return "https://github.com/rpl-cmu/tclio"; }
  // TODO: Make this return a map filled with the default values
  static std::map<std::string, evalio::Param> default_params() {
    return {{"keypoint_min_range", 1.0},
            {"keypoint_max_range", 100.0},
            {"keypoint_min_angle_degrees", 20.0},
            {"keypoint_per_line", 50},
            {"keypoint_neighbor_radius", 3.0},
            {"dot_tol", 0.1},
            {"max_dist", 0.1},
            {"voxel_keypoint_width", 0.5}};
  }

  // Getters
  virtual const evalio::SE3 pose() {
    auto odom = pipeline_->getOdometry();
    if (odom.has_value()) {
      return to_evalio_se3(odom.value().pose.pose);
    } else {
      return evalio::SE3::identity();
    }
  };
  virtual const std::vector<evalio::Point> map() {
    auto world_map = pipeline_->create_world_map();
    std::vector<evalio::Point> map;
    map.reserve(world_map.size());
    for (auto &[index, scan_points] : world_map) {
      for (auto &p : scan_points) {
        map.push_back(to_evalio_pointXYZSL(p));
      }
    }
    return map;
  };

  // Setters
  virtual void set_imu_params(evalio::ImuParams params) {
    params_.preintegration->n_gravity = params.gravity;

    // clang-format off
    params_.preintegration->gyroscopeCovariance = std_to_cov3(params.gyro);
    params_.preintegration->accelerometerCovariance = std_to_cov3(params.accel);
    params_.preintegration->biasOmegaCovariance = std_to_cov3(params.gyro_bias);
    params_.preintegration->biasAccCovariance = std_to_cov3(params.accel_bias);
    params_.preintegration->integrationCovariance = std_to_cov3(params.integration);
    params_.preintegration->biasAccOmegaInt = std_to_cov6(params.bias_init);
    // clang-format on
  };
  // TODO: I don't think it even uses any of these?
  virtual void set_lidar_params(evalio::LidarParams params) {};
  virtual void set_imu_T_lidar(evalio::SE3 T) {
    params_.imu_T_lidar = to_gtsam_se3(T);
  };
  // TODO
  virtual void set_params(std::map<std::string, evalio::Param>){};

  // Doers
  virtual void initialize() {
    pipeline_ = std::make_unique<tclio::Estimator>(params_);
  };
  virtual void add_imu(evalio::ImuMeasurement mm) {
    auto result = pipeline_->registerImu(to_tclio_imu(mm));
  };
  virtual std::vector<evalio::Point> add_lidar(evalio::LidarMeasurement mm) {
    tclio::point_types::PointCloud<tclio::point_types::PointXYZICD<float>>
        cloud;
    cloud.points.reserve(mm.points.size());
    for (size_t i = 0; i < mm.points.size(); ++i) {
      cloud.points.push_back(to_tclio_point(mm.points[i]));
    }

    cloud.stamp.setFromNanoseconds(mm.stamp.to_nsec());
    cloud.firstStamp = tclio::timing::Time::max();
    cloud.lastStamp = tclio::timing::Time::min();
    for (const auto &point : cloud.points) {
      const auto tmp = point.timeOffset + cloud.stamp;
      if (tmp < cloud.firstStamp) {
        cloud.firstStamp = tmp;
      } else if (tmp > cloud.lastStamp) {
        cloud.lastStamp = tmp;
      }
    }

    auto result = pipeline_->registerScan(cloud);

    // get most recent scan out
    auto scan = pipeline_->m_scanBuffer.front();
    std::vector<evalio::Point> evalio_scan;
    evalio_scan.reserve(scan.points.size());
    for (const auto &p : scan.points) {
      evalio_scan.push_back(to_evalio_pointXYZICD(p, scan.stamp));
    }

    return evalio_scan;
  };

private:
  std::unique_ptr<tclio::Estimator> pipeline_;
  tclio::Estimator::Params params_;
};
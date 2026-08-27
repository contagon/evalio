#pragma once

#include <pcl/point_cloud.h>

#include <iostream>
#include <map>
#include <string>

#include "dlio/odom.h"
#include "evalio/convert/base.h"
#include "evalio/pipeline.h"
#include "evalio/types.h"

namespace ev = evalio;

// ------------------------- Fill out some converters for custom types ------------------------- //
namespace evalio {
// Point conversions
template<>
inline Point convert(const dlio::Point& in) {
  return Point {
    .x = in.x,
    .y = in.y,
    .z = in.z,
    .intensity = in.intensity,
    .t = ev::Duration::from_nsec(in.t)
  };
}

template<>
inline dlio::Point convert(const ev::Point& in) {
  dlio::Point out;
  out.x = static_cast<float>(in.x);
  out.y = static_cast<float>(in.y);
  out.z = static_cast<float>(in.z);
  out.intensity = static_cast<float>(in.intensity);
  out.t = static_cast<uint32_t>(in.t.to_nsec());
  return out;
}

// IMU conversions
template<>
inline dlio::OdomNode::ImuMeas convert(const ev::ImuMeasurement& in) {
  return dlio::OdomNode::ImuMeas {
    .stamp = in.stamp.to_sec(),
    .dt = 0.0, // Will be computed by DLIO internally
    .ang_vel = in.gyro.cast<float>(),
    .lin_accel = in.accel.cast<float>()
  };
}

// SE3 conversion from DLIO State
template<>
inline ev::SE3 convert(const dlio::OdomNode::State& in) {
  const auto& q = in.q;
  const auto rot = ev::SO3 {.qx = q.x(), .qy = q.y(), .qz = q.z(), .qw = q.w()};
  return ev::SE3(rot, in.p.cast<double>());
}

} // namespace evalio

// ------------------------- The pipeline impl ------------------------- //
class DLIO: public ev::Pipeline {
public:
  DLIO() : config_(), lidar_T_imu_(ev::SE3::identity()) {}

  // Info
  static std::string version() {
    return XSTR(EVALIO_DLIO);
  }

  static std::string name() {
    return "dlio";
  }

  static std::string url() {
    return "https://github.com/vectr-ucla/direct_lidar_inertial_odometry";
  }

  // Defaults as found in cfg/params.yaml in the DLIO repo
  // clang-format off
  EVALIO_SETUP_PARAMS(
    (bool, verbose, false, config_.verbose),

    (bool, deskew, true, config_.deskew),
    (double, gravity, 9.80665, config_.gravity),
    (bool, time_offset, true, config_.time_offset),

    (double, keyframe_thresh_dist, 1.0, config_.keyframe_thresh_dist),
    (double, keyframe_thresh_rot, 45.0, config_.keyframe_thresh_rot),

    (int, submap_knn, 10, config_.submap_knn),
    (int, submap_kcv, 10, config_.submap_kcv),
    (int, submap_kcc, 10, config_.submap_kcc),

    (bool, densemap_filtered, false, config_.densemap_filtered),
    (bool, wait_until_move, true, config_.wait_until_move),

    (double, crop_size, 1.0, config_.crop_size),

    (bool, vf_use, true, config_.vf_use),
    (double, vf_res, 0.25, config_.vf_res),

    (bool, adaptive_params, true, config_.adaptive_params),

    (bool, calibrate_gyro, true, config_.calibrate_gyro),
    (bool, calibrate_accel, true, config_.calibrate_accel),
    (double, imu_calib_time, 3.0, config_.imu_calib_time),
    (int, imu_buffer_size, 5000, config_.imu_buffer_size),

    (bool, gravity_align, true, config_.gravity_align),
    (bool, imu_calibrate, true, config_.imu_calibrate),

    (int, gicp_min_num_points, 64, config_.gicp_min_num_points),
    (int, gicp_k_correspondences, 16, config_.gicp_k_correspondences),
    (double, gicp_max_corr_dist, 0.5, config_.gicp_max_corr_dist),
    (int, gicp_max_iter, 32, config_.gicp_max_iter),
    (double, gicp_transformation_ep, 0.01, config_.gicp_transformation_ep),
    (double, gicp_rotation_ep, 0.01, config_.gicp_rotation_ep),
    (double, gicp_init_lambda_factor, 1e-9, config_.gicp_init_lambda_factor),

    (double, geo_Kp, 4.5, config_.geo_Kp),
    (double, geo_Kv, 11.25, config_.geo_Kv),
    (double, geo_Kq, 4.0, config_.geo_Kq),
    (double, geo_Kab, 2.25, config_.geo_Kab),
    (double, geo_Kgb, 1.0, config_.geo_Kgb),
    (double, geo_abias_max, 5.0, config_.geo_abias_max),
    (double, geo_gbias_max, 0.5, config_.geo_gbias_max)
  );
  // clang-format on

  // Getters
  const std::map<std::string, std::vector<ev::Point>> map() override {
    return ev::make_map("point", *dlio_->getMap());
  }

  // Setters
  void set_imu_params(ev::ImuParams params) override {
    config_.gravity = params.gravity.norm();
  }

  void set_lidar_params(ev::LidarParams params) override {
    // DLIO doesn't have explicit lidar params in its config
  }

  void set_imu_T_lidar(ev::SE3 T) override {
    // DLIO uses baselink2imu and baselink2lidar extrinsics
    // Assuming baselink = imu frame
    config_.extrinsics_baselink2lidar_t = T.trans.cast<float>();
    config_.extrinsics_baselink2lidar_R = T.rot.to_mat().cast<float>();
    // baselink2lidar is identity since baselink = lidar
    config_.extrinsics_baselink2imu_t = Eigen::Vector3f::Zero();
    config_.extrinsics_baselink2imu_R = Eigen::Matrix3f::Identity();
  }

  // Doers
  void initialize() override {
    dlio_ = std::make_unique<dlio::OdomNode>(config_);
    dlio_->start();
  }

  void add_imu(ev::ImuMeasurement mm) override {
    dlio_->callbackImu(ev::convert<dlio::OdomNode::ImuMeas>(mm));
  }

  void add_lidar(ev::LidarMeasurement mm) override {
    // Convert to PCL point cloud
    auto cloud =
      ev::convert_iter<pcl::PointCloud<dlio::Point>>(mm.points).makeShared();

    // Run through pipeline
    dlio_->callbackPointCloud(cloud, mm.stamp.to_sec());

    // Get pose from DLIO state and save
    const auto state = dlio_->getState();
    // Results are in the baselink = imu frame
    this->save(mm.stamp, state);

    // Save the current cloud used by DLIO
    this->save(mm.stamp, "point", *dlio_->getCurrentScan());
  }

private:
  std::unique_ptr<dlio::OdomNode> dlio_;
  dlio::OdomNode::Params config_;
  ev::SE3 lidar_T_imu_;
};

#pragma once

#include <pcl/point_cloud.h>

#include <map>
#include <string>

#include "LIO-SAM/lio-sam.h"
#include "LIO-SAM/types.h"
#include "evalio/convert/base.h"
#include "evalio/pipeline.h"
#include "evalio/types.h"

namespace ev = evalio;

// ------------------------- Fill out some converters for custom types ------------------------- //
namespace evalio {
// Point conversions
template<>
inline Point convert(const lio_sam::PointXYZIRT& in) {
  return Point {
    .x = in.x,
    .y = in.y,
    .z = in.z,
    .intensity = in.intensity,
    .t = ev::Duration::from_sec(in.time),
    .row = static_cast<uint8_t>(in.ring)
  };
}

template<>
inline Point convert(const lio_sam::PointType& in) {
  return Point {.x = in.x, .y = in.y, .z = in.z, .intensity = in.intensity};
}

template<>
inline lio_sam::PointXYZIRT convert(const ev::Point& in) {
  return lio_sam::PointXYZIRT {
    .x = static_cast<float>(in.x),
    .y = static_cast<float>(in.y),
    .z = static_cast<float>(in.z),
    .intensity = static_cast<float>(in.intensity),
    .ring = static_cast<uint16_t>(in.row),
    .time = static_cast<float>(in.t.to_sec()),
  };
}

// IMU conversions
template<>
inline lio_sam::Imu convert(const ev::ImuMeasurement& in) {
  return lio_sam::Imu {
    .stamp = in.stamp.to_sec(),
    .gyro = in.gyro,
    .acc = in.accel
  };
}

// SE3 conversions
template<>
inline ev::SE3 convert(const lio_sam::Odometry& in) {
  const auto t = in.position;
  const auto q = in.orientation;
  const auto rot = ev::SO3 {.qx = q.x(), .qy = q.y(), .qz = q.z(), .qw = q.w()};
  return ev::SE3(rot, t);
}
} // namespace evalio

// ------------------------- The pipeline impl ------------------------- //
class LioSam: public ev::Pipeline {
public:
  LioSam() : config_(), lidar_T_imu_(ev::SE3::identity()) {}

  // Info
  static std::string version() {
    return XSTR(EVALIO_LIO_SAM);
  }

  static std::string name() {
    return "liosam";
  }

  static std::string url() {
    return "https://github.com/contagon/LIO-SAM";
  }

  // clang-format off
  EVALIO_SETUP_PARAMS(
    (int, downsampleRate, 1, config_.downsampleRate),
    (double, edgeThreshold, 1.0, config_.edgeThreshold),
    (double, surfThreshold, 0.1, config_.surfThreshold),
    (int, edgeFeatureMinValidNum, 10, config_.edgeFeatureMinValidNum),
    (int, surfFeatureMinValidNum, 100, config_.surfFeatureMinValidNum),

    // voxel filter paprams
    (double, odometrySurfLeafSize, 0.4, config_.odometrySurfLeafSize),
    (double, mappingCornerLeafSize, 0.2, config_.mappingCornerLeafSize),
    (double, mappingSurfLeafSize, 0.4, config_.mappingSurfLeafSize),

    (double, z_tolerance, 1000.0, config_.z_tolerance),
    (double, rotation_tolerance, 1000.0, config_.rotation_tolerance),

    // CPU Params
    (int, numberOfCores, 4, config_.numberOfCores),
    (double, mappingProcessInterval, 0.15, config_.mappingProcessInterval),

    // Surrounding map
    (double, surroundingkeyframeAddingDistThreshold, 1.0, config_.surroundingkeyframeAddingDistThreshold),
    (double, surroundingkeyframeAddingAngleThreshold, 0.2, config_.surroundingkeyframeAddingAngleThreshold),
    (double, surroundingKeyframeDensity, 2.0, config_.surroundingKeyframeDensity),
    (double, surroundingKeyframeSearchRadius, 50.0, config_.surroundingKeyframeSearchRadius),

    // global map visualization radius
    (double, globalMapVisualizationSearchRadius, 1000.0, config_.globalMapVisualizationSearchRadius),
    (double, globalMapVisualizationPoseDensity, 10.0, config_.globalMapVisualizationPoseDensity),
    (double, globalMapVisualizationLeafSize, 1.0, config_.globalMapVisualizationLeafSize)
  );
  // clang-format on

  // Getters
  const ev::SE3 pose() override {
    return ev::convert<ev::SE3>(lio_sam_->getPose()) * lidar_T_imu_;
  }

  const std::map<std::string, std::vector<ev::Point>> map() override {
    return ev::convert_map<pcl::PointCloud<lio_sam::PointType>>(
      {{"point", *lio_sam_->getMap()}}
    );
  }

  // Setters
  void set_imu_params(ev::ImuParams params) override {
    config_.imuAccNoise = params.accel;
    config_.imuAccBiasN = params.accel_bias;
    config_.imuGyrNoise = params.gyro;
    config_.imuGyrBiasN = params.gyro_bias;
    config_.imuGravity = params.gravity[2];
  }

  void set_lidar_params(ev::LidarParams params) override {
    config_.N_SCAN = params.num_rows;
    config_.Horizon_SCAN = params.num_columns;
    config_.lidarMaxRange = params.max_range;
    config_.lidarMinRange = params.min_range;
  }

  void set_imu_T_lidar(ev::SE3 T) override {
    lidar_T_imu_ = T.inverse();
    config_.lidar_P_imu = lidar_T_imu_.trans;
    config_.lidar_R_imu = lidar_T_imu_.rot.to_eigen();
  }

  // Doers
  void initialize() override {
    lio_sam_ = std::make_unique<lio_sam::LIOSAM>(config_);
  }

  void add_imu(ev::ImuMeasurement mm) override {
    lio_sam_->addImuMeasurement(ev::convert<lio_sam::Imu>(mm));
  }

  std::map<std::string, std::vector<ev::Point>>
  add_lidar(ev::LidarMeasurement mm) override {
    // Set everything up
    auto cloud =
      ev::convert_iter<pcl::PointCloud<lio_sam::PointXYZIRT>>(mm.points)
        // NOTE: This likely causes a copy, see if we can avoid that later
        .makeShared();

    // Run through pipeline
    lio_sam_->addLidarMeasurement(mm.stamp.to_sec(), cloud);

    // Return features
    return ev::convert_map<pcl::PointCloud<lio_sam::PointType>>(
      {{"point", *lio_sam_->getMostRecentFrame()}}
    );
  }

private:
  std::unique_ptr<lio_sam::LIOSAM> lio_sam_;
  lio_sam::LioSamParams config_;
  ev::SE3 lidar_T_imu_;
};

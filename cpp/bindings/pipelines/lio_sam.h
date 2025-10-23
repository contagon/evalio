#pragma once

#include <pcl/point_cloud.h>

#include <map>
#include <string>

#include "LIO-SAM/lio-sam.h"
#include "LIO-SAM/types.h"
#include "evalio/pipeline.h"
#include "evalio/types.h"

inline void
to_evalio_point(evalio::Point& ev_point, const lio_sam::PointXYZIRT& ls_point) {
  ev_point.x = ls_point.x;
  ev_point.y = ls_point.y;
  ev_point.z = ls_point.z;
  ev_point.intensity = ls_point.intensity;
  ev_point.t = evalio::Duration::from_sec(ls_point.time);
  ev_point.row = ls_point.ring;
}

inline void
to_evalio_point(evalio::Point& ev_point, const lio_sam::PointType& ls_point) {
  ev_point.x = ls_point.x;
  ev_point.y = ls_point.y;
  ev_point.z = ls_point.z;
  ev_point.intensity = ls_point.intensity;
}

inline void
to_pcl_point(lio_sam::PointXYZIRT& ls_point, const evalio::Point& ev_point) {
  ls_point.x = ev_point.x;
  ls_point.y = ev_point.y;
  ls_point.z = ev_point.z;
  ls_point.intensity = ev_point.intensity;
  ls_point.time = ev_point.t.to_sec();
  ls_point.ring = ev_point.row;
}

inline evalio::SE3 to_evalio_se3(lio_sam::Odometry pose) {
  const auto t = pose.position;
  const auto q = pose.orientation;
  const auto rot =
    evalio::SO3 {.qx = q.x(), .qy = q.y(), .qz = q.z(), .qw = q.w()};
  return evalio::SE3(rot, t);
}

class LioSam: public evalio::Pipeline {
public:
  LioSam() : config_(), lidar_T_imu_(evalio::SE3::identity()) {}

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
  const std::map<std::string, std::vector<evalio::Point>> map() override {
    auto map = lio_sam_->getMap();
    std::vector<evalio::Point> evalio_map(map->size());
    for (std::size_t i = 0; i < map->size(); ++i) {
      to_evalio_point(evalio_map[i], map->at(i));
    }
    return {{"point", evalio_map}};
  }

  // Setters
  void set_imu_params(evalio::ImuParams params) override {
    config_.imuAccNoise = params.accel;
    config_.imuAccBiasN = params.accel_bias;
    config_.imuGyrNoise = params.gyro;
    config_.imuGyrBiasN = params.gyro_bias;
    config_.imuGravity = params.gravity[2];
  }

  void set_lidar_params(evalio::LidarParams params) override {
    config_.N_SCAN = params.num_rows;
    config_.Horizon_SCAN = params.num_columns;
    config_.lidarMaxRange = params.max_range;
    config_.lidarMinRange = params.min_range;
  }

  void set_imu_T_lidar(evalio::SE3 T) override {
    lidar_T_imu_ = T.inverse();
    config_.lidar_P_imu = lidar_T_imu_.trans;
    config_.lidar_R_imu = lidar_T_imu_.rot.toEigen();
  }

  // Doers
  void initialize() override {
    lio_sam_ = std::make_unique<lio_sam::LIOSAM>(config_);
  }

  void add_imu(evalio::ImuMeasurement mm) override {
    lio_sam::Imu imuMsg {
      .stamp = mm.stamp.to_sec(),
      .gyro = mm.gyro,
      .acc = mm.accel
    };
    lio_sam_->addImuMeasurement(imuMsg);
  }

  std::map<std::string, std::vector<evalio::Point>>
  add_lidar(evalio::LidarMeasurement mm) override {
    // Set everything up
    pcl::PointCloud<lio_sam::PointXYZIRT>::Ptr cloud;
    cloud.reset(new pcl::PointCloud<lio_sam::PointXYZIRT>);
    cloud->points.resize(mm.points.size());

    // Convert to pcl
    for (size_t i = 0; i < mm.points.size(); ++i) {
      to_pcl_point(cloud->points[i], mm.points[i]);
    }

    // Run through pipeline
    lio_sam_->addLidarMeasurement(mm.stamp.to_sec(), cloud);

    // Save pose
    const auto pose = to_evalio_se3(lio_sam_->getPose()) * lidar_T_imu_;
    this->push_back_estimate(mm.stamp, pose);

    // Return features
    auto used_points = lio_sam_->getMostRecentFrame();
    std::vector<evalio::Point> result(used_points->points.size());
    for (size_t i = 0; i < used_points->points.size(); ++i) {
      to_evalio_point(result[i], used_points->points[i]);
    }
    return {{"point", result}};
  }

private:
  std::unique_ptr<lio_sam::LIOSAM> lio_sam_;
  lio_sam::LioSamParams config_;
  evalio::SE3 lidar_T_imu_;
};

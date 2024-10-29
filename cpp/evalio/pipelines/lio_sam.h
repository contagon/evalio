#pragma once

#include <map>
#include <pcl/point_cloud.h>
#include <stdexcept>
#include <string>

#include "LIO-SAM/lio-sam.h"
#include "LIO-SAM/types.h"
#include "evalio/pipelines/base.h"
#include "evalio/types.h"

inline void to_evalio_point(evalio::Point &ev_point,
                            const lio_sam::PointXYZIRT &ls_point) {
  ev_point.x = ls_point.x;
  ev_point.y = ls_point.y;
  ev_point.z = ls_point.z;
  ev_point.intensity = ls_point.intensity;
  ev_point.t = evalio::Stamp::from_sec(ls_point.time);
  ev_point.row = ls_point.ring;
}

inline void to_evalio_point(evalio::Point &ev_point,
                            const lio_sam::PointType &ls_point) {
  ev_point.x = ls_point.x;
  ev_point.y = ls_point.y;
  ev_point.z = ls_point.z;
  ev_point.intensity = ls_point.intensity;
}

inline void to_pcl_point(lio_sam::PointXYZIRT &ls_point,
                         const evalio::Point &ev_point) {
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
      evalio::SO3{.qx = q.x(), .qy = q.y(), .qz = q.z(), .qw = q.w()};
  return evalio::SE3(rot, t);
}

class LioSam : public evalio::Pipeline {
public:
  LioSam() : config_(), lidar_T_imu_(evalio::SE3::identity()){};

  // Info
  static std::string name() { return "kiss"; }
  static std::string url() { return "https://github.com/contagon/LIO-SAM"; }
  static std::map<std::string, evalio::Param> default_params() {
    return {
        {"edgeThreshold", 1.0},
        {"surfThreshold", 0.1},
        {"edgeFeatureMinValidNum", 10},
        {"surfFeatureMinValidNum", 100},

        // voxel filter paprams
        {"odometrySurfLeafSize", 0.4},
        {"mappingCornerLeafSize", 0.2},
        {"mappingSurfLeafSize", 0.4},

        {"z_tollerance", 1000},
        {"rotation_tollerance", 1000},

        // CPU Params
        {"numberOfCores", 4},
        {"mappingProcessInterval", 0.15},

        // Surrounding map
        {"surroundingkeyframeAddingDistThreshold", 1.0},
        {"surroundingkeyframeAddingAngleThreshold", 0.2},
        {"surroundingKeyframeDensity", 2.0},
        {"surroundingKeyframeSearchRadius", 50.0},

        // global map visualization radius
        {"globalMapVisualizationSearchRadius", 1000.0},
        {"globalMapVisualizationPoseDensity", 10.0},
        {"globalMapVisualizationLeafSize", 1.0},
    };
  }

  // Getters
  const evalio::SE3 pose() override {
    return to_evalio_se3(lio_sam_->getPose()) * lidar_T_imu_;
  }

  const std::vector<evalio::Point> map() override {
    auto map = lio_sam_->getMap();
    std::vector<evalio::Point> evalio_map(map->size());
    for (std::size_t i = 0; i < map->size(); ++i) {
      to_evalio_point(evalio_map[i], map->at(i));
    }
    return evalio_map;
  }

  // Setters
  void set_imu_params(evalio::ImuParams params) override {};
  void set_lidar_params(evalio::LidarParams params) override {};
  void set_imu_T_lidar(evalio::SE3 T) override { lidar_T_imu_ = T.inverse(); };

  void set_params(std::map<std::string, evalio::Param> params) override {
    for (auto &[key, value] : params) {
      if (std::holds_alternative<bool>(value)) {
        throw std::invalid_argument(
            "Invalid parameter, KissICP doesn't have bool param " + key);
      } else if (std::holds_alternative<int>(value)) {
        if (key == "edgeFeatureMinValidNum") {
          config_.edgeFeatureMinValidNum = std::get<int>(value);
        } else if (key == "surfFeatureMinValidNum") {
          config_.surfFeatureMinValidNum = std::get<int>(value);
        } else if (key == "numberOfCores") {
          config_.numberOfCores = std::get<int>(value);
        } else {
          throw std::invalid_argument(
              "Invalid parameter, KissICP doesn't have int param " + key);
        }
      } else if (std::holds_alternative<double>(value)) {
        if (key == "edgeThreshold") {
          config_.edgeThreshold = std::get<double>(value);
        } else if (key == "surfThreshold") {
          config_.surfThreshold = std::get<double>(value);
        } else if (key == "odometrySurfLeafSize") {
          config_.odometrySurfLeafSize = std::get<double>(value);
        } else if (key == "mappingCornerLeafSize") {
          config_.mappingCornerLeafSize = std::get<double>(value);
        } else if (key == "mappingSurfLeafSize") {
          config_.mappingSurfLeafSize = std::get<double>(value);
        } else if (key == "z_tollerance") {
          config_.z_tollerance = std::get<double>(value);
        } else if (key == "rotation_tollerance") {
          config_.rotation_tollerance = std::get<double>(value);
        } else if (key == "surroundingkeyframeAddingDistThreshold") {
          config_.surroundingkeyframeAddingDistThreshold =
              std::get<double>(value);
        } else if (key == "surroundingkeyframeAddingAngleThreshold") {
          config_.surroundingkeyframeAddingAngleThreshold =
              std::get<double>(value);
        } else if (key == "surroundingKeyframeDensity") {
          config_.surroundingKeyframeDensity = std::get<double>(value);
        } else if (key == "surroundingKeyframeSearchRadius") {
          config_.surroundingKeyframeSearchRadius = std::get<double>(value);
        } else if (key == "globalMapVisualizationSearchRadius") {
          config_.globalMapVisualizationSearchRadius = std::get<double>(value);
        } else if (key == "globalMapVisualizationPoseDensity") {
          config_.globalMapVisualizationPoseDensity = std::get<double>(value);
        } else if (key == "globalMapVisualizationLeafSize") {
          config_.globalMapVisualizationLeafSize = std::get<double>(value);
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
    lio_sam_ = std::make_unique<lio_sam::LIOSAM>(config_);
  }

  void add_imu(evalio::ImuMeasurement mm) override {};

  std::vector<evalio::Point> add_lidar(evalio::LidarMeasurement mm) override {
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

    // Return features
    auto used_points = lio_sam_->getMostRecentFrame();
    std::vector<evalio::Point> result(used_points->points.size());
    for (size_t i = 0; i < used_points->points.size(); ++i) {
      to_evalio_point(result[i], used_points->points[i]);
    }
    return result;
  }

private:
  std::unique_ptr<lio_sam::LIOSAM> lio_sam_;
  lio_sam::LioSamParams config_;
  evalio::SE3 lidar_T_imu_;
};
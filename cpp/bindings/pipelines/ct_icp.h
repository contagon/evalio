#pragma once

#include <memory>

#include "SlamCore/pointcloud.h"
#include "SlamCore/types.h"
#include "ct_icp/map.h"
#include "ct_icp/neighborhood_strategy.h"
#include "ct_icp/odometry.h"
#include "evalio/pipeline.h"
#include "evalio/types.h"

class CTICP: public evalio::Pipeline {
private:
  std::unique_ptr<ct_icp::Odometry> ct_icp_;
  ct_icp::OdometryOptions config_ =
    ct_icp::OdometryOptions::DefaultRobustOutdoorLowInertia();
  evalio::SE3 lidar_T_imu_ = evalio::SE3::identity();
  size_t scan_idx_ = 0;

  inline evalio::SE3 to_evalio_pose(const slam::SE3& pose) const {
    return evalio::SE3(evalio::SO3::fromEigen(pose.quat), pose.tr);
  }

  inline evalio::Point to_evalio_point(const slam::Point3D& point) const {
    return evalio::Point {
      .x = point.point.x(),
      .y = point.point.y(),
      .z = point.point.z(),
      .intensity = 0.0,
      .t = evalio::Duration::from_sec(point.timestamp),
      .row = 0,
      .col = 0,
    };
  }

public:
  CTICP() : config_() {
    config_.debug_print = false;
    config_.ct_icp_options.debug_print = false;
    config_.ct_icp_options.output_weights = false;

    auto neighbor_options = ct_icp::DefaultNearestNeighborStrategy::Options();
    neighbor_options.max_num_neighbors = 20;
    neighbor_options.min_num_neighbors = 10;
    config_.neighborhood_strategy =
      std::make_shared<ct_icp::DefaultNearestNeighborStrategy::Options>(
        neighbor_options
      );

    using ct_icp::MultipleResolutionVoxelMap;
    auto map_options = MultipleResolutionVoxelMap::Options();
    map_options.resolutions = {
      MultipleResolutionVoxelMap::ResolutionParam {0.5, 0.05, 30},
      MultipleResolutionVoxelMap::ResolutionParam {1.0, 0.1, 30},
      MultipleResolutionVoxelMap::ResolutionParam {2.0, 0.2, 30}
    };
    config_.map_options =
      std::make_shared<MultipleResolutionVoxelMap::Options>(map_options);
  }

  // Info
  static std::string version() {
    return XSTR(EVALIO_CT_ICP);
  }

  static std::string name() {
    return "ct";
  }

  static std::string url() {
    return "https://github.com/jedeschaud/ct_icp";
  }

  static std::map<std::string, evalio::Param> default_params() {
    // Pull all of the defaults parameters for nclt_config.yaml
    // https://github.com/jedeschaud/ct_icp/blob/master/config/odometry/nclt_config.yaml
    return {
      // odometry options
      {"motion_compensation", std::string("CONTINUOUS")},
      {"initialization", std::string("INIT_CONSTANT_VELOCITY")},
      {"sample_voxel_size", 0.8},
      {"sampling", std::string("ADAPTIVE")},
      {"voxel_size", 0.5},
      {"max_distance", 100.0},
      {"distance_error_threshold", 5.0},
      {"max_num_keypoints", 1500},
      {"size_voxel_map", 1.0},
      {"voxel_neighborhood", 1},
      {"max_num_points_in_voxel", 20},
      {"min_distance_points", 0.1},
      // ct-icp options
      {"num_iters_icp", 20},
      {"parametrization", std::string("CONTINUOUS_TIME")},
      {"distance", std::string("POINT_TO_PLANE")},
      {"max_num_residuals", 1500},
      {"min_num_residuals", 100},
      {"weighting_scheme", std::string("ALL")},
      {"weight_alpha", 0.9},
      {"weight_neighborhood", 0.1},
      {"min_number_neighbors", 10},
      {"max_number_neighbors", 20},
      {"num_closest_neighbors", 1},
      {"power_planarity", 2},
      {"threshold_voxel_occupancy", 1},
      {"threshold_orientation_norm", 0.1},
      {"threshold_translation_norm", 0.01},
      {"point_to_plane_with_distortion", true},
      // ceres solver options
      {"loss_function", std::string("CAUCHY")},
      {"ls_max_num_iters", 10},
      {"ls_num_threads", 6},
      {"ls_sigma", 0.1},
      {"ls_tolerant_min_threshold", 0.05},
    };
  }

  // Getters
  const evalio::SE3 pose() override {
    const auto pose = ct_icp_->Trajectory().back().begin_pose.pose;
    return to_evalio_pose(pose) * lidar_T_imu_;
  }

  const std::map<std::string, std::vector<evalio::Point>> map() override {
    return {};
  }

  // Setters
  void set_imu_params(evalio::ImuParams params) override {}

  void set_lidar_params(evalio::LidarParams params) override {
    config_.max_distance = params.max_range;
  }

  void set_imu_T_lidar(evalio::SE3 T) override {
    lidar_T_imu_ = T.inverse();
  }

  void set_params(std::map<std::string, evalio::Param> params) override {
    for (auto& [key, value] : params) {
      // odometry options
      if (key == "motion_compensation") {
        const std::string& mc_str = std::get<std::string>(value);
        if (mc_str == "NONE") {
          config_.motion_compensation = ct_icp::MOTION_COMPENSATION::NONE;
        } else if (mc_str == "CONSTANT_VELOCITY") {
          config_.motion_compensation =
            ct_icp::MOTION_COMPENSATION::CONSTANT_VELOCITY;
        } else if (mc_str == "ITERATIVE") {
          config_.motion_compensation = ct_icp::MOTION_COMPENSATION::ITERATIVE;
        } else if (mc_str == "CONTINUOUS") {
          config_.motion_compensation = ct_icp::MOTION_COMPENSATION::CONTINUOUS;
        }
      } else if (key == "initialization") {
        const std::string& init_str = std::get<std::string>(value);
        if (init_str == "INIT_NONE") {
          config_.initialization = ct_icp::INITIALIZATION::INIT_NONE;
        } else if (init_str == "INIT_CONSTANT_VELOCITY") {
          config_.initialization =
            ct_icp::INITIALIZATION::INIT_CONSTANT_VELOCITY;
        }
      } else if (key == "sample_voxel_size") {
        config_.sample_voxel_size = std::get<double>(value);
      } else if (key == "sampling") {
        const std::string& sampling_str = std::get<std::string>(value);
        if (sampling_str == "NONE") {
          config_.sampling = ct_icp::sampling::NONE;
        } else if (sampling_str == "GRID") {
          config_.sampling = ct_icp::sampling::GRID;
        } else if (sampling_str == "ADAPTIVE") {
          config_.sampling = ct_icp::sampling::ADAPTIVE;
        }
      } else if (key == "voxel_size") {
        config_.voxel_size = std::get<double>(value);
      } else if (key == "max_distance") {
        config_.max_distance = std::get<double>(value);
      } else if (key == "distance_error_threshold") {
        config_.distance_error_threshold = std::get<double>(value);
      } else if (key == "max_num_keypoints") {
        config_.max_num_keypoints = std::get<int>(value);
      } else if (key == "size_voxel_map") {
        config_.size_voxel_map = std::get<double>(value);
      } else if (key == "voxel_neighborhood") {
        config_.voxel_neighborhood = std::get<int>(value);
      } else if (key == "max_num_points_in_voxel") {
        config_.max_num_points_in_voxel = std::get<int>(value);
      } else if (key == "min_distance_points") {
        config_.min_distance_points = std::get<double>(value);
      }

      // ct-icp options
      if (key == "num_iters_icp") {
        config_.ct_icp_options.num_iters_icp = std::get<int>(value);
      } else if (key == "parametrization") {
        const std::string& param_str = std::get<std::string>(value);
        if (param_str == "CONTINUOUS_TIME") {
          config_.ct_icp_options.parametrization = ct_icp::CONTINUOUS_TIME;
        } else if (param_str == "SIMPLE") {
          config_.ct_icp_options.parametrization = ct_icp::SIMPLE;
        }
      } else if (key == "distance") {
        const std::string& dist_str = std::get<std::string>(value);
        if (dist_str == "POINT_TO_PLANE") {
          config_.ct_icp_options.distance = ct_icp::POINT_TO_PLANE;
        } else if (dist_str == "POINT_TO_POINT") {
          config_.ct_icp_options.distance = ct_icp::POINT_TO_POINT;
        } else if (dist_str == "POINT_TO_LINE") {
          config_.ct_icp_options.distance = ct_icp::POINT_TO_LINE;
        } else if (dist_str == "POINT_TO_DISTRIBUTION") {
          config_.ct_icp_options.distance = ct_icp::POINT_TO_DISTRIBUTION;
        }
      } else if (key == "max_num_residuals") {
        config_.ct_icp_options.max_num_residuals = std::get<int>(value);
      } else if (key == "min_num_residuals") {
        config_.ct_icp_options.min_num_residuals = std::get<int>(value);
      } else if (key == "weighting_scheme") {
        const std::string& ws_str = std::get<std::string>(value);
        if (ws_str == "PLANARITY") {
          config_.ct_icp_options.weighting_scheme = ct_icp::PLANARITY;
        } else if (ws_str == "NEIGHBORHOOD") {
          config_.ct_icp_options.weighting_scheme = ct_icp::NEIGHBORHOOD;
        } else if (ws_str == "ALL") {
          config_.ct_icp_options.weighting_scheme = ct_icp::ALL;
        }
      } else if (key == "weight_alpha") {
        config_.ct_icp_options.weight_alpha = std::get<double>(value);
      } else if (key == "weight_neighborhood") {
        config_.ct_icp_options.weight_neighborhood = std::get<double>(value);
      } else if (key == "min_number_neighbors") {
        config_.ct_icp_options.min_number_neighbors = std::get<int>(value);
      } else if (key == "max_number_neighbors") {
        config_.ct_icp_options.max_number_neighbors = std::get<int>(value);
      } else if (key == "num_closest_neighbors") {
        config_.ct_icp_options.num_closest_neighbors = std::get<int>(value);
      } else if (key == "power_planarity") {
        config_.ct_icp_options.power_planarity = std::get<int>(value);
      } else if (key == "threshold_voxel_occupancy") {
        config_.ct_icp_options.threshold_voxel_occupancy = std::get<int>(value);
      } else if (key == "threshold_orientation_norm") {
        config_.ct_icp_options.threshold_orientation_norm =
          std::get<double>(value);
      } else if (key == "threshold_translation_norm") {
        config_.ct_icp_options.threshold_translation_norm =
          std::get<double>(value);
      } else if (key == "point_to_plane_with_distortion") {
        config_.ct_icp_options.point_to_plane_with_distortion =
          std::get<bool>(value);
      }

      // ceres solver options
      else if (key == "loss_function") {
        const std::string& loss_str = std::get<std::string>(value);
        if (loss_str == "STANDARD") {
          config_.ct_icp_options.loss_function =
            ct_icp::LEAST_SQUARES::STANDARD;
        } else if (loss_str == "CAUCHY") {
          config_.ct_icp_options.loss_function = ct_icp::LEAST_SQUARES::CAUCHY;
        } else if (loss_str == "HUBER") {
          config_.ct_icp_options.loss_function = ct_icp::LEAST_SQUARES::HUBER;
        } else if (loss_str == "TOLERANT") {
          config_.ct_icp_options.loss_function =
            ct_icp::LEAST_SQUARES::TOLERANT;
        } else if (loss_str == "TRUNCATED") {
          config_.ct_icp_options.loss_function =
            ct_icp::LEAST_SQUARES::TRUNCATED;
        }
      } else if (key == "ls_max_num_iters") {
        config_.ct_icp_options.ls_max_num_iters = std::get<int>(value);
      } else if (key == "ls_num_threads") {
        config_.ct_icp_options.ls_num_threads = std::get<int>(value);
      } else if (key == "ls_sigma") {
        config_.ct_icp_options.ls_sigma = std::get<double>(value);
      } else if (key == "ls_tolerant_min_threshold") {
        config_.ct_icp_options.ls_tolerant_min_threshold =
          std::get<double>(value);
      }
    }
  }

  // Doers
  void initialize() override {
    config_.sample_voxel_size = 0.8;
    config_.sampling = ct_icp::sampling::ADAPTIVE;
    ct_icp_ = std::make_unique<ct_icp::Odometry>(config_);
  }

  void add_imu(evalio::ImuMeasurement mm) override {}

  std::map<std::string, std::vector<evalio::Point>>
  add_lidar(evalio::LidarMeasurement mm) override {
    // Set everything up
    auto pc = slam::PointCloud::DefaultXYZ<double>();
    pc.resize(mm.points.size());
    pc.AddDefaultTimestampsField();
    auto xyz = pc.XYZ<double>();
    auto timestamps = pc.TimestampsProxy<double>();

    // Copy
    for (size_t idx = 0; idx < mm.points.size(); ++idx) {
      const auto& point = mm.points[idx];
      xyz[idx] = Eigen::Vector3d(point.x, point.y, point.z);
      timestamps[idx] = point.t.to_sec();
    }

    // Run through pipeline
    const auto summary = ct_icp_->RegisterFrame(pc, scan_idx_);

    // Return the used points
    std::vector<evalio::Point> ev_planar_points;
    ev_planar_points.reserve(summary.keypoints.size());
    for (const auto& point : summary.keypoints) {
      ev_planar_points.push_back(to_evalio_point(point.raw_point));
    }

    scan_idx_++;

    return {{"planar", ev_planar_points}};
  }
};

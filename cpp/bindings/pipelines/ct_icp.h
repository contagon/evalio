#pragma once

#include <algorithm>
#include <memory>

#include "ct_icp/odometry.hpp"
#include "ct_icp/types.hpp"
#include "evalio/pipeline.h"
#include "evalio/types.h"

class CTICP: public evalio::Pipeline {
private:
  std::unique_ptr<ct_icp::Odometry> ct_icp_;
  ct_icp::OdometryOptions config_ =
    ct_icp::OdometryOptions::DefaultDrivingProfile();
  evalio::SE3 lidar_T_imu_ = evalio::SE3::identity();
  size_t scan_idx_ = 0;

  inline evalio::SE3 to_evalio_pose(const ct_icp::TrajectoryFrame& pose) const {
    return evalio::SE3(evalio::SO3::fromMat(pose.begin_R), pose.begin_t);
  }

  inline evalio::Point to_evalio_point(const ct_icp::Point3D& point) const {
    return evalio::Point {
      .x = point.raw_pt.x(),
      .y = point.raw_pt.y(),
      .z = point.raw_pt.z(),
      .intensity = 0.0,
      .t = evalio::Duration::from_nsec(0),
      .row = 0,
      .col = 0,
    };
  }

  inline evalio::Point to_evalio_point(const Eigen::Vector3d& point) const {
    return evalio::Point {
      .x = point.x(),
      .y = point.y(),
      .z = point.z(),
      .intensity = 0.0,
      .t = evalio::Duration::from_nsec(0),
      .row = 0,
      .col = 0,
    };
  }

public:
  CTICP() {
    config_.debug_print = false;
    config_.ct_icp_options.debug_print = false;
  }

  // Info
  static std::string version() {
    return XSTR(EVALIO_CT_ICP);
  }

  static std::string name() {
    return "ct_2022";
  }

  static std::string url() {
    return "https://github.com/jedeschaud/ct_icp";
  }

  static std::map<std::string, evalio::Param> default_params() {
    // Pull all of the defaults parameters for nclt_config.yaml
    // https://github.com/jedeschaud/ct_icp/blob/master/config/odometry/nclt_config.yaml
    return {
      // odometry options
      {"init_voxel_size", 0.2},
      {"init_sample_voxel_size", 1.0},
      {"init_num_frames", 20},
      {"voxel_size", 0.5},
      {"sample_voxel_size", 1.5},
      {"max_num_points_in_voxel", 20},
      {"min_distance_points", 0.1},
      {"distance_error_threshold", 5.0},
      {"motion_compensation", std::string("CONTINUOUS")},
      {"initialization", std::string("INIT_CONSTANT_VELOCITY")},
      // ct_icp
      {"threshold_voxel_occupancy", 1},
      {"init_num_frames", 20},
      {"size_voxel_map", 1.0},
      {"num_iters_icp", 30},
      {"min_number_neighbors", 20},
      {"voxel_neighborhood", 1},
      {"power_planarity", 2.0},
      {"estimate_normal_from_neighborhood", true},
      {"max_number_neighbors", 20},
      {"max_dist_to_plane_ct_icp", 0.5},
      {"threshold_orientation_norm", 0.0001},
      {"threshold_translation_norm", 0.001},
      {"point_to_plane_with_distortion", true},
      {"max_num_residuals", -1},
      {"min_num_residuals", 100},
      {"distance", std::string("CT_POINT_TO_PLANE")},
      {"num_closest_neighbors", 1},
      {"beta_location_consistency", 0.001},
      {"beta_constant_velocity", 0.001},
      {"beta_small_velocity", 0.0},
      {"beta_orientation_consistency", 0.0},
      {"weighting_scheme", std::string("ALL")},
      {"weight_alpha", 0.9},
      {"weight_neighborhood", 0.1},
      {"solver", std::string("CERES")},
      {"loss_function", std::string("CAUCHY")},
      {"ls_max_num_iters", 10},
      {"ls_num_threads", 8},
      {"ls_sigma", 0.1},
      {"ls_tolerant_min_threshold", 0.05},
    };
  }

  // Getters
  const evalio::SE3 pose() override {
    const auto pose = ct_icp_->Trajectory().back();
    return to_evalio_pose(pose) * lidar_T_imu_;
  }

  const std::map<std::string, std::vector<evalio::Point>> map() override {
    const auto map = ct_icp_->GetLocalMap();
    std::vector<evalio::Point> ev_points;
    ev_points.reserve(map.size());
    for (const auto& point : map) {
      ev_points.push_back(to_evalio_point(point));
    }
    return {{"planar", std::move(ev_points)}};
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
      if (key == "init_voxel_size") {
        config_.init_voxel_size = std::get<double>(value);
      } else if (key == "init_sample_voxel_size") {
        config_.init_sample_voxel_size = std::get<double>(value);
      } else if (key == "init_num_frames") {
        config_.init_num_frames = std::get<int>(value);
      } else if (key == "voxel_size") {
        config_.voxel_size = std::get<double>(value);
      } else if (key == "sample_voxel_size") {
        config_.sample_voxel_size = std::get<double>(value);
      } else if (key == "max_num_points_in_voxel") {
        config_.max_num_points_in_voxel = std::get<int>(value);
      } else if (key == "min_distance_points") {
        config_.min_distance_points = std::get<double>(value);
      } else if (key == "distance_error_threshold") {
        config_.distance_error_threshold = std::get<double>(value);
      } else if (key == "motion_compensation") {
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
      }

      // ct-icp options
      if (key == "threshold_voxel_occupancy") {
        config_.ct_icp_options.threshold_voxel_occupancy = std::get<int>(value);
      } else if (key == "init_num_frames") {
        config_.ct_icp_options.init_num_frames = std::get<int>(value);
      } else if (key == "size_voxel_map") {
        config_.ct_icp_options.size_voxel_map = std::get<double>(value);
      } else if (key == "num_iters_icp") {
        config_.ct_icp_options.num_iters_icp = std::get<int>(value);
      } else if (key == "min_number_neighbors") {
        config_.ct_icp_options.min_number_neighbors = std::get<int>(value);
      } else if (key == "voxel_neighborhood") {
        config_.ct_icp_options.voxel_neighborhood =
          static_cast<short>(std::get<int>(value));
      } else if (key == "power_planarity") {
        config_.ct_icp_options.power_planarity = std::get<double>(value);
      } else if (key == "estimate_normal_from_neighborhood") {
        config_.ct_icp_options.estimate_normal_from_neighborhood =
          std::get<bool>(value);
      } else if (key == "max_number_neighbors") {
        config_.ct_icp_options.max_number_neighbors = std::get<int>(value);
      } else if (key == "max_dist_to_plane_ct_icp") {
        config_.ct_icp_options.max_dist_to_plane_ct_icp =
          std::get<double>(value);
      } else if (key == "threshold_orientation_norm") {
        config_.ct_icp_options.threshold_orientation_norm =
          std::get<double>(value);
      } else if (key == "threshold_translation_norm") {
        config_.ct_icp_options.threshold_translation_norm =
          std::get<double>(value);
      } else if (key == "point_to_plane_with_distortion") {
        config_.ct_icp_options.point_to_plane_with_distortion =
          std::get<bool>(value);
      } else if (key == "max_num_residuals") {
        config_.ct_icp_options.max_num_residuals = std::get<int>(value);
      } else if (key == "min_num_residuals") {
        config_.ct_icp_options.min_num_residuals = std::get<int>(value);
      } else if (key == "distance") {
        const std::string& dist_str = std::get<std::string>(value);
        if (dist_str == "POINT_TO_PLANE") {
          config_.ct_icp_options.distance =
            ct_icp::ICP_DISTANCE::CT_POINT_TO_PLANE;
        } else if (dist_str == "CT_POINT_TO_PLANE") {
          config_.ct_icp_options.distance =
            ct_icp::ICP_DISTANCE::CT_POINT_TO_PLANE;
        }
      } else if (key == "num_closest_neighbors") {
        config_.ct_icp_options.num_closest_neighbors = std::get<int>(value);
      } else if (key == "beta_location_consistency") {
        config_.ct_icp_options.beta_location_consistency =
          std::get<double>(value);
      } else if (key == "beta_constant_velocity") {
        config_.ct_icp_options.beta_constant_velocity = std::get<double>(value);
      } else if (key == "beta_small_velocity") {
        config_.ct_icp_options.beta_small_velocity = std::get<double>(value);
      } else if (key == "beta_orientation_consistency") {
        config_.ct_icp_options.beta_orientation_consistency =
          std::get<double>(value);
      } else if (key == "weighting_scheme") {
        const std::string& ws_str = std::get<std::string>(value);
        if (ws_str == "PLANARITY") {
          config_.ct_icp_options.weighting_scheme =
            ct_icp::WEIGHTING_SCHEME::PLANARITY;
        } else if (ws_str == "NEIGHBORHOOD") {
          config_.ct_icp_options.weighting_scheme =
            ct_icp::WEIGHTING_SCHEME::NEIGHBORHOOD;
        } else if (ws_str == "ALL") {
          config_.ct_icp_options.weighting_scheme =
            ct_icp::WEIGHTING_SCHEME::ALL;
        }
      } else if (key == "weight_alpha") {
        config_.ct_icp_options.weight_alpha = std::get<double>(value);
      } else if (key == "weight_neighborhood") {
        config_.ct_icp_options.weight_neighborhood = std::get<double>(value);
      } else if (key == "solver") {
        const std::string& solver_str = std::get<std::string>(value);
        if (solver_str == "GN") {
          config_.ct_icp_options.solver = ct_icp::CT_ICP_SOLVER::GN;
        } else if (solver_str == "CERES") {
          config_.ct_icp_options.solver = ct_icp::CT_ICP_SOLVER::CERES;
        }
      } else if (key == "loss_function") {
        const std::string& lf_str = std::get<std::string>(value);
        if (lf_str == "STANDARD") {
          config_.ct_icp_options.loss_function =
            ct_icp::LEAST_SQUARES::STANDARD;
        } else if (lf_str == "CAUCHY") {
          config_.ct_icp_options.loss_function = ct_icp::LEAST_SQUARES::CAUCHY;
        } else if (lf_str == "HUBER") {
          config_.ct_icp_options.loss_function = ct_icp::LEAST_SQUARES::HUBER;
        } else if (lf_str == "TOLERANT") {
          config_.ct_icp_options.loss_function =
            ct_icp::LEAST_SQUARES::TOLERANT;
        } else if (lf_str == "TRUNCATED") {
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
    ct_icp_ = std::make_unique<ct_icp::Odometry>(config_);
  }

  void add_imu(evalio::ImuMeasurement mm) override {}

  std::map<std::string, std::vector<evalio::Point>>
  add_lidar(evalio::LidarMeasurement mm) override {
    // Set everything up
    std::vector<ct_icp::Point3D> pc;
    pc.reserve(mm.points.size());

    // Figure out min/max timesteps
    const auto& [min, max] = std::minmax_element(
      mm.points.cbegin(),
      mm.points.cend(),
      [](const evalio::Point& a, const evalio::Point& b) { return a.t < b.t; }
    );

    const auto min_t = min->t.to_sec();
    const auto max_t = max->t.to_sec();
    const auto normalize = [min_t, max_t](evalio::Duration t) {
      return (t.to_sec() - min_t) / (max_t - min_t);
    };

    // Copy
    for (const auto& point : mm.points) {
      ct_icp::Point3D p;
      p.raw_pt = Eigen::Vector3d(point.x, point.y, point.z);
      p.pt = p.raw_pt;
      p.alpha_timestamp = normalize(point.t);
      p.index_frame = scan_idx_;
      pc.push_back(p);
    }

    // Run through pipeline
    const auto summary = ct_icp_->RegisterFrame(pc);

    // Return the used points
    std::vector<evalio::Point> ev_planar_points;
    ev_planar_points.reserve(summary.keypoints.size());
    for (const auto& point : summary.keypoints) {
      ev_planar_points.push_back(to_evalio_point(point));
    }

    scan_idx_++;

    return {{"planar", ev_planar_points}};
  }
};

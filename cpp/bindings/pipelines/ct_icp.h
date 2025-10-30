#pragma once

#include <algorithm>
#include <memory>

#include "ct_icp/odometry.hpp"
#include "ct_icp/types.hpp"
#include "evalio/convert/base.h"
#include "evalio/convert/eigen.h"
#include "evalio/pipeline.h"
#include "evalio/types.h"

namespace ev = evalio;

// ------------------------- Handle all conversions ------------------------- //
namespace evalio {
template<>
inline evalio::Point convert(const ct_icp::Point3D& from) {
  return ev::Point {
    .x = from.raw_pt.x(),
    .y = from.raw_pt.y(),
    .z = from.raw_pt.z(),
    .intensity = 0.0,
    .t = ev::Duration::from_nsec(0),
    .row = 0,
    .col = 0,
  };
}

template<>
inline ct_icp::Point3D convert(const evalio::Point& from) {
  ct_icp::Point3D to;
  to.raw_pt = Eigen::Vector3d(from.x, from.y, from.z);
  to.pt = to.raw_pt;
  to.alpha_timestamp = from.t.to_sec();
  to.index_frame = 0;
  return to;
}

template<>
inline evalio::SE3 convert(const ct_icp::TrajectoryFrame& in) {
  return ev::SE3(ev::SO3::fromMat(in.begin_R), in.begin_t);
}
} // namespace evalio

// ------------------------- Actual Wrapper ------------------------- //
// Simple enum wrapper to parse all strings -> enum options
struct CTICPEnumParams {
  std::string motion_compensation;
  std::string initialization;
  std::string distance;
  std::string weighting_scheme;
  std::string solver;
  std::string loss_function;

  void store(ct_icp::OdometryOptions& options) {
    if (motion_compensation == "NONE") {
      options.motion_compensation = ct_icp::MOTION_COMPENSATION::NONE;
    } else if (motion_compensation == "CONSTANT_VELOCITY") {
      options.motion_compensation =
        ct_icp::MOTION_COMPENSATION::CONSTANT_VELOCITY;
    } else if (motion_compensation == "ITERATIVE") {
      options.motion_compensation = ct_icp::MOTION_COMPENSATION::ITERATIVE;
    } else if (motion_compensation == "CONTINUOUS") {
      options.motion_compensation = ct_icp::MOTION_COMPENSATION::CONTINUOUS;
    } else {
      throw std::runtime_error(
        "Invalid motion compensation option: " + motion_compensation
      );
    }

    if (initialization == "INIT_NONE") {
      options.initialization = ct_icp::INITIALIZATION::INIT_NONE;
    } else if (initialization == "INIT_CONSTANT_VELOCITY") {
      options.initialization = ct_icp::INITIALIZATION::INIT_CONSTANT_VELOCITY;
    } else {
      throw std::runtime_error(
        "Invalid initialization option: " + initialization
      );
    }

    if (distance == "POINT_TO_PLANE") {
      options.ct_icp_options.distance = ct_icp::ICP_DISTANCE::POINT_TO_PLANE;
    } else if (distance == "CT_POINT_TO_PLANE") {
      options.ct_icp_options.distance = ct_icp::ICP_DISTANCE::CT_POINT_TO_PLANE;
    } else {
      throw std::runtime_error("Invalid distance option: " + distance);
    }

    if (weighting_scheme == "PLANARITY") {
      options.ct_icp_options.weighting_scheme =
        ct_icp::WEIGHTING_SCHEME::PLANARITY;
    } else if (weighting_scheme == "NEIGHBORHOOD") {
      options.ct_icp_options.weighting_scheme =
        ct_icp::WEIGHTING_SCHEME::NEIGHBORHOOD;
    } else if (weighting_scheme == "ALL") {
      options.ct_icp_options.weighting_scheme = ct_icp::WEIGHTING_SCHEME::ALL;
    } else {
      throw std::runtime_error(
        "Invalid weighting scheme option: " + weighting_scheme
      );
    }

    if (solver == "GN") {
      options.ct_icp_options.solver = ct_icp::CT_ICP_SOLVER::GN;
    } else if (solver == "CERES") {
      options.ct_icp_options.solver = ct_icp::CT_ICP_SOLVER::CERES;
    } else {
      throw std::runtime_error("Invalid solver option: " + solver);
    }

    if (loss_function == "STANDARD") {
      options.ct_icp_options.loss_function = ct_icp::LEAST_SQUARES::STANDARD;
    } else if (loss_function == "CAUCHY") {
      options.ct_icp_options.loss_function = ct_icp::LEAST_SQUARES::CAUCHY;
    } else if (loss_function == "HUBER") {
      options.ct_icp_options.loss_function = ct_icp::LEAST_SQUARES::HUBER;
    } else if (loss_function == "TOLERANT") {
      options.ct_icp_options.loss_function = ct_icp::LEAST_SQUARES::TOLERANT;
    } else if (loss_function == "TRUNCATED") {
      options.ct_icp_options.loss_function = ct_icp::LEAST_SQUARES::TRUNCATED;
    } else {
      throw std::runtime_error(
        "Invalid loss function option: " + loss_function
      );
    }
  }
};

class CTICP: public ev::Pipeline {
private:
  std::unique_ptr<ct_icp::Odometry> ct_icp_;
  ct_icp::OdometryOptions config_ =
    ct_icp::OdometryOptions::DefaultDrivingProfile();
  CTICPEnumParams enum_params_;

  ev::SE3 lidar_T_imu_ = ev::SE3::identity();
  size_t scan_idx_ = 0;

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
    return "ct";
  }

  static std::string url() {
    return "https://github.com/jedeschaud/ct_icp";
  }

  // clang-format off
  EVALIO_SETUP_PARAMS(
    // odometry options
    (double, init_voxel_size, 0.2, config_.init_voxel_size),
    (double, init_sample_voxel_size, 1.0, config_.init_sample_voxel_size),
    (int, init_num_frames, 20, config_.init_num_frames),
    (double, voxel_size, 0.5, config_.voxel_size),
    (double, sample_voxel_size, 1.5, config_.sample_voxel_size),
    (int, max_num_points_in_voxel, 20, config_.max_num_points_in_voxel),
    (double, min_distance_points, 0.1, config_.min_distance_points),
    (double, distance_error_threshold, 5.0, config_.distance_error_threshold),
    (std::string, motion_compensation, std::string("CONTINUOUS"), enum_params_.motion_compensation),
    (std::string, initialization, std::string("INIT_CONSTANT_VELOCITY"), enum_params_.initialization),
    // ct-icp options
    (int, threshold_voxel_occupancy, 1, config_.ct_icp_options.threshold_voxel_occupancy),
    (int, ct_init_num_frames, 20, config_.ct_icp_options.init_num_frames),
    (double, size_voxel_map, 1.0, config_.ct_icp_options.size_voxel_map),
    (int, num_iters_icp, 30, config_.ct_icp_options.num_iters_icp),
    (int, min_number_neighbors, 20, config_.ct_icp_options.min_number_neighbors),
    (int, voxel_neighborhood, 1, config_.ct_icp_options.voxel_neighborhood),
    (double, power_planarity, 2.0, config_.ct_icp_options.power_planarity),
    (bool, estimate_normal_from_neighborhood, true, config_.ct_icp_options.estimate_normal_from_neighborhood),
    (int, max_number_neighbors, 20, config_.ct_icp_options.max_number_neighbors),
    (double, max_dist_to_plane_ct_icp, 0.5, config_.ct_icp_options.max_dist_to_plane_ct_icp),
    (double, threshold_orientation_norm, 0.0001, config_.ct_icp_options.threshold_orientation_norm),
    (double, threshold_translation_norm, 0.001, config_.ct_icp_options.threshold_translation_norm),
    (bool, point_to_plane_with_distortion, true, config_.ct_icp_options.point_to_plane_with_distortion),
    (int, max_num_residuals, -1, config_.ct_icp_options.max_num_residuals),
    (int, min_num_residuals, 100, config_.ct_icp_options.min_num_residuals),
    (std::string, distance, std::string("CT_POINT_TO_PLANE"), enum_params_.distance),
    (int, num_closest_neighbors, 1, config_.ct_icp_options.num_closest_neighbors),
    (double, beta_location_consistency, 0.001, config_.ct_icp_options.beta_location_consistency),
    (double, beta_constant_velocity, 0.001, config_.ct_icp_options.beta_constant_velocity),
    (double, beta_small_velocity, 0.0, config_.ct_icp_options.beta_small_velocity),
    (double, beta_orientation_consistency, 0.0, config_.ct_icp_options.beta_orientation_consistency),
    (std::string, weighting_scheme, std::string("ALL"), enum_params_.weighting_scheme),
    (double, weight_alpha, 0.9, config_.ct_icp_options.weight_alpha),
    (double, weight_neighborhood, 0.1, config_.ct_icp_options.weight_neighborhood),
    (std::string, solver, std::string("CERES"), enum_params_.solver),
    (std::string, loss_function, std::string("CAUCHY"), enum_params_.loss_function),
    (int, ls_max_num_iters, 10, config_.ct_icp_options.ls_max_num_iters),
    (int, ls_num_threads, 8, config_.ct_icp_options.ls_num_threads),
    (double, ls_sigma, 0.1, config_.ct_icp_options.ls_sigma),
    (double, ls_tolerant_min_threshold, 0.05, config_.ct_icp_options.ls_tolerant_min_threshold),
  )
  // clang-format on

  // Getters
  const std::map<std::string, std::vector<ev::Point>> map() override {
    auto map = ct_icp_->GetLocalMap();
    return ev::convert_map<decltype(map)>({{"planar", map}});
  }

  // Setters
  void set_imu_params(ev::ImuParams params) override {}

  void set_lidar_params(ev::LidarParams params) override {
    config_.max_distance = params.max_range;
  }

  void set_imu_T_lidar(ev::SE3 T) override {
    lidar_T_imu_ = T.inverse();
  }

  // Doers
  void initialize() override {
    enum_params_.store(config_);
    ct_icp_ = std::make_unique<ct_icp::Odometry>(config_);
  }

  void add_imu(ev::ImuMeasurement mm) override {}

  void add_lidar(ev::LidarMeasurement mm) override {
    // Convert
    auto pc = ev::convert_iter<std::vector<ct_icp::Point3D>>(mm.points);

    // Normalize timestamps to [0, 1]
    const auto& [min, max] = std::minmax_element(
      pc.cbegin(),
      pc.cend(),
      [](const ct_icp::Point3D& a, const ct_icp::Point3D& b) {
        return a.alpha_timestamp < b.alpha_timestamp;
      }
    );

    const auto min_t = min->alpha_timestamp;
    const auto max_t = max->alpha_timestamp;
    const auto normalize = [min_t, max_t](double t) {
      return (t - min_t) / (max_t - min_t);
    };

    // Copy
    for (auto& p : pc) {
      p.alpha_timestamp = normalize(p.alpha_timestamp);
    }

    // Run through pipeline
    const auto summary = ct_icp_->RegisterFrame(pc);

    // Save the estimate
    const auto pose = ct_icp_->Trajectory().back();
    this->save(mm.stamp, ev::convert<ev::SE3>(pose) * lidar_T_imu_);

    // Save the used points
    this->save<std::vector<ct_icp::Point3D>>(
      mm.stamp,
      {{"planar", summary.keypoints}}
    );

    scan_idx_++;
  }
};

#pragma once

#include <deque>
#include <memory>
#include <stdexcept>

#include "evalio/pipeline.h"
#include "evalio/types.h"
#include "loam/loam.h"

class LOAM : public evalio::Pipeline {
 public:
  LOAM() {}

  // Info
  static std::string version() { return XSTR(EVALIO_LOAM); }
  static std::string name() { return "loam"; }
  static std::string url() { return "https://github.com/DanMcGann/loam"; }

  static std::map<std::string, evalio::Param> default_params() {
    return {
        // Local Map
        {"num_local_map_scans", 3},
        // Feature Extraction
        {"neighbor_points", 3},
        {"number_sectors", 6},
        {"max_edge_feats_per_sector", 5},
        {"max_planar_feats_per_sector", 50},
        {"planar_feat_threshold", 1.0},
        {"edge_feat_threshold", 100.0},
        {"occlusion_thresh", 0.5},
        {"parallel_thresh", 1.0},
        // Registration
        {"max_icf_iterations", 20},
        {"rotation_convergence_thresh", 1e-4},
        {"position_convergence_thresh", 1e-3},
        {"num_edge_neighbors", 5},
        {"max_edge_neighbor_dist", 2.0},
        {"min_line_fit_points", 3},
        {"min_line_condition_number", 5.0},
        {"num_plane_neighbors", 5},
        {"max_plane_neighbor_dist", 4.0},
        {"min_plane_fit_points", 4},
        {"max_avg_point_plane_dist", 0.2},
    };
  }

  // Getters
  const evalio::SE3 pose() override { return to_se3(current_estimated_pose) * lidar_T_imu_; }

  const std::vector<evalio::Point> map() override {
    return features_to_points(transform_features(map_features(), current_estimated_pose));
  }

  // Setters
  void set_imu_params(evalio::ImuParams params) override {};
  void set_lidar_params(evalio::LidarParams params) override {
    loam_lidar_params_ =
        std::make_unique<loam::LidarParams>(params.num_rows, params.num_columns, params.min_range, params.max_range);
  };
  void set_imu_T_lidar(evalio::SE3 T) override { lidar_T_imu_ = T.inverse(); }

  void set_params(std::map<std::string, evalio::Param> params) override {
    for (auto& [key, value] : params) {
      if (std::holds_alternative<int>(value)) {
        if (key == "num_local_map_scans") {
          num_local_map_scans_ = std::get<int>(value);
        } else if (key == "max_edge_feats_per_sector") {
          loam_fe_params_.max_edge_feats_per_sector = std::get<int>(value);
        } else if (key == "max_planar_feats_per_sector") {
          loam_fe_params_.max_planar_feats_per_sector = std::get<int>(value);
        } else if (key == "neighbor_points") {
          loam_fe_params_.neighbor_points = std::get<int>(value);
        } else if (key == "number_sectors") {
          loam_fe_params_.number_sectors = std::get<int>(value);
        } else if (key == "max_icf_iterations") {
          loam_registration_params_.max_iterations = std::get<int>(value);
        } else if (key == "num_edge_neighbors") {
          loam_registration_params_.num_edge_neighbors = std::get<int>(value);
        } else if (key == "min_line_fit_points") {
          loam_registration_params_.min_line_fit_points = std::get<int>(value);
        } else if (key == "num_plane_neighbors") {
          loam_registration_params_.num_plane_neighbors = std::get<int>(value);
        } else if (key == "min_plane_fit_points") {
          loam_registration_params_.min_plane_fit_points = std::get<int>(value);
        }

      } else if (std::holds_alternative<double>(value)) {
        if (key == "planar_feat_threshold") {
          loam_fe_params_.planar_feat_threshold = std::get<double>(value);
        } else if (key == "edge_feat_threshold") {
          loam_fe_params_.edge_feat_threshold = std::get<double>(value);
        } else if (key == "occlusion_thresh") {
          loam_fe_params_.occlusion_thresh = std::get<double>(value);
        } else if (key == "parallel_thresh") {
          loam_fe_params_.occlusion_thresh = std::get<double>(value);
        } else if (key == "rotation_convergence_thresh") {
          loam_registration_params_.rotation_convergence_thresh = std::get<double>(value);
        } else if (key == "position_convergence_thresh") {
          loam_registration_params_.position_convergence_thresh = std::get<double>(value);
        } else if (key == "max_edge_neighbor_dist") {
          loam_registration_params_.max_edge_neighbor_dist = std::get<double>(value);
        } else if (key == "min_line_condition_number") {
          loam_registration_params_.min_line_condition_number = std::get<double>(value);
        } else if (key == "max_plane_neighbor_dist") {
          loam_registration_params_.max_plane_neighbor_dist = std::get<double>(value);
        } else if (key == "max_avg_point_plane_dist") {
          loam_registration_params_.max_avg_point_plane_dist = std::get<double>(value);
        }
      }
    }
  }

  // Doers
  void initialize() override {}
  void add_imu(evalio::ImuMeasurement mm) override {};

  std::vector<evalio::Point> add_lidar(evalio::LidarMeasurement mm) override {
    // Handle Edge case of the first scan
    if (past_k_scans_.size() == 0) {
      // Extract Features from the first scan
      auto scan_features = loam::extractFeatures(mm.points, *loam_lidar_params_, loam_fe_params_);
      // Initialize the Local Map
      past_k_scans_.push_back(std::make_pair(loam::Pose3d::Identity(), scan_features));
      // Initialize the odometry frame pose
      current_estimated_pose = loam::Pose3d::Identity();
      // Return the initial scan features
      return features_to_points(scan_features);
    }
    // Default case for all iterations except the first
    else {
      // Extract Features from the new scan
      auto scan_features = loam::extractFeatures(mm.points, *loam_lidar_params_, loam_fe_params_);
      // Get the Local Map Features (in the frame of the previous scan)
      auto local_map = map_features();
      // Register to the local map
      auto init_transform = past_k_scans_.size() > 0 ? past_k_scans_.back().first : loam::Pose3d();
      auto map_T_scan = loam::registerFeatures(scan_features, local_map, init_transform, loam_registration_params_);
      // Aggregate scan into history maintaining only K=num_local_map_scans scans
      past_k_scans_.push_back(std::make_pair(map_T_scan, scan_features));
      if (past_k_scans_.size() > num_local_map_scans_) past_k_scans_.pop_front();
      // Update the Odometry frame pose
      current_estimated_pose = current_estimated_pose.compose(map_T_scan);
      // Return the points associated with the used features
      return features_to_points(scan_features);
    }
  }

 private:
  /// @brief The lidar params converted to the LOAM type
  std::unique_ptr<loam::LidarParams> loam_lidar_params_;
  /// @brief The parameters for feature (plan + edge) extraction
  loam::FeatureExtractionParams loam_fe_params_;
  /// @brief The parameters for feature registration
  loam::RegistrationParams loam_registration_params_;
  /// @brief The number of scans to keep in the local map
  size_t num_local_map_scans_{1};

  /// @brief The last k scans (features points only) and their respective odometry measurements
  std::deque<std::pair<loam::Pose3d, loam::LoamFeatures<evalio::Point>>> past_k_scans_;
  ///  @brief The current pose estimate in the odometry frame {odom_T_lidar}
  loam::Pose3d current_estimated_pose{loam::Pose3d::Identity()};
  /// @brief The transform from lidar to IMU
  evalio::SE3 lidar_T_imu_{evalio::SE3::identity()};

 private:
  /// @brief Converts a loam SE(3) pose type to a evalio SE(3) pose type
  inline evalio::SE3 to_se3(loam::Pose3d pose) {
    return evalio::SE3(evalio::SO3::fromEigen(pose.rotation), pose.translation);
  }

  /// @brief Converts evalio point to eigen
  inline Eigen::Vector3d to_eigen_point(evalio::Point point) { return {point.x, point.y, point.z}; }

  /// @brief Transforms a evalio point from its source frame to a target frame
  inline evalio::Point transform_point(evalio::Point src_pt, loam::Pose3d target_T_source) {
    auto tgt_pt = target_T_source.act(to_eigen_point(src_pt));
    return {
        tgt_pt(0), tgt_pt(1), tgt_pt(2), src_pt.intensity, src_pt.t, src_pt.range, src_pt.row, src_pt.col,
    };
  }

  /// @brief Helper to aggregate all features (edge + planar) into a single container
  std::vector<evalio::Point> features_to_points(const loam::LoamFeatures<evalio::Point> features) {
    std::vector<evalio::Point> points;
    for (const auto& planar_pt : features.planar_points) points.push_back(planar_pt);
    for (const auto& edge_pt : features.edge_points) points.push_back(edge_pt);
    return points;
  }

  /// @brief Aggregate the history of k most recent scans into a local map in the frame of the most recent registered
  loam::LoamFeatures<evalio::Point> map_features() {
    // Setup accumulators
    auto current_T_map = loam::Pose3d::Identity();
    auto map_features = loam::LoamFeatures<evalio::Point>();

    // Iterate from the most recent scan to the oldest in the stored history
    for (auto it = past_k_scans_.rbegin(); it != past_k_scans_.rend(); ++it) {
      auto [prev_T_current, features] = *it;

      // Transform from the current scan frame into the map frame (latest registered scan)
      auto scan_feat_in_map_frame = transform_features(features, current_T_map.inverse());
      map_features.edge_points.insert(map_features.edge_points.end(), scan_feat_in_map_frame.edge_points.begin(),
                                      scan_feat_in_map_frame.edge_points.end());
      map_features.planar_points.insert(map_features.planar_points.end(), scan_feat_in_map_frame.planar_points.begin(),
                                        scan_feat_in_map_frame.planar_points.end());
      // Accumulate the transforms
      current_T_map = prev_T_current.compose(current_T_map);
    }
    return map_features;
  }

  /// @brief Transforms all features into a new frame
  loam::LoamFeatures<evalio::Point> transform_features(loam::LoamFeatures<evalio::Point> features,
                                                       loam::Pose3d target_T_source) {
    loam::LoamFeatures<evalio::Point> transformed_features;
    for (const auto& planar_pt : features.planar_points) {
      transformed_features.planar_points.push_back(transform_point(planar_pt, target_T_source));
    }
    for (const auto& edge_pt : features.edge_points) {
      transformed_features.edge_points.push_back(transform_point(edge_pt, target_T_source));
    }
    return transformed_features;
  }
};
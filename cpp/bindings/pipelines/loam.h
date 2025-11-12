#pragma once

#include <deque>
#include <memory>

#include "evalio/convert/eigen.h"
#include "evalio/pipeline.h"
#include "evalio/types.h"
#include "loam/loam.h"

namespace ev = evalio;

namespace evalio {
template<>
inline ev::SE3 convert(const loam::Pose3d& from) {
  return ev::SE3(ev::SO3::from_eigen(from.rotation), from.translation);
}
} // namespace evalio

class LOAM: public ev::Pipeline {
public:
  LOAM() {}

  // Info
  static std::string version() {
    return XSTR(EVALIO_LOAM);
  }

  static std::string name() {
    return "loam";
  }

  static std::string url() {
    return "https://github.com/DanMcGann/loam";
  }

  // clang-format off
  EVALIO_SETUP_PARAMS(
    // Local Map
    (int, num_local_map_scans, 3, num_local_map_scans_),
    // Feature Extraction
    (int, neighbor_points, 3, loam_fe_params_.neighbor_points),
    (int, number_sectors, 6, loam_fe_params_.number_sectors),
    (int, max_edge_feats_per_sector, 5, loam_fe_params_.max_edge_feats_per_sector),
    (int, max_planar_feats_per_sector, 50, loam_fe_params_.max_planar_feats_per_sector),
    (double, planar_feat_threshold, 1.0, loam_fe_params_.planar_feat_threshold),
    (double, edge_feat_threshold, 100.0, loam_fe_params_.edge_feat_threshold),
    (double, occlusion_thresh, 0.5, loam_fe_params_.occlusion_thresh),
    (double, parallel_thresh, 1.0, loam_fe_params_.parallel_thresh),
    // Registration
    (int, max_icf_iterations, 20, loam_registration_params_.max_iterations),
    (double, rotation_convergence_thresh, 1e-4, loam_registration_params_.rotation_convergence_thresh),
    (double, position_convergence_thresh, 1e-3, loam_registration_params_.position_convergence_thresh),
    (int, num_edge_neighbors, 5, loam_registration_params_.num_edge_neighbors),
    (double, max_edge_neighbor_dist, 2.0, loam_registration_params_.max_edge_neighbor_dist),
    (int, min_line_fit_points, 3, loam_registration_params_.min_line_fit_points),
    (double, min_line_condition_number, 5.0, loam_registration_params_.min_line_condition_number),
    (int, num_plane_neighbors, 5, loam_registration_params_.num_plane_neighbors),
    (double, max_plane_neighbor_dist, 4.0, loam_registration_params_.max_plane_neighbor_dist),
    (int, min_plane_fit_points, 4, loam_registration_params_.min_plane_fit_points),
    (double, max_avg_point_plane_dist, 0.2, loam_registration_params_.max_avg_point_plane_dist)
  );
  // clang-format on

  // Getters
  const std::map<std::string, std::vector<ev::Point>> map() override {
    return features_to_points(
      transform_features(map_features(), current_estimated_pose)
    );
  }

  // Setters
  void set_imu_params(ev::ImuParams params) override {}

  void set_lidar_params(ev::LidarParams params) override {
    loam_lidar_params_ = std::make_unique<loam::LidarParams>(
      params.num_rows,
      params.num_columns,
      params.min_range,
      params.max_range
    );
  }

  void set_imu_T_lidar(ev::SE3 T) override {
    lidar_T_imu_ = T.inverse();
  }

  // Doers
  void initialize() override {}

  void add_imu(ev::ImuMeasurement mm) override {}

  void add_lidar(ev::LidarMeasurement mm) override {
    // Handle Edge case of the first scan
    if (past_k_scans_.size() == 0) {
      // Extract Features from the first scan
      auto scan_features =
        loam::extractFeatures(mm.points, *loam_lidar_params_, loam_fe_params_);
      // Initialize the Local Map
      past_k_scans_.push_back(
        std::make_pair(loam::Pose3d::Identity(), scan_features)
      );
      // Initialize the odometry frame pose
      this->save(mm.stamp, evalio::SE3::identity() * lidar_T_imu_);
      // Return the initial scan features
      this->save(mm.stamp, features_to_points(scan_features));
    }
    // Default case for all iterations except the first
    else {
      // Extract Features from the new scan
      auto scan_features =
        loam::extractFeatures(mm.points, *loam_lidar_params_, loam_fe_params_);
      // Get the Local Map Features (in the frame of the previous scan)
      auto local_map = map_features();
      // Register to the local map
      auto init_transform =
        past_k_scans_.size() > 0 ? past_k_scans_.back().first : loam::Pose3d();
      auto map_T_scan = loam::registerFeatures(
        scan_features,
        local_map,
        init_transform,
        loam_registration_params_
      );
      // Aggregate scan into history maintaining only K=num_local_map_scans
      // scans
      past_k_scans_.push_back(std::make_pair(map_T_scan, scan_features));
      if (past_k_scans_.size() > num_local_map_scans_) {
        past_k_scans_.pop_front();
      }
      // Update the Odometry frame pose
      current_estimated_pose = current_estimated_pose.compose(map_T_scan);
      const auto pose_ev =
        ev::convert<ev::SE3>(current_estimated_pose) * lidar_T_imu_;
      this->save(mm.stamp, pose_ev);
      // Return the points associated with the used features
      this->save(mm.stamp, features_to_points(scan_features));
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
  size_t num_local_map_scans_ {1};

  /// @brief The last k scans (features points only) and their respective
  /// odometry measurements
  std::deque<std::pair<loam::Pose3d, loam::LoamFeatures<ev::Point>>>
    past_k_scans_;
  ///  @brief The current pose estimate in the odometry frame {odom_T_lidar}
  loam::Pose3d current_estimated_pose {loam::Pose3d::Identity()};
  /// @brief The transform from lidar to IMU
  ev::SE3 lidar_T_imu_ {ev::SE3::identity()};

private:
  /// @brief Transforms a evalio point from its source frame to a target frame
  inline ev::Point
  transform_point(ev::Point src_pt, loam::Pose3d target_T_source) {
    auto tgt_pt = target_T_source.act(ev::convert<Eigen::Vector3d>(src_pt));
    return {
      tgt_pt(0),
      tgt_pt(1),
      tgt_pt(2),
      src_pt.intensity,
      src_pt.t,
      src_pt.range,
      src_pt.row,
      src_pt.col,
    };
  }

  /// @brief Helper to aggregate all features (edge + planar) into a single
  /// container
  std::map<std::string, std::vector<ev::Point>>
  features_to_points(const loam::LoamFeatures<ev::Point> features) {
    return {{"planar", features.planar_points}, {"edge", features.edge_points}};
  }

  /// @brief Aggregate the history of k most recent scans into a local map in
  /// the frame of the most recent registered
  loam::LoamFeatures<ev::Point> map_features() {
    // Setup accumulators
    auto current_T_map = loam::Pose3d::Identity();
    auto map_features = loam::LoamFeatures<ev::Point>();

    // Iterate from the most recent scan to the oldest in the stored history
    for (auto it = past_k_scans_.rbegin(); it != past_k_scans_.rend(); ++it) {
      auto [prev_T_current, features] = *it;

      // Transform from the current scan frame into the map frame (latest
      // registered scan)
      auto scan_feat_in_map_frame =
        transform_features(features, current_T_map.inverse());
      map_features.edge_points.insert(
        map_features.edge_points.end(),
        scan_feat_in_map_frame.edge_points.begin(),
        scan_feat_in_map_frame.edge_points.end()
      );
      map_features.planar_points.insert(
        map_features.planar_points.end(),
        scan_feat_in_map_frame.planar_points.begin(),
        scan_feat_in_map_frame.planar_points.end()
      );
      // Accumulate the transforms
      current_T_map = prev_T_current.compose(current_T_map);
    }
    return map_features;
  }

  /// @brief Transforms all features into a new frame
  loam::LoamFeatures<ev::Point> transform_features(
    loam::LoamFeatures<ev::Point> features,
    loam::Pose3d target_T_source
  ) {
    loam::LoamFeatures<ev::Point> transformed_features;
    for (const auto& planar_pt : features.planar_points) {
      transformed_features.planar_points.push_back(
        transform_point(planar_pt, target_T_source)
      );
    }
    for (const auto& edge_pt : features.edge_points) {
      transformed_features.edge_points.push_back(
        transform_point(edge_pt, target_T_source)
      );
    }
    return transformed_features;
  }
};

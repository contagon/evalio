#pragma once

#include <gtsam/geometry/Pose3.h>

#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "evalio/convert/base.h"
#include "evalio/convert/gtsam.h"
#include "evalio/pipeline.h"
#include "evalio/types.h"
#include "form/feature/extraction.hpp"
#include "form/form.hpp"
#include "form/utils.hpp"

namespace ev = evalio;

namespace evalio {

template<>
inline Point convert(const form::PointFeat& in) {
  return {
    .x = in.x,
    .y = in.y,
    .z = in.z,
    .intensity = 0.0,
    .t = Duration::from_sec(0),
    .row = 0,
    .col = static_cast<uint16_t>(in.scan),
  };
}

template<>
inline Point convert(const form::PlanarFeat& in) {
  return {
    .x = in.x,
    .y = in.y,
    .z = in.z,
    .intensity = 0.0,
    .t = Duration::from_sec(0),
    .row = 0,
    .col = static_cast<uint16_t>(in.scan),
  };
}

template<>
inline Point convert(const form::PointXYZf& in) {
  return {
    .x = in.x,
    .y = in.y,
    .z = in.z,
    .intensity = 0.0,
    .t = Duration::from_sec(0),
    .row = 0,
    .col = 0,
  };
}

template<>
inline form::PointXYZf convert(const Point& in) {
  return {
    static_cast<float>(in.x),
    static_cast<float>(in.y),
    static_cast<float>(in.z),
  };
}

} // namespace evalio

class FORM: public ev::Pipeline {
public:
  FORM() : estimator_(), params_() {}

  // Info
  static std::string version() {
    return XSTR(EVALIO_FORM);
  }

  static std::string name() {
    return "form";
  }

  static std::string url() {
    return "https://github.com/rpl-cmu/form";
  }

  // clang-format off
  EVALIO_SETUP_PARAMS(
    // feature extraction
    (int, neighbor_points, 5, params_.extraction.neighbor_points),
    (int, num_sectors, 6, params_.extraction.num_sectors),
    (double, planar_threshold, 1.0, params_.extraction.planar_threshold),
    (int, planar_feats_per_sector, 50, params_.extraction.planar_feats_per_sector),
    (int, point_feats_per_sector, 3, params_.extraction.point_feats_per_sector),
    (double, radius, 1.0, params_.extraction.radius),
    (int, min_points, 5, params_.extraction.min_points),
    // optimization
    (double, max_dist_matching, 0.8, params_.matcher.max_dist_matching),
    (double, new_pose_threshold, 1e-4, params_.matcher.new_pose_threshold),
    (int, max_num_rematches, 30, params_.matcher.max_num_rematches),
    (bool, disable_smoothing, false, params_.constraints.disable_smoothing),
    // mapping
    (int, max_num_keyscans, 50, params_.scans.max_num_keyscans),
    (int, max_num_recent_scans, 10, params_.scans.max_num_recent_scans),
    (int, max_steps_unused_keyscan, 10, params_.scans.max_steps_unused_keyscan),
    (double, keyscan_match_ratio, 0.1, params_.scans.keyscan_match_ratio),
    (double, min_dist_map, 0.1, params_.map.min_dist_map),
    // misc
    (int, num_threads, 0, params_.num_threads)
  );
  // clang-format on

  // Getters
  const std::map<std::string, std::vector<ev::Point>> map() override {
    const auto world_map =
      form::tuple::transform(estimator_.m_keypoint_map, [&](auto& map) {
        return map.to_voxel_map(
          estimator_.m_constraints.get_values(),
          estimator_.m_params.map.min_dist_map
        );
      });

    std::tuple<std::string, std::string> map_names = {"planar", "point"};
    std::map<std::string, std::vector<ev::Point>> points;

    form::tuple::for_seq(std::make_index_sequence<2> {}, [&](auto I) {
      const auto name = std::get<I>(map_names);
      points.insert({name, {}});
      auto& vec = points[name];

      for (const auto& [_, voxel] : std::get<I>(world_map)) {
        for (const auto& point : voxel) {
          vec.push_back(ev::convert<ev::Point>(point));
        }
      }
    });

    return points;
  }

  // Setters
  void set_imu_params(ev::ImuParams params) override {}

  void set_lidar_params(ev::LidarParams params) override {
    params_.extraction.min_norm_squared = params.min_range * params.min_range;
    params_.extraction.max_norm_squared = params.max_range * params.max_range;
    params_.extraction.num_columns = params.num_columns;
    params_.extraction.num_rows = params.num_rows;
  }

  void set_imu_T_lidar(ev::SE3 T) override {
    lidar_T_imu_ = ev::convert<gtsam::Pose3>(T).inverse();
  }

  // Doers
  void initialize() override {
    estimator_ = form::Estimator(params_);
  }

  void add_imu(ev::ImuMeasurement mm) override {}

  void add_lidar(ev::LidarMeasurement mm) override {
    const auto scan = ev::convert_iter<std::vector<form::PointXYZf>>(mm.points);

    auto [planar_kp, point_kp] = estimator_.register_scan(scan);
    current_pose_ =
      ev::convert<ev::SE3>(estimator_.current_lidar_estimate() * lidar_T_imu_);

    this->save(mm.stamp, current_pose_);
    this->save(mm.stamp, ev::make_map("planar", planar_kp, "point", point_kp));
  }

private:
  form::Estimator estimator_;
  form::Estimator::Params params_;

  gtsam::Pose3 lidar_T_imu_ = gtsam::Pose3::Identity();
  ev::SE3 current_pose_ = ev::SE3::identity();
};

#pragma once

#include <Eigen/Core>
#include <map>
#include <optional>
#include <set>
#include <variant>

#include "evalio/convert/base.h"
#include "evalio/convert/eigen.h"
#include "evalio/macros.h"
#include "evalio/types.h"

// Gotten from here, so we don't have to use pybind here directly
// https://github.com/pybind/pybind11/blob/master/include/pybind11/detail/common.h#L164C1-L170C7
// Helps evalio bindings be found in other modules
// Licensed via BSD-3-Clause
#if !defined(PYBIND11_EXPORT)
  #if defined(WIN32) || defined(_WIN32)
    #define PYBIND11_EXPORT __declspec(dllexport)
  #else
    #define PYBIND11_EXPORT __attribute__((visibility("default")))
  #endif
#endif

// For converting version definitions to string
// https://stackoverflow.com/a/10791845
#define STR(x) #x
#define XSTR(x) STR(x)

namespace evalio {

/// @brief A variant type for parameters.
using Param = std::variant<bool, int, double, std::string>;

/// @brief A map from string keys to containers of points.
template<typename T = std::vector<Point>>
using Map = std::map<std::string, T>;

/// @brief A tuple of a timestamp and arbitrary other types.
template<typename... T>
using Stamped = std::tuple<Stamp, T...>;

class PYBIND11_EXPORT Pipeline {
public:
  virtual ~Pipeline() {}

  // ------------------------- Things to override/use ------------------------- //
  // Info
  static std::string version() {
    return "0.0.0";
  }

  static std::string url() {
    return "url-not-set";
  }

  static std::string name() {
    throw std::runtime_error("Not implemented");
  }

  // Should be implemented with EVALIO_SETUP_PARAMS
  static std::map<std::string, Param> default_params() {
    throw std::runtime_error("Not implemented");
  }

  // Getters
  virtual const Map<> map() = 0;

  // Setters
  virtual void set_imu_params(ImuParams params) = 0;
  virtual void set_lidar_params(LidarParams params) = 0;
  virtual void set_imu_T_lidar(SE3 T) = 0;
  // Should be implemented with EVALIO_SETUP_PARAMS
  virtual std::map<std::string, Param>
  set_params(std::map<std::string, Param> params) = 0;

  // Doers
  virtual void initialize() = 0;
  virtual void add_imu(ImuMeasurement mm) = 0;
  virtual void add_lidar(LidarMeasurement mm) = 0;

  // ------------------------- For saving results ------------------------- //
  template<typename T>
  void save(const Stamp& stamp, const T& pose) {
    saved_poses_.emplace_back(stamp, convert<SE3>(pose));
  }

  template<typename T>
  void save(const Stamp& stamp, const std::map<std::string, T>& features) {
    // Only convert & save if they'll be visualized
    if (vis_options_
        && vis_options_->find(VisOption::FEATURES) != vis_options_->end()) {
      saved_features_.emplace_back(stamp, convert_map(features));
    }

    // Only query the map if it'll be visualized
    if (vis_options_
        && vis_options_->find(VisOption::MAP) != vis_options_->end()) {
      saved_maps_.emplace_back(stamp, map());
    }
  }

  // ------------------------- For Internal Usage ------------------------- //
  const std::vector<Stamped<SE3>> saved_estimates() {
    std::vector<Stamped<SE3>> copy;
    copy.swap(saved_poses_);
    return copy;
  }

  const std::vector<Stamped<Map<>>> saved_features() {
    std::vector<Stamped<Map<>>> copy;
    copy.swap(saved_features_);
    return copy;
  }

  const std::vector<Stamped<Map<>>> saved_maps() {
    std::vector<Stamped<Map<>>> copy;
    copy.swap(saved_maps_);
    return copy;
  }

  // Save as Eigen matrices for easier use in Python
  const std::vector<Stamped<Map<Eigen::MatrixX3d>>> saved_features_cleaned() {
    return clean(saved_features());
  }

  const std::vector<Stamped<Map<Eigen::MatrixX3d>>> saved_maps_cleaned() {
    return clean(saved_maps());
  }

  void set_visualizing(const std::optional<std::set<VisOption>>& options) {
    vis_options_ = options;
  }

protected:
  // Helper to convert std::vector<Point> to Eigen matrices
  static inline std::vector<Stamped<Map<Eigen::MatrixX3d>>>
  clean(const std::vector<Stamped<Map<>>>& saved) {
    std::vector<Stamped<Map<Eigen::MatrixX3d>>> cleaned;
    // Iterate over stamps
    for (const auto& [stamp, original] : saved) {
      // Iterate over map entries
      Map<Eigen::MatrixX3d> eigen;
      for (const auto& [key, points] : original) {
        eigen[key] = convert(points);
      }
      cleaned.emplace_back(stamp, eigen);
    }
    return cleaned;
  }

private:
  std::vector<Stamped<SE3>> saved_poses_;
  std::vector<Stamped<Map<>>> saved_features_;
  std::vector<Stamped<Map<>>> saved_maps_;
  std::optional<std::set<VisOption>> vis_options_ = std::nullopt;
};

} // namespace evalio

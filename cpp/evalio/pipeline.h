#pragma once

#include <Eigen/Core>
#include <map>
#include <variant>

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

using Param = std::variant<bool, int, double, std::string>;

class PYBIND11_EXPORT Pipeline {
public:
  virtual ~Pipeline() {}

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

  static std::map<std::string, Param> default_params() {
    throw std::runtime_error("Not implemented");
  }

  // Getters
  virtual const SE3 pose() = 0;
  virtual const std::vector<Point> map() = 0;

  // Setters
  virtual void set_imu_params(ImuParams params) = 0;
  virtual void set_lidar_params(LidarParams params) = 0;
  virtual void set_imu_T_lidar(SE3 T) = 0;
  virtual void set_params(std::map<std::string, Param>) = 0;

  // Doers
  virtual void initialize() = 0;
  virtual void add_imu(ImuMeasurement mm) = 0;
  virtual std::vector<Point> add_lidar(LidarMeasurement mm) = 0;
};

} // namespace evalio

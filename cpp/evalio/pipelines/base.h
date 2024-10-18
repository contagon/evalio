#pragma once

#include <Eigen/Core>
#include <map>
#include <variant>

#include "evalio/types.h"

namespace evalio {

using Param = std::variant<bool, int, double, std::string>;

class Pipeline {
public:
  virtual ~Pipeline(){};

  // Info
  static std::string name() { throw std::runtime_error("Not implemented"); }
  static std::string url() { throw std::runtime_error("Not implemented"); }
  // TODO: Make this return a map filled with the default values
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
#pragma once

#include <Eigen/Core>
#include <map>

#include "types.h"

namespace evalio {

class Pipeline {
 public:
  virtual ~Pipeline() {};

  // Getters
  virtual const SE3 pose() = 0;
  virtual const std::vector<Point> map() = 0;

  // Setters
  virtual void set_imu_params(ImuParams params) = 0;
  virtual void set_lidar_params(LidarParams params) = 0;
  virtual void set_imu_T_lidar(SE3 T) = 0;

  virtual void set_param(std::string key, std::string value) = 0;
  virtual void set_param(std::string key, double value) = 0;
  virtual void set_param(std::string key, int value) = 0;
  virtual void set_param(std::string key, bool value) = 0;

  // Doers
  virtual void initialize() = 0;
  virtual void add_imu(ImuMeasurement mm) = 0;
  virtual void add_lidar(LidarMeasurement mm) = 0;
};

}  // namespace evalio
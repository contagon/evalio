#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>

namespace evalio {

struct Stamp {
  uint64_t nsecs;

  Stamp(uint64_t nsecs) : nsecs(nsecs) {}
};

struct Point {
  double x;
  double y;
  double z;
  double intensity;
  double offset;
  short unsigned int column;
  short unsigned int row;
};

struct LidarMeasurement {
  Stamp stamp;
  std::vector<Point> points;

  LidarMeasurement(Stamp stamp, std::vector<Point> points)
      : points(points), stamp(stamp) {}
};

struct LidarParams {
  // num_rows = num scan lines / channels / rings
  int num_rows;
  int num_columns;
  double min_range;
  double max_range;
};

struct ImuMeasurement {
  Stamp stamp;
  Eigen::Vector3d gyro;
  Eigen::Vector3d accel;
};

struct ImuParams {
  double gyro = 1e-5;
  double accel = 1e-5;
  double gyro_bias = 1e-6;
  double accel_bias = 1e-6;
  double bias_init = 1e-7;
  double integration = 1e-7;
  Eigen::Vector3d gravity;

  static ImuParams up() {
    ImuParams imu_params;
    imu_params.gravity = Eigen::Vector3d(0, 0, 9.81);
    return imu_params;
  }

  static ImuParams down() {
    ImuParams imu_params;
    imu_params.gravity = Eigen::Vector3d(0, 0, -9.81);
    return imu_params;
  }
};

struct SO3 {
  double qx;
  double qy;
  double qz;
  double qw;
};

struct SE3 {
  SO3 rot;
  Eigen::Vector3d trans;

  SE3(SO3 rot, Eigen::Vector3d trans) : rot(rot), trans(trans) {}
};

}  // namespace evalio
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace evalio {

struct Stamp {
  uint32_t sec;
  uint32_t nsec;

  std::string toString() const {
    return "Stamp(" + std::to_string(sec) + "." + std::to_string(nsec) + ")";
  }

  std::string toStringBrief() const {
    return std::to_string(sec) + "." + std::to_string(nsec);
  };
};

struct Point {
  double x;
  double y;
  double z;
  double intensity;
  uint32_t t;  // in nanoseconds?
  uint32_t range;
  uint8_t row;
  uint16_t col;

  std::string toString() const {
    return "Point(x: " + std::to_string(x) + ", y: " + std::to_string(y) +
           ", z: " + std::to_string(z) +
           ", intensity: " + std::to_string(intensity) +
           ", t: " + std::to_string(t) + ", row: " + std::to_string(row) +
           ", col: " + std::to_string(col) + ")";
  }
};

struct LidarMeasurement {
  Stamp stamp;
  std::vector<Point> points;

  LidarMeasurement(Stamp stamp) : stamp(stamp) {}

  LidarMeasurement(Stamp stamp, std::vector<Point> points)
      : points(points), stamp(stamp) {}

  std::string toString() const {
    std::ostringstream oss;
    oss << "LidarMeasurement(stamp: " << stamp.toStringBrief()
        << ", num_points: " << points.size() << ")";
    return oss.str();
  }
};

struct LidarParams {
  // num_rows = num scan lines / channels / rings
  int num_rows;
  int num_columns;
  double min_range;
  double max_range;

  std::string toString() const {
    return "LidarParams(rows: " + std::to_string(num_rows) +
           ", cols: " + std::to_string(num_columns) +
           ", min_range: " + std::to_string(min_range) +
           ", max_range: " + std::to_string(max_range) + ")";
  };
};

struct ImuMeasurement {
  Stamp stamp;
  Eigen::Vector3d gyro;
  Eigen::Vector3d accel;

  std::string toString() const {
    std::ostringstream oss;
    oss << "ImuMeasurement(stamp: " << stamp.toStringBrief() << ", gyro: ["
        << gyro.transpose() << "]"
        << ", accel: [" << accel.transpose() << "])";
    return oss.str();
  }
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

  std::string toString() const {
    std::ostringstream oss;
    oss << "ImuParams(gyro: " << gyro << ", accel: " << accel
        << ", gyro_bias: " << gyro_bias << ", accel_bias: " << accel_bias
        << ", bias_init: " << bias_init << ", integration: " << integration
        << ", gravity: [" << gravity.transpose() << "])";
    return oss.str();
  }
};

struct SO3 {
  double qx;
  double qy;
  double qz;
  double qw;

  std::string toString() const {
    return "SO3(x: " + std::to_string(qx) + ", y: " + std::to_string(qy) +
           ", z: " + std::to_string(qz) + ", w: " + std::to_string(qw) + ")";
  }

  std::string toStringBrief() const {
    return "x: " + std::to_string(qx) + ", y: " + std::to_string(qy) +
           ", z: " + std::to_string(qz) + ", w: " + std::to_string(qw);
  }
};

struct SE3 {
  SO3 rot;
  Eigen::Vector3d trans;

  SE3(SO3 rot, Eigen::Vector3d trans) : rot(rot), trans(trans) {}

  std::string toString() const {
    std::ostringstream oss;
    oss << "SE3(rot: [" << rot.toStringBrief() << "], "
        << "t: [" << trans.transpose() << "])";
    return oss.str();
  }
};

}  // namespace evalio
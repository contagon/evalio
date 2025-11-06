#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <iostream>
#include <vector>

namespace evalio {

struct Duration {
  // Also tried saving this in seconds, but found we had occasional floating
  // point errors when adding/subtracting durations.
  int64_t nsec;

  static Duration from_sec(double sec) {
    return Duration {.nsec = int64_t(sec * 1e9)};
  }

  static Duration from_nsec(int64_t nsec) {
    return Duration {.nsec = nsec};
  }

  double to_sec() const {
    return double(nsec) / 1e9;
  }

  int64_t to_nsec() const {
    return nsec;
  }

  std::string to_string() const {
    return "Duration(" + to_string_brief() + ")";
  }

  std::string to_string_brief() const {
    return std::to_string(to_sec());
  }

  bool operator<(const Duration& other) const {
    return nsec < other.nsec;
  }

  bool operator>(const Duration& other) const {
    return nsec > other.nsec;
  }

  bool operator==(const Duration& other) const {
    return nsec == other.nsec;
  }

  bool operator!=(const Duration& other) const {
    return !(*this == other);
  }

  Duration operator-(const Duration& other) const {
    return Duration::from_nsec(nsec - other.nsec);
  }

  Duration operator+(const Duration& other) const {
    return Duration::from_nsec(nsec + other.nsec);
  }
};

struct Stamp {
  uint32_t sec;
  uint32_t nsec;

  static Stamp from_sec(double sec) {
    return Stamp {
      .sec = uint32_t(sec),
      .nsec = uint32_t((sec - uint32_t(sec)) * 1e9)
    };
  }

  static Stamp from_nsec(uint64_t nsec) {
    return Stamp {
      .sec = uint32_t(nsec / 1e9),
      .nsec = uint32_t(nsec % uint64_t(1e9))
    };
  }

  uint64_t to_nsec() const {
    return uint64_t(sec) * uint64_t(1e9) + nsec;
  }

  double to_sec() const {
    return double(sec) + double(nsec) * 1e-9;
  }

  std::string to_string() const {
    return "Stamp(" + to_string_brief() + ")";
  }

  std::string to_string_brief() const {
    size_t n_zeros = 9;
    auto nsec_str = std::to_string(nsec);
    auto nsec_str_leading =
      std::string(9 - std::min(n_zeros, nsec_str.length()), '0') + nsec_str;
    return std::to_string(sec) + "." + nsec_str_leading;
  }

  bool operator<(const Stamp& other) const {
    return sec < other.sec || (sec == other.sec && nsec < other.nsec);
  }

  bool operator>(const Stamp& other) const {
    return sec > other.sec || (sec == other.sec && nsec > other.nsec);
  }

  bool operator==(const Stamp& other) const {
    return sec == other.sec && nsec == other.nsec;
  }

  bool operator!=(const Stamp& other) const {
    return !(*this == other);
  }

  Stamp operator-(const Duration& other) const {
    return Stamp::from_nsec(to_nsec() - other.nsec);
  }

  Stamp operator+(const Duration& other) const {
    return Stamp::from_nsec(to_nsec() + other.nsec);
  }

  Duration operator-(const Stamp& other) const {
    return Duration::from_sec(to_sec() - other.to_sec());
  }
};

struct Point {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double intensity = 0.0;
  Duration t = Duration {.nsec = 0};
  uint32_t range = 0;
  uint8_t row = 0;
  uint16_t col = 0;

  std::string to_string() const {
    return "Point(x: " + std::to_string(x) + ", y: " + std::to_string(y)
      + ", z: " + std::to_string(z) + ", intensity: "
      + std::to_string(intensity) + ", t: " + std::to_string(t.to_sec())
      + ", row: " + std::to_string(row) + ", col: " + std::to_string(col) + ")";
  }

  bool operator!=(const Point& other) const {
    return !(*this == other);
  }

  bool operator==(const Point& other) const {
    return x == other.x && y == other.y && z == other.z
      && intensity == other.intensity && t == other.t && range == other.range
      && row == other.row && col == other.col;
  }
};

struct LidarMeasurement {
  Stamp stamp;
  std::vector<Point> points;

  LidarMeasurement(Stamp stamp) : stamp(stamp) {}

  LidarMeasurement(Stamp stamp, std::vector<Point> points) :
    stamp(stamp), points(points) {}

  std::string to_string() const {
    std::ostringstream oss;
    oss << "LidarMeasurement(stamp: " << stamp.to_string_brief()
        << ", num_points: " << points.size() << ")";
    return oss.str();
  }

  std::vector<Eigen::Vector3d> to_vec_positions() const {
    std::vector<Eigen::Vector3d> eigen_points;
    eigen_points.reserve(points.size());
    for (const auto& point : points) {
      eigen_points.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }
    return eigen_points;
  }

  std::vector<double> to_vec_stamps() const {
    std::vector<double> vec_stamps;
    vec_stamps.reserve(points.size());
    for (const auto& point : points) {
      vec_stamps.push_back(point.t.to_sec());
    }
    return vec_stamps;
  }

  bool operator!=(const LidarMeasurement& other) const {
    return !(*this == other);
  }

  bool operator==(const LidarMeasurement& other) const {
    if (stamp != other.stamp || points.size() != other.points.size()) {
      return false;
    }
    for (size_t i = 0; i < points.size(); ++i) {
      if (points[i] != other.points[i]) {
        return false;
      }
    }
    return true;
  }
};

struct LidarParams {
  // num_rows = num scan lines / channels / rings
  int num_rows;
  int num_columns;
  // in meters
  double min_range;
  double max_range;
  // in Hz
  double rate = 10.0;
  std::string brand = "-";
  std::string model = "-";

  std::string to_string() const {
    return "LidarParams(rows: " + std::to_string(num_rows)
      + ", cols: " + std::to_string(num_columns) + ", min_range: "
      + std::to_string(min_range) + ", max_range: " + std::to_string(max_range)
      + ", rate: " + std::to_string(rate) + ")";
  }

  Duration delta_time() const {
    return Duration::from_sec(1.0 / rate);
  }
};

struct ImuMeasurement {
  Stamp stamp;
  Eigen::Vector3d gyro;
  Eigen::Vector3d accel;

  std::string to_string() const {
    std::ostringstream oss;
    oss << "ImuMeasurement(stamp: " << stamp.to_string_brief() << ", gyro: ["
        << gyro.transpose() << "]"
        << ", accel: [" << accel.transpose() << "])";
    return oss.str();
  }

  bool operator!=(const ImuMeasurement& other) const {
    return !(*this == other);
  }

  bool operator==(const ImuMeasurement& other) const {
    return stamp == other.stamp && gyro == other.gyro && accel == other.accel;
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
  double rate = 100.0;
  std::string brand = "-";
  std::string model = "-";

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

  std::string to_string() const {
    std::ostringstream oss;
    oss << "ImuParams(gyro: " << gyro << ", accel: " << accel
        << ", gyro_bias: " << gyro_bias << ", accel_bias: " << accel_bias
        << ", bias_init: " << bias_init << ", integration: " << integration
        << ", gravity: [" << gravity.transpose() << "], " << "rate: " << rate
        << ")";
    return oss.str();
  }
};

struct SO3 {
  double qx;
  double qy;
  double qz;
  double qw;

  Eigen::Quaterniond to_eigen() const {
    return Eigen::Quaterniond(qw, qx, qy, qz);
  }

  static SO3 from_eigen(const Eigen::Quaterniond& q) {
    return SO3 {.qx = q.x(), .qy = q.y(), .qz = q.z(), .qw = q.w()};
  }

  static SO3 identity() {
    return SO3 {.qx = 0, .qy = 0, .qz = 0, .qw = 1};
  }

  static SO3 from_mat(const Eigen::Matrix3d& R) {
    return from_eigen(Eigen::Quaterniond(R));
  }

  SO3 inverse() const {
    return SO3 {.qx = -qx, .qy = -qy, .qz = -qz, .qw = qw};
  }

  SO3 operator*(const SO3& other) const {
    return from_eigen(to_eigen() * other.to_eigen());
  }

  Eigen::Vector3d rotate(const Eigen::Vector3d& v) const {
    return to_eigen() * v;
  }

  static Eigen::Matrix3d hat(const Eigen::Vector3d& xi) {
    Eigen::Matrix3d Omega;
    Omega << 0, -xi.z(), xi.y(), xi.z(), 0, -xi.x(), -xi.y(), xi.x(), 0;
    return Omega;
  }

  static SO3 exp(const Eigen::Vector3d& v) {
    Eigen::AngleAxisd axis(v.norm(), v.normalized());
    Eigen::Quaterniond q(axis);
    return from_eigen(q);
  }

  Eigen::Vector3d log() const {
    Eigen::Quaterniond q = to_eigen();
    auto axis = Eigen::AngleAxisd(q);
    return axis.angle() * axis.axis();
  }

  Eigen::Matrix3d to_mat() const {
    return to_eigen().toRotationMatrix();
  }

  std::string to_string() const {
    return "SO3(x: " + std::to_string(qx) + ", y: " + std::to_string(qy)
      + ", z: " + std::to_string(qz) + ", w: " + std::to_string(qw) + ")";
  }

  std::string to_string_brief() const {
    return "x: " + std::to_string(qx) + ", y: " + std::to_string(qy)
      + ", z: " + std::to_string(qz) + ", w: " + std::to_string(qw);
  }

  bool operator==(const SO3& other) const {
    return qx == other.qx && qy == other.qy && qz == other.qz && qw == other.qw;
  }

  bool operator!=(const SO3& other) const {
    return !(*this == other);
  }
};

struct SE3 {
  SO3 rot;
  Eigen::Vector3d trans;

  SE3(SO3 rot, Eigen::Vector3d trans) : rot(rot), trans(trans) {}

  static SE3 identity() {
    return SE3(SO3::identity(), Eigen::Vector3d::Zero());
  }

  static SE3 from_mat(const Eigen::Matrix4d& T) {
    return SE3(SO3::from_mat(T.block<3, 3>(0, 0)), T.block<3, 1>(0, 3));
  }

  Eigen::Matrix4d to_mat() const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = rot.to_mat();
    T.block<3, 1>(0, 3) = trans;
    return T;
  }

  SE3 inverse() const {
    const auto inv_rot = rot.inverse();
    return SE3(inv_rot, inv_rot.rotate(-trans));
  }

  static SE3 exp(const Eigen::Matrix<double, 6, 1>& xi) {
    Eigen::Vector3d omega = xi.head<3>();
    Eigen::Vector3d xyz = xi.tail<3>();

    double theta2 = omega.squaredNorm();
    double B, C;
    if (theta2 < 1e-5) {
      B = 0.5;
      C = 1.0 / 6.0;
    } else {
      double theta = std::sqrt(theta2);
      double A = std::sin(theta) / theta;
      B = (1.0 - std::cos(theta)) / theta2;
      C = (1.0 - A) / theta2;
    }

    SO3 R = SO3::exp(omega);
    Eigen::Matrix3d Omega = SO3::hat(omega);
    Eigen::Matrix3d V =
      Eigen::Matrix3d::Identity() + B * Omega + C * Omega * Omega;

    return SE3(R, V * xyz);
  }

  Eigen::Matrix<double, 6, 1> log() const {
    Eigen::Vector3d omega = rot.log();

    double theta2 = omega.squaredNorm();
    double B, C;
    if (theta2 < 1e-5) {
      B = 0.5;
      C = 1.0 / 6.0;
    } else {
      double theta = std::sqrt(theta2);
      double A = std::sin(theta) / theta;
      B = (1.0 - std::cos(theta)) / theta2;
      C = (1.0 - A) / theta2;
    }

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d wx = SO3::hat(omega);
    Eigen::Matrix3d V = I + B * wx + C * wx * wx;

    Eigen::Vector3d xyz = V.inverse() * trans;
    Eigen::Matrix<double, 6, 1> xi;
    xi << omega, xyz;

    return xi;
  }

  SE3 operator*(const SE3& other) const {
    return SE3(rot * other.rot, rot.rotate(other.trans) + trans);
  }

  std::string to_string() const {
    std::ostringstream oss;
    oss << "SE3(rot: [" << rot.to_string_brief() << "], "
        << "t: [" << trans.transpose() << "])";
    return oss.str();
  }

  bool operator==(const SE3& other) const {
    return rot == other.rot && trans == other.trans;
  }

  bool operator!=(const SE3& other) const {
    return !(*this == other);
  }

  // Helpers for stats computations
  static std::pair<double, double> error(const SE3& a, const SE3& b) {
    auto delta = a.inverse() * b;
    double rot_err = delta.rot.log().norm() * (180.0 / M_PI);
    double trans_err = (delta.trans).norm();
    return {rot_err, trans_err};
  }

  static double distance(const SE3& a, const SE3& b) {
    return (a.trans - b.trans).norm();
  }
};

} // namespace evalio

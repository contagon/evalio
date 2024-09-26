// TODO: Make some basic versions of the following types
// - stamp
// - point type
// - pose3
// - imu measurement
// - lidar measurement
// - preint noise

#pragma once

#include <cstdint>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace evalio {

struct Stamp {
    uint64_t nsecs;
};

struct Point {
    double x;
    double y;
    double z;
    double intensity;
    double offset;
    short unsigned int channel;
    short unsigned int ring;
};

struct LidarMeasurement {
    std::vector<Point> points;
    Stamp stamp;
};

struct ImuMeasurement {
    Eigen::Vector3d gyro;
    Eigen::Vector3d accel;
    Stamp stamp;
};

struct SE3 {
    double qx;
    double qy;
    double qz;
    double qw;
    Eigen::Vector3d trans;
};

struct PreintNoise {
    double gyro;
    double accel;
    double gyro_bias;
    double accel_bias;
    double bias_init;
    double integration;
    Eigen::Vector3d gravity;
};

} // namespace evalio
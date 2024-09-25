#pragma once

#include <map>

#include <gtsam/geometry/Pose3.h>

class Pipeline {
    public:
        virtual ~Pipeline() {};

        virtual const gtsam::Pose3 pose() = 0;

        virtual const std::vector<Eigen::Vector4d> map() = 0;

        virtual void add_imu(Eigen::Vector3d gyro, Eigen::Vector3d acc, uint64_t timestamp) = 0;

        virtual void add_lidar(std::vector<Eigen::Vector4d> points, uint64_t timestamp) = 0;

        virtual void set_param(std::string key, std::string value) = 0;

        virtual void set_param(std::string key, double value) = 0;

        virtual void set_param(std::string key, int value) = 0;

        virtual void set_param(std::string key, bool value) = 0;
};
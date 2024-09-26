#pragma once

#include <map>
#include <Eigen/Core>

#include "types.h"

namespace evalio {

class Pipeline {
    public:
        virtual ~Pipeline() {};

        virtual const SE3 pose() = 0;

        virtual const std::vector<Point> map() = 0;

        virtual void add_imu(ImuMeasurement mm) = 0;

        virtual void add_lidar(LidarMeasurement mm) = 0;

        virtual void set_param(std::string key, std::string value) = 0;

        virtual void set_param(std::string key, double value) = 0;

        virtual void set_param(std::string key, int value) = 0;

        virtual void set_param(std::string key, bool value) = 0;
};

} // namespace evalio
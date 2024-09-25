#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pipelines/base.h"
#include "pipelines/kiss_icp.h"

namespace py = pybind11;
using namespace pybind11::literals;

class PyPipeline: public Pipeline {
    public:
        using Pipeline::Pipeline;

        const gtsam::Pose3 pose() override {
            PYBIND11_OVERRIDE_PURE(const gtsam::Pose3, Pipeline, pose);
        }

        const std::vector<Eigen::Vector4d> map() override {
            PYBIND11_OVERRIDE_PURE(const std::vector<Eigen::Vector4d>, Pipeline, map);
        }

        void add_imu(Eigen::Vector3d gyro, Eigen::Vector3d acc, uint64_t timestamp) override {
            PYBIND11_OVERRIDE_PURE(void, Pipeline, add_imu, gyro, acc, timestamp);
        }

        void add_lidar(std::vector<Eigen::Vector4d> points, uint64_t timestamp) override {
            PYBIND11_OVERRIDE_PURE(void, Pipeline, add_lidar, points, timestamp);
        }

        void set_param(std::string key, std::string value) override {
            PYBIND11_OVERRIDE_PURE(void, Pipeline, set_param, key, value);
        }

        void set_param(std::string key, double value) override {
            PYBIND11_OVERRIDE_PURE(void, Pipeline, set_param, key, value);
        }

        void set_param(std::string key, int value) override {
            PYBIND11_OVERRIDE_PURE(void, Pipeline, set_param, key, value);
        }

        void set_param(std::string key, bool value) override {
            PYBIND11_OVERRIDE_PURE(void, Pipeline, set_param, key, value);
        }
};

PYBIND11_MODULE(evalio_python, m){
    py::class_<Pipeline, PyPipeline>(m, "Pipeline")
        .def(py::init<>())
        .def("pose", &Pipeline::pose)
        .def("map", &Pipeline::map)
        .def("add_imu", &Pipeline::add_imu)
        .def("add_lidar", &Pipeline::add_lidar)
        .def("set_param", py::overload_cast<std::string, std::string>(&Pipeline::set_param))
        .def("set_param", py::overload_cast<std::string, double>(&Pipeline::set_param))
        .def("set_param", py::overload_cast<std::string, int>(&Pipeline::set_param))
        .def("set_param", py::overload_cast<std::string, bool>(&Pipeline::set_param));

    py::class_<KissICP, Pipeline>(m, "KissICP");
}
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pipelines/base.h"
#include "pipelines/kiss_icp.h"

namespace py = pybind11;
using namespace pybind11::literals;

class PyPipeline: public evalio::Pipeline {
    public:
        using evalio::Pipeline::Pipeline;

        const evalio::SE3 pose() override {
            PYBIND11_OVERRIDE_PURE(const evalio::SE3, evalio::Pipeline, pose);
        }

        const std::vector<evalio::Point> map() override {
            PYBIND11_OVERRIDE_PURE(const std::vector<evalio::Point>, evalio::Pipeline, map);
        }

        void add_imu(evalio::ImuMeasurement mm) override {
            PYBIND11_OVERRIDE_PURE(void, evalio::Pipeline, add_imu, mm);
        }

        void add_lidar(evalio::LidarMeasurement mm) override {
            PYBIND11_OVERRIDE_PURE(void, evalio::Pipeline, add_lidar, mm);
        }

        void set_param(std::string key, std::string value) override {
            PYBIND11_OVERRIDE_PURE(void, evalio::Pipeline, set_param, key, value);
        }

        void set_param(std::string key, double value) override {
            PYBIND11_OVERRIDE_PURE(void, evalio::Pipeline, set_param, key, value);
        }

        void set_param(std::string key, int value) override {
            PYBIND11_OVERRIDE_PURE(void, evalio::Pipeline, set_param, key, value);
        }

        void set_param(std::string key, bool value) override {
            PYBIND11_OVERRIDE_PURE(void, evalio::Pipeline, set_param, key, value);
        }
};

PYBIND11_MODULE(_cpp, m){
    // ------------------------- Types for converting back and forth ------------------------- //
    // TODO: How will these constructors work?
    py::class_<evalio::Stamp>(m, "Stamp")
        .def(py::init<>())
        .def_readwrite("nsecs", &evalio::Stamp::nsecs);

    py::class_<evalio::Point>(m, "Point")
        .def(py::init<>())
        .def_readwrite("x", &evalio::Point::x)
        .def_readwrite("y", &evalio::Point::y)
        .def_readwrite("z", &evalio::Point::z)
        .def_readwrite("intensity", &evalio::Point::intensity)
        .def_readwrite("offset", &evalio::Point::offset)
        .def_readwrite("channel", &evalio::Point::channel)
        .def_readwrite("ring", &evalio::Point::ring);

    py::class_<evalio::LidarMeasurement>(m, "LidarMeasurement")
        .def(py::init<>())
        .def_readwrite("points", &evalio::LidarMeasurement::points)
        .def_readwrite("stamp", &evalio::LidarMeasurement::stamp);

    py::class_<evalio::ImuMeasurement>(m, "ImuMeasurement")
        .def(py::init<>())
        .def_readwrite("gyro", &evalio::ImuMeasurement::gyro)
        .def_readwrite("accel", &evalio::ImuMeasurement::accel)
        .def_readwrite("stamp", &evalio::ImuMeasurement::stamp);

    py::class_<evalio::SE3>(m, "SE3")
        .def(py::init<>())
        .def_readwrite("qx", &evalio::SE3::qx)
        .def_readwrite("qy", &evalio::SE3::qy)
        .def_readwrite("qz", &evalio::SE3::qz)
        .def_readwrite("qw", &evalio::SE3::qw)
        .def_readwrite("trans", &evalio::SE3::trans);

    py::class_<evalio::PreintNoise>(m, "PreintNoise")
        .def(py::init<>())
        .def_readwrite("gyro", &evalio::PreintNoise::gyro)
        .def_readwrite("accel", &evalio::PreintNoise::accel)
        .def_readwrite("gyro_bias", &evalio::PreintNoise::gyro_bias)
        .def_readwrite("accel_bias", &evalio::PreintNoise::accel_bias)
        .def_readwrite("bias_init", &evalio::PreintNoise::bias_init)
        .def_readwrite("integration", &evalio::PreintNoise::integration)
        .def_readwrite("gravity", &evalio::PreintNoise::gravity);

    // ------------------------- Pipelines ------------------------- //
    py::class_<evalio::Pipeline, PyPipeline>(m, "Pipeline")
        .def(py::init<>())
        .def("pose", &evalio::Pipeline::pose)
        .def("map", &evalio::Pipeline::map)
        .def("add_imu", &evalio::Pipeline::add_imu)
        .def("add_lidar", &evalio::Pipeline::add_lidar)
        .def("set_param", py::overload_cast<std::string, std::string>(&evalio::Pipeline::set_param))
        .def("set_param", py::overload_cast<std::string, double>(&evalio::Pipeline::set_param))
        .def("set_param", py::overload_cast<std::string, int>(&evalio::Pipeline::set_param))
        .def("set_param", py::overload_cast<std::string, bool>(&evalio::Pipeline::set_param));

    py::class_<KissICP, evalio::Pipeline>(m, "KissICP");
}
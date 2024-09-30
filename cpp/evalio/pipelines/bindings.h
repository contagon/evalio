#pragma once

#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "evalio/pipelines/base.h"
#include "evalio/pipelines/kiss_icp.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace evalio {

class PyPipeline : public evalio::Pipeline {
 public:
  using evalio::Pipeline::Pipeline;

  // Info
  // static std::string name() override {
  //   PYBIND11_OVERRIDE(std::string, evalio::Pipeline, name);
  // }
  // static std::string nickname() override {
  //   PYBIND11_OVERRIDE(std::string, evalio::Pipeline, nickname);
  // }

  // Getters
  const evalio::SE3 pose() override {
    PYBIND11_OVERRIDE_PURE(const evalio::SE3, evalio::Pipeline, pose);
  }
  const std::vector<evalio::Point> map() override {
    PYBIND11_OVERRIDE_PURE(const std::vector<evalio::Point>, evalio::Pipeline,
                           map);
  }

  // Setters
  void set_imu_params(evalio::ImuParams params) override {
    PYBIND11_OVERRIDE_PURE(void, evalio::Pipeline, set_imu_params, params);
  }
  void set_lidar_params(evalio::LidarParams params) override {
    PYBIND11_OVERRIDE_PURE(void, evalio::Pipeline, set_lidar_params, params);
  }
  void set_imu_T_lidar(evalio::SE3 T) override {
    PYBIND11_OVERRIDE_PURE(void, evalio::Pipeline, set_imu_T_lidar, T);
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

  // Doers
  void initialize() override {
    PYBIND11_OVERRIDE_PURE(void, evalio::Pipeline, initialize);
  }
  void add_imu(evalio::ImuMeasurement mm) override {
    PYBIND11_OVERRIDE_PURE(void, evalio::Pipeline, add_imu, mm);
  }
  void add_lidar(evalio::LidarMeasurement mm) override {
    PYBIND11_OVERRIDE_PURE(void, evalio::Pipeline, add_lidar, mm);
  }
};

void makePipelines(py::module& m) {
  py::class_<evalio::Pipeline, PyPipeline>(m, "Pipeline")
      .def(py::init<>())
      .def_static("name", &evalio::Pipeline::name)
      .def_static("nickname", &evalio::Pipeline::nickname)
      .def("pose", &evalio::Pipeline::pose)
      .def("map", &evalio::Pipeline::map)
      .def("initialize", &evalio::Pipeline::initialize)
      .def("add_imu", &evalio::Pipeline::add_imu)
      .def("add_lidar", &evalio::Pipeline::add_lidar)
      .def("set_param", py::overload_cast<std::string, std::string>(
                            &evalio::Pipeline::set_param))
      .def("set_param",
           py::overload_cast<std::string, double>(&evalio::Pipeline::set_param))
      .def("set_param",
           py::overload_cast<std::string, int>(&evalio::Pipeline::set_param))
      .def("set_param",
           py::overload_cast<std::string, bool>(&evalio::Pipeline::set_param))
      .def("set_imu_params", &evalio::Pipeline::set_imu_params)
      .def("set_lidar_params", &evalio::Pipeline::set_lidar_params)
      .def("set_imu_T_lidar", &evalio::Pipeline::set_imu_T_lidar);

  // List all the pipelines here
  py::class_<KissICP, evalio::Pipeline>(m, "KissICP")
      .def(py::init<>())
      .def_static("name", &KissICP::name)
      .def_static("nickname", &KissICP::nickname);
}

}  // namespace evalio
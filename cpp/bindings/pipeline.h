#pragma once

#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "evalio/pipeline.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace evalio {

class PyPipeline : public evalio::Pipeline {
public:
  using evalio::Pipeline::Pipeline;

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
  void set_params(std::map<std::string, Param> params) override {
    PYBIND11_OVERRIDE_PURE(void, evalio::Pipeline, set_params, params);
  }

  // Doers
  void initialize() override {
    PYBIND11_OVERRIDE_PURE(void, evalio::Pipeline, initialize);
  }
  void add_imu(evalio::ImuMeasurement mm) override {
    PYBIND11_OVERRIDE_PURE(void, evalio::Pipeline, add_imu, mm);
  }
  std::vector<Point> add_lidar(evalio::LidarMeasurement mm) override {
    PYBIND11_OVERRIDE_PURE(std::vector<Point>, evalio::Pipeline, add_lidar, mm);
  }
};

inline void makeBasePipeline(py::module &m) {
  py::class_<evalio::Pipeline, PyPipeline>(m, "Pipeline")
      .def(py::init<>())
      .def_static("name", &evalio::Pipeline::name)
      .def_static("url", &evalio::Pipeline::url)
      .def_static("default_params", &evalio::Pipeline::default_params)
      .def("pose", &evalio::Pipeline::pose)
      .def("map", &evalio::Pipeline::map)
      .def("initialize", &evalio::Pipeline::initialize)
      .def("add_imu", &evalio::Pipeline::add_imu)
      .def("add_lidar", &evalio::Pipeline::add_lidar)
      .def("set_params", &evalio::Pipeline::set_params)
      .def("set_imu_params", &evalio::Pipeline::set_imu_params)
      .def("set_lidar_params", &evalio::Pipeline::set_lidar_params)
      .def("set_imu_T_lidar", &evalio::Pipeline::set_imu_T_lidar);
}

} // namespace evalio
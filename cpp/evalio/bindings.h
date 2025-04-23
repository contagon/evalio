#pragma once

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>

#include "evalio/pipeline.h"

namespace nb = nanobind;
using namespace nb::literals;

namespace evalio {

class PyPipeline : public evalio::Pipeline {
public:
  NB_TRAMPOLINE(Pipeline, 9);

  // Getters
  const evalio::SE3 pose() override { NB_OVERRIDE_PURE(pose); }
  const std::vector<evalio::Point> map() override { NB_OVERRIDE_PURE(map); }

  // Setters
  void set_imu_params(evalio::ImuParams params) override {
    NB_OVERRIDE_PURE(set_imu_params, params);
  }
  void set_lidar_params(evalio::LidarParams params) override {
    NB_OVERRIDE_PURE(set_lidar_params, params);
  }
  void set_imu_T_lidar(evalio::SE3 T) override {
    NB_OVERRIDE_PURE(set_imu_T_lidar, T);
  }
  void set_params(std::map<std::string, Param> params) override {
    NB_OVERRIDE_PURE(set_params, params);
  }

  // Doers
  void initialize() override { NB_OVERRIDE_PURE(initialize); }
  void add_imu(evalio::ImuMeasurement mm) override {
    NB_OVERRIDE_PURE(add_imu, mm);
  }
  std::vector<Point> add_lidar(evalio::LidarMeasurement mm) override {
    NB_OVERRIDE_PURE(add_lidar, mm);
  }
};

inline void makeBasePipeline(nb::module_ &m) {
  nb::class_<evalio::Pipeline, PyPipeline>(m, "Pipeline")
      .def(nb::init<>())
      .def_static("name", &evalio::Pipeline::name)
      .def_static("url", &evalio::Pipeline::url)
      .def_static("default_params", &evalio::Pipeline::default_params)
      .def("pose", &evalio::Pipeline::pose)
      .def("map", &evalio::Pipeline::map)
      .def("initialize", &evalio::Pipeline::initialize)
      .def("add_imu", &evalio::Pipeline::add_imu, "mm"_a)
      .def("add_lidar", &evalio::Pipeline::add_lidar, "mm"_a)
      .def("set_params", &evalio::Pipeline::set_params, "params"_a)
      .def("set_imu_params", &evalio::Pipeline::set_imu_params, "params"_a)
      .def("set_lidar_params", &evalio::Pipeline::set_lidar_params, "params"_a)
      .def("set_imu_T_lidar", &evalio::Pipeline::set_imu_T_lidar, "T"_a);
}

// inline void setup(nb::module_ &m) {
//   // NOTE: typeid are not guaranteed to be unique across building from
//   different
//   // compilers
//   // https://github.com/pybind/pybind11/issues/877#issuecomment-304464896
//   // Ideally, if built using the same compiler as evalio (unlikely), this
//   should
//   // work import the original wrapper for Pipeline
//   nb::module evalio = nb::module::import("evalio");
//   // Otherwise, build a new pipeline base class
//   // This should be fine, as pipelines are all exclusively used in python
//   if (nb::detail::get_type_info(typeid(evalio::Pipeline)) == nullptr) {
//     evalio::makeBasePipeline(m);
//   };
// }

} // namespace evalio
#pragma once

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>

#include "evalio/pipeline.h"

namespace nb = nanobind;
using namespace nb::literals;

namespace evalio {

class PyPipeline: public evalio::Pipeline {
public:
  NB_TRAMPOLINE(Pipeline, 9);

  // Getters
  const std::map<std::string, std::vector<evalio::Point>> map() override {
    NB_OVERRIDE_PURE(map);
  }

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

  std::map<std::string, Param>
  set_params(std::map<std::string, Param> params) override {
    NB_OVERRIDE_PURE(set_params, params);
  }

  // Doers
  void initialize() override {
    NB_OVERRIDE_PURE(initialize);
  }

  void add_imu(evalio::ImuMeasurement mm) override {
    NB_OVERRIDE_PURE(add_imu, mm);
  }

  void add_lidar(evalio::LidarMeasurement mm) override {
    NB_OVERRIDE_PURE(add_lidar, mm);
  }
}; // namespace evalio

inline void makeBasePipeline(nb::module_& m) {
  nb::class_<evalio::Pipeline, PyPipeline>(m, "Pipeline")
    .def(nb::init<>(), "Construct a new pipeline.")
    .def_static("name", &evalio::Pipeline::name, "Name of the pipeline.")
    .def_static(
      "default_params",
      &evalio::Pipeline::default_params,
      "Default parameters for the pipeline."
    )
    .def_static(
      "url",
      &evalio::Pipeline::url,
      "URL for more information about the pipeline."
    )
    .def_static(
      "version",
      &evalio::Pipeline::version,
      "Version of the pipeline."
    )
    .def(
      "saved_estimates",
      &evalio::Pipeline::saved_estimates,
      "Most recent stamped pose estimates. Will be removed after this method is called."
    )
    .def(
      "saved_features",
      &evalio::Pipeline::saved_features,
      "Most recent stamped feature scans. Will be removed after this method is called."
    )
    .def(
      "saved_maps",
      &evalio::Pipeline::saved_maps,
      "Most recent stamped maps. Will be removed after this method is called."
    )
    .def(
      "saved_features_matrix",
      &evalio::Pipeline::saved_features_matrix,
      "Most recent stamped feature scans as matrices. Will be removed after this method is called."
    )
    .def(
      "saved_maps_matrix",
      &evalio::Pipeline::saved_maps_matrix,
      "Most recent stamped maps as matrices. Will be removed after this method is called."
    )
    .def("map", &evalio::Pipeline::map, "Map of the environment.")
    .def(
      "initialize",
      &evalio::Pipeline::initialize,
      "Initialize the pipeline. Must be called after constructing the "
      "object and before setting parameters."
    )
    .def(
      "add_imu",
      &evalio::Pipeline::add_imu,
      "mm"_a,
      "Register an IMU measurement."
    )
    .def(
      "add_lidar",
      &evalio::Pipeline::add_lidar,
      "mm"_a,
      "Register a LiDAR measurement."
    )
    .def(
      "set_params",
      &evalio::Pipeline::set_params,
      "params"_a,
      "Set parameters for the pipeline. This will override any default "
      "parameters."
    )
    .def(
      "set_imu_params",
      &evalio::Pipeline::set_imu_params,
      "params"_a,
      "Set IMU parameters for the pipeline."
    )
    .def(
      "set_lidar_params",
      &evalio::Pipeline::set_lidar_params,
      "params"_a,
      "Set LiDAR parameters for the pipeline."
    )
    .def(
      "set_imu_T_lidar",
      &evalio::Pipeline::set_imu_T_lidar,
      "T"_a,
      "Set the transformation from IMU to LiDAR frame."
    )
    .def(
      "set_visualizing",
      &evalio::Pipeline::set_visualizing,
      "visualize"_a.none(),
      "Enable or disable visualization."
    )
    .def(
      "save",
      nb::overload_cast<const evalio::Stamp&, const evalio::SE3&>(
        &evalio::Pipeline::save<const evalio::SE3&>
      ),
      "stamp"_a,
      "pose"_a,
      "Save the current pose estimate at the given timestamp."
    )
    // Use a lambda here, as nb::overload_cast has issues finding the non-template methods
    .def(
      "save",
      [](Pipeline& self, const Stamp& stamp, const Map<>& features) {
        self.save(stamp, features);
      },
      "stamp"_a,
      "features"_a,
      "Save the current feature map at the given timestamp."
    )
    .doc() =
    "Base class for all pipelines. This class defines the interface "
    "for interacting with pipelines, and is intended to be "
    "subclassed by specific implementations.";
}

} // namespace evalio

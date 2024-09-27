#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pipelines/base.h"
#include "pipelines/kiss_icp.h"

namespace py = pybind11;
using namespace pybind11::literals;

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

PYBIND11_MODULE(_cpp, m) {
  // ----------------- Types for converting back and forth ----------------- //
  py::class_<evalio::Stamp>(m, "Stamp")
      .def(py::init<uint32_t, uint32_t>(), py::kw_only(), "sec"_a, "nsec"_a)
      .def_readonly("sec", &evalio::Stamp::sec)
      .def_readonly("nsec", &evalio::Stamp::nsec)
      .def("__repr__", &evalio::Stamp::toString);

  // Lidar
  py::class_<evalio::Point>(m, "Point")
      .def(py::init<double, double, double, double, double, short unsigned int,
                    short unsigned int>(),
           py::kw_only(), "x"_a, "y"_a, "z"_a, "intensity"_a, "stamp_offset"_a,
           "row"_a, "column"_a)
      .def_readonly("x", &evalio::Point::x)
      .def_readonly("y", &evalio::Point::y)
      .def_readonly("z", &evalio::Point::z)
      .def_readonly("intensity", &evalio::Point::intensity)
      .def_readonly("stamp_offset", &evalio::Point::stamp_offset)
      .def_readonly("row", &evalio::Point::row)
      .def_readonly("column", &evalio::Point::column);

  py::class_<evalio::LidarMeasurement>(m, "LidarMeasurement")
      .def(py::init<evalio::Stamp, std::vector<evalio::Point>>(), "stamp"_a,
           "points"_a)
      .def_readonly("stamp", &evalio::LidarMeasurement::stamp)
      .def_readonly("points", &evalio::LidarMeasurement::points);

  py::class_<evalio::LidarParams>(m, "LidarParams")
      .def(py::init<int, int, double, double>(), py::kw_only(), "num_rows"_a,
           "num_columns"_a, "min_range"_a, "max_range"_a)
      .def_readonly("num_rows", &evalio::LidarParams::num_rows)
      .def_readonly("num_columns", &evalio::LidarParams::num_columns)
      .def_readonly("min_range", &evalio::LidarParams::min_range)
      .def_readonly("max_range", &evalio::LidarParams::max_range);

  // Imu
  py::class_<evalio::ImuMeasurement>(m, "ImuMeasurement")
      .def(py::init<evalio::Stamp, Eigen::Vector3d, Eigen::Vector3d>(),
           "stamp"_a, "gyro"_a, "accel"_a)
      .def_readonly("stamp", &evalio::ImuMeasurement::stamp)
      .def_readonly("gyro", &evalio::ImuMeasurement::gyro)
      .def_readonly("accel", &evalio::ImuMeasurement::accel)
      .def("__repr__", &evalio::ImuMeasurement::toString);

  py::class_<evalio::ImuParams>(m, "ImuParams")
      .def_static("up", &evalio::ImuParams::up)
      .def_static("down", &evalio::ImuParams::down)
      .def_readwrite("gyro", &evalio::ImuParams::gyro)
      .def_readwrite("accel", &evalio::ImuParams::accel)
      .def_readwrite("gyro_bias", &evalio::ImuParams::gyro_bias)
      .def_readwrite("accel_bias", &evalio::ImuParams::accel_bias)
      .def_readwrite("bias_init", &evalio::ImuParams::bias_init)
      .def_readwrite("integration", &evalio::ImuParams::integration)
      .def_readwrite("gravity", &evalio::ImuParams::gravity);

  py::class_<evalio::SO3>(m, "SO3")
      .def(py::init<double, double, double, double>(), py::kw_only(), "qx"_a,
           "qy"_a, "qz"_a, "qw"_a)
      .def_readonly("qx", &evalio::SO3::qx)
      .def_readonly("qy", &evalio::SO3::qy)
      .def_readonly("qz", &evalio::SO3::qz)
      .def_readonly("qw", &evalio::SO3::qw)
      .def("__repr__", &evalio::SO3::toString);

  py::class_<evalio::SE3>(m, "SE3")
      .def(py::init<evalio::SO3, Eigen::Vector3d>(), "rot"_a, "trans"_a)
      .def_readonly("rot", &evalio::SE3::rot)
      .def_readonly("trans", &evalio::SE3::trans)
      .def("__repr__", &evalio::SE3::toString);

  // ------------------------- Pipelines ------------------------- //
  py::class_<evalio::Pipeline, PyPipeline>(m, "Pipeline")
      .def(py::init<>())
      .def("pose", &evalio::Pipeline::pose)
      .def("map", &evalio::Pipeline::map)
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

  py::class_<KissICP, evalio::Pipeline>(m, "KissICP").def(py::init<>());
}
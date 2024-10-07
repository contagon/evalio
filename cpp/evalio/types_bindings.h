#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "evalio/types.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace evalio {

void makeTypes(py::module& m) {
  py::class_<evalio::Stamp>(m, "Stamp")
      .def(py::init<uint32_t, uint32_t>(), py::kw_only(), "sec"_a, "nsec"_a)
      .def_static("from_sec", &evalio::Stamp::from_sec)
      .def_static("from_nsec", &evalio::Stamp::from_nsec)
      .def("to_sec", &evalio::Stamp::to_sec)
      .def("to_nsec", &evalio::Stamp::to_nsec)
      .def_readonly("sec", &evalio::Stamp::sec)
      .def_readonly("nsec", &evalio::Stamp::nsec)
      .def(py::self < py::self)
      .def(py::self > py::self)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def("__repr__", &evalio::Stamp::toString);

  // Lidar
  py::class_<evalio::Point>(m, "Point")
      .def(py::init<double, double, double, double, uint32_t, uint32_t, uint8_t,
                    uint16_t>(),
           py::kw_only(), "x"_a = 0, "y"_a = 0, "z"_a = 0, "intensity"_a = 0,
           "t"_a = 0, "range"_a = 0, "row"_a = 0, "col"_a = 0)
      .def_readwrite("x", &evalio::Point::x)
      .def_readwrite("y", &evalio::Point::y)
      .def_readwrite("z", &evalio::Point::z)
      .def_readwrite("intensity", &evalio::Point::intensity)
      .def_readwrite("range", &evalio::Point::range)
      .def_readwrite("t", &evalio::Point::t)
      .def_readwrite("row", &evalio::Point::row)
      .def_readwrite("col", &evalio::Point::col)
      .def("__repr__", &evalio::Point::toString);

  py::class_<evalio::LidarMeasurement>(m, "LidarMeasurement")
      .def(py::init<evalio::Stamp, std::vector<evalio::Point>>(), "stamp"_a,
           "points"_a)
      .def_readonly("stamp", &evalio::LidarMeasurement::stamp)
      .def_readonly("points", &evalio::LidarMeasurement::points)
      .def("__repr__", &evalio::LidarMeasurement::toString);

  py::class_<evalio::LidarParams>(m, "LidarParams")
      .def(py::init<int, int, double, double>(), py::kw_only(), "num_rows"_a,
           "num_columns"_a, "min_range"_a, "max_range"_a)
      .def_readonly("num_rows", &evalio::LidarParams::num_rows)
      .def_readonly("num_columns", &evalio::LidarParams::num_columns)
      .def_readonly("min_range", &evalio::LidarParams::min_range)
      .def_readonly("max_range", &evalio::LidarParams::max_range)
      .def("__repr__", &evalio::LidarParams::toString);

  // Imu
  py::class_<evalio::ImuMeasurement>(m, "ImuMeasurement")
      .def(py::init<evalio::Stamp, Eigen::Vector3d, Eigen::Vector3d>(),
           "stamp"_a, "gyro"_a, "accel"_a)
      .def_readonly("stamp", &evalio::ImuMeasurement::stamp)
      .def_readonly("gyro", &evalio::ImuMeasurement::gyro)
      .def_readonly("accel", &evalio::ImuMeasurement::accel)
      .def("__repr__", &evalio::ImuMeasurement::toString);

  py::class_<evalio::ImuParams>(m, "ImuParams")
      .def(py::init<double, double, double, double, double, double,
                    Eigen::Vector3d>(),
           py::kw_only(), "gyro"_a = 1e-5, "accel"_a = 1e-5,
           "gyro_bias"_a = 1e-6, "accel_bias"_a = 1e-6, "bias_init"_a = 1e-7,
           "integration"_a = 1e-7, "gravity"_a = Eigen::Vector3d(0, 0, 9.81))
      .def_static("up", &evalio::ImuParams::up)
      .def_static("down", &evalio::ImuParams::down)
      .def_readwrite("gyro", &evalio::ImuParams::gyro)
      .def_readwrite("accel", &evalio::ImuParams::accel)
      .def_readwrite("gyro_bias", &evalio::ImuParams::gyro_bias)
      .def_readwrite("accel_bias", &evalio::ImuParams::accel_bias)
      .def_readwrite("bias_init", &evalio::ImuParams::bias_init)
      .def_readwrite("integration", &evalio::ImuParams::integration)
      .def_readwrite("gravity", &evalio::ImuParams::gravity)
      .def("__repr__", &evalio::ImuParams::toString);

  py::class_<evalio::SO3>(m, "SO3")
      .def(py::init<double, double, double, double>(), py::kw_only(), "qx"_a,
           "qy"_a, "qz"_a, "qw"_a)
      .def_readonly("qx", &evalio::SO3::qx)
      .def_readonly("qy", &evalio::SO3::qy)
      .def_readonly("qz", &evalio::SO3::qz)
      .def_readonly("qw", &evalio::SO3::qw)
      .def_static("identity", &evalio::SO3::identity)
      .def("inverse", &evalio::SO3::inverse)
      .def(py::self * py::self)
      .def("__repr__", &evalio::SO3::toString);

  py::class_<evalio::SE3>(m, "SE3")
      .def(py::init<evalio::SO3, Eigen::Vector3d>(), "rot"_a, "trans"_a)
      .def_static("identity", &evalio::SE3::identity)
      .def_readonly("rot", &evalio::SE3::rot)
      .def_readonly("trans", &evalio::SE3::trans)
      .def("inverse", &evalio::SE3::inverse)
      .def(py::self * py::self)
      .def("__repr__", &evalio::SE3::toString);
}

}  // namespace evalio
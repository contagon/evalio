#pragma once
#include <cstdint>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "evalio/types.h"

namespace nb = nanobind;
using namespace nb::literals;

namespace evalio {

// TODO: Check if copy/deepcopy works or not

inline void makeTypes(nb::module_ &m) {
  nb::class_<Duration>(m, "Duration")
      .def_static("from_sec", &Duration::from_sec)
      .def_static("from_nsec", &Duration::from_nsec)
      .def("to_sec", &Duration::to_sec)
      .def("to_nsec", &Duration::to_nsec)
      .def_ro("nsec", &Duration::nsec)
      .def(nb::self < nb::self)
      .def(nb::self > nb::self)
      .def(nb::self == nb::self)
      .def(nb::self != nb::self)
      .def(nb::self - nb::self)
      .def(nb::self + nb::self)
      .def("__repr__", &Duration::toString)
      .def("__copy__", [](const Duration &self) { return Duration(self); })
      .def(
          "__deepcopy__",
          [](const Duration &self, nb::dict) { return Duration(self); },
          "memo"_a)
      .def("__getstate__",
           [](const Duration &p) { return nb::make_tuple(p.nsec); })
      .def("__setstate__", [](Duration &p, std::tuple<int64_t> t) {
        new (&p) Duration{.nsec = std::get<0>(t)};
      });

  nb::class_<Stamp>(m, "Stamp")
      .def(nb::init<uint32_t, uint32_t>(), nb::kw_only(), "sec"_a, "nsec"_a)
      .def_static("from_sec", &Stamp::from_sec)
      .def_static("from_nsec", &Stamp::from_nsec)
      .def("to_sec", &Stamp::to_sec)
      .def("to_nsec", &Stamp::to_nsec)
      .def_ro("sec", &Stamp::sec)
      .def_ro("nsec", &Stamp::nsec)
      .def(nb::self < nb::self)
      .def(nb::self > nb::self)
      .def(nb::self == nb::self)
      .def(nb::self != nb::self)
      .def(nb::self - nb::self)
      .def(nb::self + Duration())
      .def(nb::self - Duration())
      .def("__repr__", &Stamp::toString)
      .def("__copy__", [](const Stamp &self) { return Stamp(self); })
      .def(
          "__deepcopy__",
          [](const Stamp &self, nb::dict) { return Stamp(self); }, "memo"_a)
      .def("__getstate__",
           [](const Stamp &p) { return nb::make_tuple(p.sec, p.nsec); })
      .def("__setstate__", [](Stamp &p, std::tuple<uint32_t, uint32_t> t) {
        new (&p) Stamp{.sec = std::get<0>(t), .nsec = std::get<1>(t)};
      });
  ;

  // Lidar
  nb::class_<Point>(m, "Point")
      .def(nb::init<double, double, double, double, Duration, uint32_t, uint8_t,
                    uint16_t>(),
           nb::kw_only(), "x"_a = 0, "y"_a = 0, "z"_a = 0, "intensity"_a = 0,
           "t"_a = Duration::from_sec(0.0), "range"_a = 0, "row"_a = 0,
           "col"_a = 0)
      .def_rw("x", &Point::x)
      .def_rw("y", &Point::y)
      .def_rw("z", &Point::z)
      .def_rw("intensity", &Point::intensity)
      .def_rw("range", &Point::range)
      .def_rw("t", &Point::t)
      .def_rw("row", &Point::row)
      .def_rw("col", &Point::col)
      .def(nb::self == nb::self)
      .def(nb::self != nb::self)
      .def("__repr__", &Point::toString)
      .def("__getstate__",
           [](const Point &p) {
             return std::make_tuple(p.x, p.y, p.z, p.intensity, p.t, p.range,
                                    p.row, p.col);
           })
      .def("__setstate__",
           [](Point &p, std::tuple<double, double, double, double, Duration,
                                   uint32_t, uint8_t, uint16_t>
                            t) {
             new (&p) Point{.x = std::get<0>(t),
                            .y = std::get<1>(t),
                            .z = std::get<2>(t),
                            .intensity = std::get<3>(t),
                            .t = std::get<4>(t),
                            .range = std::get<5>(t),
                            .row = std::get<6>(t),
                            .col = std::get<7>(t)};
           });

  nb::class_<LidarMeasurement>(m, "LidarMeasurement")
      .def(nb::init<Stamp>(), "stamp"_a)
      .def(nb::init<Stamp, std::vector<Point>>(), "stamp"_a, "points"_a)
      .def_rw("stamp", &LidarMeasurement::stamp)
      .def_rw("points", &LidarMeasurement::points)
      .def("to_vec_positions", &LidarMeasurement::to_vec_positions)
      .def("to_vec_stamps", &LidarMeasurement::to_vec_stamps)
      .def(nb::self == nb::self)
      .def(nb::self != nb::self)
      .def("__repr__", &LidarMeasurement::toString)
      .def("__getstate__",
           [](const LidarMeasurement &p) {
             return std::make_tuple(p.stamp, p.points);
           })
      .def("__setstate__",
           [](LidarMeasurement &p, std::tuple<Stamp, std::vector<Point>> t) {
             new (&p) LidarMeasurement(std::get<0>(t), std::get<1>(t));
           });

  nb::class_<LidarParams>(m, "LidarParams")
      .def(nb::init<int, int, double, double, double>(), nb::kw_only(),
           "num_rows"_a, "num_columns"_a, "min_range"_a, "max_range"_a,
           "rate"_a = 10.0)
      .def_ro("num_rows", &LidarParams::num_rows)
      .def_ro("num_columns", &LidarParams::num_columns)
      .def_ro("min_range", &LidarParams::min_range)
      .def_ro("max_range", &LidarParams::max_range)
      .def_ro("rate", &LidarParams::rate)
      .def("delta_time", &LidarParams::delta_time)
      .def("__repr__", &LidarParams::toString);

  // Imu
  nb::class_<ImuMeasurement>(m, "ImuMeasurement")
      .def(nb::init<Stamp, Eigen::Vector3d, Eigen::Vector3d>(), "stamp"_a,
           "gyro"_a, "accel"_a)
      .def_ro("stamp", &ImuMeasurement::stamp)
      .def_ro("gyro", &ImuMeasurement::gyro)
      .def_ro("accel", &ImuMeasurement::accel)
      .def(nb::self == nb::self)
      .def(nb::self != nb::self)
      .def("__repr__", &ImuMeasurement::toString)
      .def("__getstate__",
           [](const ImuMeasurement &p) {
             return std::make_tuple(p.stamp, p.gyro, p.accel);
           })
      .def("__setstate__",
           [](ImuMeasurement &p,
              std::tuple<Stamp, Eigen::Vector3d, Eigen::Vector3d> t) {
             new (&p) ImuMeasurement{.stamp = std::get<0>(t),
                                     .gyro = std::get<1>(t),
                                     .accel = std::get<2>(t)};
           });

  nb::class_<ImuParams>(m, "ImuParams")
      .def(nb::init<double, double, double, double, double, double,
                    Eigen::Vector3d>(),
           nb::kw_only(), "gyro"_a = 1e-5, "accel"_a = 1e-5,
           "gyro_bias"_a = 1e-6, "accel_bias"_a = 1e-6, "bias_init"_a = 1e-7,
           "integration"_a = 1e-7, "gravity"_a = Eigen::Vector3d(0, 0, 9.81))
      .def_static("up", &ImuParams::up)
      .def_static("down", &ImuParams::down)
      .def_rw("gyro", &ImuParams::gyro)
      .def_rw("accel", &ImuParams::accel)
      .def_rw("gyro_bias", &ImuParams::gyro_bias)
      .def_rw("accel_bias", &ImuParams::accel_bias)
      .def_rw("bias_init", &ImuParams::bias_init)
      .def_rw("integration", &ImuParams::integration)
      .def_rw("gravity", &ImuParams::gravity)
      .def("__repr__", &ImuParams::toString);

  nb::class_<SO3>(m, "SO3")
      .def(nb::init<double, double, double, double>(), nb::kw_only(), "qx"_a,
           "qy"_a, "qz"_a, "qw"_a)
      .def_ro("qx", &SO3::qx)
      .def_ro("qy", &SO3::qy)
      .def_ro("qz", &SO3::qz)
      .def_ro("qw", &SO3::qw)
      .def_static("identity", &SO3::identity)
      .def_static("fromMat", &SO3::fromMat)
      .def_static("exp", &SO3::exp)
      .def("inverse", &SO3::inverse)
      .def("log", &SO3::log)
      .def("toMat", &SO3::toMat)
      .def("rotate", &SO3::rotate)
      .def(nb::self * nb::self)
      .def("__repr__", &SO3::toString)
      .def("__copy__", [](const SO3 &self) { return SO3(self); })
      .def(
          "__deepcopy__", [](const SO3 &self, nb::dict) { return SO3(self); },
          "memo"_a);

  nb::class_<SE3>(m, "SE3")
      .def(nb::init<SO3, Eigen::Vector3d>(), "rot"_a, "trans"_a)
      .def_static("identity", &SE3::identity)
      .def_static("fromMat", &SE3::fromMat)
      .def_ro("rot", &SE3::rot)
      .def_ro("trans", &SE3::trans)
      .def("toMat", &SE3::toMat)
      .def("inverse", &SE3::inverse)
      .def(nb::self * nb::self)
      .def("__repr__", &SE3::toString)
      .def("__copy__", [](const SE3 &self) { return SE3(self); })
      .def(
          "__deepcopy__", [](const SE3 &self, nb::dict) { return SE3(self); },
          "memo"_a)
      .def("__getstate__",
           [](const SE3 &p) {
             return nb::make_tuple(p.rot.qx, p.rot.qy, p.rot.qz, p.rot.qw,
                                   p.trans[0], p.trans[1], p.trans[2]);
           })
      .def("__setstate__",
           [](SE3 &p,
              std::tuple<double, double, double, double, double, double, double>
                  t) {
             new (&p) SE3(SO3{.qx = std::get<0>(t),
                              .qy = std::get<1>(t),
                              .qz = std::get<2>(t),
                              .qw = std::get<3>(t)},
                          Eigen::Vector3d{std::get<4>(t), std::get<5>(t),
                                          std::get<6>(t)});
           });
}

} // namespace evalio
#pragma once
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <cstdint>

#include "evalio/types.h"

namespace nb = nanobind;
using namespace nb::literals;

namespace evalio {

// TODO: Check if copy/deepcopy works or not

inline void makeTypes(nb::module_& m) {
  nb::class_<Duration>(m, "Duration")
    .def_static(
      "from_sec",
      &Duration::from_sec,
      "sec"_a,
      "Create a Duration from seconds"
    )
    .def_static(
      "from_nsec",
      &Duration::from_nsec,
      "nsec"_a,
      "Create a Duration from nanoseconds"
    )
    .def("to_sec", &Duration::to_sec, "Convert to seconds")
    .def("to_nsec", &Duration::to_nsec, "Convert to nanoseconds")
    .def_ro("nsec", &Duration::nsec, "Underlying nanoseconds representation")
    .def(nb::self < nb::self, "Compare two Durations")
    .def(nb::self > nb::self, "Compare two Durations")
    .def(
      nb::self == nb::self,
      "Check for equality",
      nb::sig("def __eq__(self, arg: object, /) -> bool")
    )
    .def(
      nb::self != nb::self,
      "Check for inequality",
      nb::sig("def __ne__(self, arg: object, /) -> bool")
    )
    .def(nb::self - nb::self, "Compute the difference between two Durations")
    .def(nb::self + nb::self, "Add two Durations")
    .def("__repr__", &Duration::toString)
    .def("__copy__", [](const Duration& self) { return Duration(self); })
    .def(
      "__deepcopy__",
      [](const Duration& self, nb::typed<nb::dict, nb::any, nb::any>) {
        return Duration(self);
      },
      "memo"_a
    )
    .def(
      "__getstate__",
      [](const Duration& p) { return std::make_tuple(p.nsec); }
    )
    .def(
      "__setstate__",
      [](Duration& p, std::tuple<int64_t> t) {
        new (&p) Duration {.nsec = std::get<0>(t)};
      }
    )
    .doc() =
    "Duration class for representing a positive or negative delta time, uses "
    "int64 as the underlying data storage for nanoseconds.";

  nb::class_<Stamp>(m, "Stamp")
    .def(
      nb::init<uint32_t, uint32_t>(),
      nb::kw_only(),
      "sec"_a,
      "nsec"_a,
      "Create a Stamp from seconds and nanoseconds"
    )
    .def_static(
      "from_sec",
      &Stamp::from_sec,
      "sec"_a,
      "Create a Stamp from seconds"
    )
    .def_static(
      "from_nsec",
      &Stamp::from_nsec,
      "nsec"_a,
      "Create a Stamp from nanoseconds"
    )
    .def("to_sec", &Stamp::to_sec, "Convert to seconds")
    .def("to_nsec", &Stamp::to_nsec, "Convert to nanoseconds")
    .def_ro("sec", &Stamp::sec, "Underlying seconds storage")
    .def_ro("nsec", &Stamp::nsec, "Underlying nanoseconds storage")
    .def(nb::self < nb::self, "Compare two Stamps to see which happened first")
    .def(nb::self > nb::self, "Compare two Stamps to see which happened first")
    .def(
      nb::self == nb::self,
      "Check for equality",
      nb::sig("def __eq__(self, arg: object, /) -> bool")
    )
    .def(
      nb::self != nb::self,
      "Check for inequality",
      nb::sig("def __ne__(self, arg: object, /) -> bool")
    )
    .def(
      nb::self - nb::self,
      "Compute the difference between two Stamps, returning a duration"
    )
    .def(nb::self + Duration(), "Add a Duration to a Stamp")
    .def(nb::self - Duration(), "Subtract a Duration from a Stamp")
    .def("__repr__", &Stamp::toString)
    .def("__copy__", [](const Stamp& self) { return Stamp(self); })
    .def(
      "__deepcopy__",
      [](const Stamp& self, nb::typed<nb::dict, nb::any, nb::any>) {
        return Stamp(self);
      },
      "memo"_a
    )
    .def(
      "__getstate__",
      [](const Stamp& p) { return std::make_tuple(p.sec, p.nsec); }
    )
    .def(
      "__setstate__",
      [](Stamp& p, std::tuple<uint32_t, uint32_t> t) {
        new (&p) Stamp {.sec = std::get<0>(t), .nsec = std::get<1>(t)};
      }
    )
    .doc() =
    "Stamp class for representing an absolute point in time, uses uint32 as "
    "the underlying data storage for seconds and nanoseconds.";
  ;

  // Lidar
  nb::class_<Point>(m, "Point")
    .def(
      nb::init<
        double,
        double,
        double,
        double,
        Duration,
        uint32_t,
        uint8_t,
        uint16_t>(),
      nb::kw_only(),
      "x"_a = 0,
      "y"_a = 0,
      "z"_a = 0,
      "intensity"_a = 0,
      "t"_a = Duration::from_sec(0.0),
      "range"_a = 0,
      "row"_a = 0,
      "col"_a = 0,
      "Create a Point from x, y, z, intensity, t, range, row, col"
    )
    .def_rw("x", &Point::x, "X coordinate")
    .def_rw("y", &Point::y, "Y coordinate")
    .def_rw("z", &Point::z, "Z coordinate")
    .def_rw("intensity", &Point::intensity, "Intensity value as a float.")
    .def_rw("range", &Point::range, "Range value as a uint32.")
    .def_rw(
      "t",
      &Point::t,
      "Timestamp of the point as a Duration. In evalio, this is always "
      "relative to the point cloud stamp, which occurs at the start of "
      "the scan."
    )
    .def_rw(
      "row",
      &Point::row,
      "Row index of the point in the point cloud. Also known as the "
      "scanline index."
    )
    .def_rw("col", &Point::col, "Column index of the point in the point cloud.")
    .def(
      nb::self == nb::self,
      "Check for equality",
      nb::sig("def __eq__(self, arg: object, /) -> bool")
    )
    .def(
      nb::self != nb::self,
      "Check for inequality",
      nb::sig("def __ne__(self, arg: object, /) -> bool")
    )
    .def("__repr__", &Point::toString)
    .def(
      "__getstate__",
      [](const Point& p) {
        return std::make_tuple(
          p.x,
          p.y,
          p.z,
          p.intensity,
          p.t,
          p.range,
          p.row,
          p.col
        );
      }
    )
    .def(
      "__setstate__",
      [](
        Point& p,
        std::tuple<
          double,
          double,
          double,
          double,
          Duration,
          uint32_t,
          uint8_t,
          uint16_t> t
      ) {
        new (&p) Point {
          .x = std::get<0>(t),
          .y = std::get<1>(t),
          .z = std::get<2>(t),
          .intensity = std::get<3>(t),
          .t = std::get<4>(t),
          .range = std::get<5>(t),
          .row = std::get<6>(t),
          .col = std::get<7>(t)
        };
      }
    )
    .doc() =
    "Point is a general point structure in evalio, with common "
    "point cloud attributes included.";

  nb::class_<LidarMeasurement>(m, "LidarMeasurement")
    .def(nb::init<Stamp>(), "stamp"_a)
    .def(nb::init<Stamp, std::vector<Point>>(), "stamp"_a, "points"_a)
    .def_rw(
      "stamp",
      &LidarMeasurement::stamp,
      "Timestamp of the point cloud, always at the start of the scan."
    )
    .def_rw(
      "points",
      &LidarMeasurement::points,
      "List of points in the "
      "point cloud. Note, this is always in row major format."
    )
    .def(
      "to_vec_positions",
      &LidarMeasurement::to_vec_positions,
      "Convert the point cloud to a (n,3) numpy array."
    )
    .def(
      "to_vec_stamps",
      &LidarMeasurement::to_vec_stamps,
      "Convert the point stamps to a list of durations."
    )
    .def(
      nb::self == nb::self,
      "Check for equality",
      nb::sig("def __eq__(self, arg: object, /) -> bool")
    )
    .def(
      nb::self != nb::self,
      "Check for inequality",
      nb::sig("def __ne__(self, arg: object, /) -> bool")
    )
    .def("__repr__", &LidarMeasurement::toString)
    .def(
      "__getstate__",
      [](const LidarMeasurement& p) {
        return std::make_tuple(p.stamp, p.points);
      }
    )
    .def(
      "__setstate__",
      [](LidarMeasurement& p, std::tuple<Stamp, std::vector<Point>> t) {
        new (&p) LidarMeasurement(std::get<0>(t), std::get<1>(t));
      }
    )
    .doc() =
    "LidarMeasurement is a structure for storing a point cloud "
    "measurement, with a timestamp and a vector of points. Note, "
    "the stamp always represents the _start_ of the scan. "
    "Additionally, the points are always in row major format.";

  nb::class_<LidarParams>(m, "LidarParams")
    .def(
      nb::init<int, int, double, double, double, std::string, std::string>(),
      nb::kw_only(),
      "num_rows"_a,
      "num_columns"_a,
      "min_range"_a,
      "max_range"_a,
      "rate"_a = 10.0,
      "brand"_a = "-",
      "model"_a = "-"
    )
    .def_ro(
      "num_rows",
      &LidarParams::num_rows,
      "Number of rows in the point cloud, also known as the scanlines."
    )
    .def_ro(
      "num_columns",
      &LidarParams::num_columns,
      "Number of columns in the point cloud, also known as the number "
      "of points per scanline."
    )
    .def_ro(
      "min_range",
      &LidarParams::min_range,
      "Minimum range of the lidar sensor, in meters."
    )
    .def_ro(
      "max_range",
      &LidarParams::max_range,
      "Maximum range of the lidar sensor, in meters."
    )
    .def_ro("rate", &LidarParams::rate, "Rate of the lidar sensor, in Hz.")
    .def_ro("brand", &LidarParams::brand, "Brand of the lidar sensor.")
    .def_ro("model", &LidarParams::model, "Model of the lidar sensor.")
    .def(
      "delta_time",
      &LidarParams::delta_time,
      "Get the time between two consecutive scans as a Duration. Inverse "
      "of the rate."
    )
    .def("__repr__", &LidarParams::toString)
    .doc() =
    "LidarParams is a structure for storing the parameters of a "
    "lidar sensor.";

  // Imu
  nb::class_<ImuMeasurement>(m, "ImuMeasurement")
    .def(
      nb::init<Stamp, Eigen::Vector3d, Eigen::Vector3d>(),
      "stamp"_a,
      "gyro"_a,
      "accel"_a
    )
    .def_ro(
      "stamp",
      &ImuMeasurement::stamp,
      "Timestamp of the IMU measurement."
    )
    .def_ro(
      "gyro",
      &ImuMeasurement::gyro,
      "Gyroscope measurement as a 3D vector."
    )
    .def_ro(
      "accel",
      &ImuMeasurement::accel,
      "Accelerometer measurement as a 3D vector."
    )
    .def(
      nb::self == nb::self,
      "Check for equality",
      nb::sig("def __eq__(self, arg: object, /) -> bool")
    )
    .def(
      nb::self != nb::self,
      "Check for inequality",
      nb::sig("def __ne__(self, arg: object, /) -> bool")
    )
    .def("__repr__", &ImuMeasurement::toString)
    .def(
      "__getstate__",
      [](const ImuMeasurement& p) {
        return std::make_tuple(p.stamp, p.gyro, p.accel);
      }
    )
    .def(
      "__setstate__",
      [](
        ImuMeasurement& p,
        std::tuple<Stamp, Eigen::Vector3d, Eigen::Vector3d> t
      ) {
        new (&p) ImuMeasurement {
          .stamp = std::get<0>(t),
          .gyro = std::get<1>(t),
          .accel = std::get<2>(t)
        };
      }
    )

    .doc() =
    "ImuMeasurement is a simple structure for storing an IMU measurement.";

  nb::class_<ImuParams>(m, "ImuParams")
    .def(
      nb::init<
        double,
        double,
        double,
        double,
        double,
        double,
        Eigen::Vector3d,
        std::string,
        std::string>(),
      nb::kw_only(),
      "gyro"_a = 1e-5,
      "accel"_a = 1e-5,
      "gyro_bias"_a = 1e-6,
      "accel_bias"_a = 1e-6,
      "bias_init"_a = 1e-7,
      "integration"_a = 1e-7,
      "gravity"_a = Eigen::Vector3d(0, 0, 9.81),
      "brand"_a = "-",
      "model"_a = "-"
    )
    .def_static(
      "up",
      &ImuParams::up,
      "Simple helper for initializing with an `up` gravity vector."
    )
    .def_static(
      "down",
      &ImuParams::down,
      "Simple helper for initializing with a `down` gravity vector."
    )
    .def_ro(
      "gyro",
      &ImuParams::gyro,
      "Gyroscope standard deviation, in rad/s/sqrt(Hz)."
    )
    .def_ro(
      "accel",
      &ImuParams::accel,
      "Accelerometer standard deviation, in m/s^2/sqrt(Hz)."
    )
    .def_ro(
      "gyro_bias",
      &ImuParams::gyro_bias,
      "Gyroscope bias standard deviation, in rad/s^2/sqrt(Hz)."
    )
    .def_ro(
      "accel_bias",
      &ImuParams::accel_bias,
      "Accelerometer bias standard deviation, in m/s^3/sqrt(Hz)."
    )
    .def_ro(
      "bias_init",
      &ImuParams::bias_init,
      "Initial bias standard deviation."
    )
    .def_ro(
      "integration",
      &ImuParams::integration,
      "Integration standard deviation."
    )
    .def_ro("gravity", &ImuParams::gravity, "Gravity vector as a 3D vector.")
    .def_ro("brand", &ImuParams::brand, "Brand of the IMU sensor.")
    .def_ro("model", &ImuParams::model, "Model of the IMU sensor.")
    .def("__repr__", &ImuParams::toString)
    .doc() = "ImuParams is a structure for storing the parameters of an IMU";

  nb::class_<SO3>(m, "SO3")
    .def(
      nb::init<double, double, double, double>(),
      nb::kw_only(),
      "qx"_a,
      "qy"_a,
      "qz"_a,
      "qw"_a
    )
    .def_ro("qx", &SO3::qx, "X component of the quaternion.")
    .def_ro("qy", &SO3::qy, "Y component of the quaternion.")
    .def_ro("qz", &SO3::qz, "Z component of the quaternion.")
    .def_ro("qw", &SO3::qw, "Scalar component of the quaternion.")
    .def_static("identity", &SO3::identity, "Create an identity rotation.")
    .def_static(
      "fromMat",
      &SO3::fromMat,
      "mat"_a,
      "Create a rotation from a 3x3 rotation matrix."
    )
    .def_static("exp", &SO3::exp, "v"_a, "Create a rotation from a 3D vector.")
    .def("inverse", &SO3::inverse, "Compute the inverse of the rotation.")
    .def("log", &SO3::log, "Compute the logarithm of the rotation.")
    .def("toMat", &SO3::toMat, "Convert the rotation to a 3x3 matrix.")
    .def("rotate", &SO3::rotate, "v"_a, "Rotate a 3D vector by the rotation.")
    .def(nb::self * nb::self, "Compose two rotations.")
    .def(
      nb::self == nb::self,
      "Check for equality",
      nb::sig("def __eq__(self, arg: object, /) -> bool")
    )
    .def(
      nb::self != nb::self,
      "Check for inequality",
      nb::sig("def __ne__(self, arg: object, /) -> bool")
    )
    .def("__repr__", &SO3::toString)
    .def("__copy__", [](const SO3& self) { return SO3(self); })
    .def(
      "__deepcopy__",
      [](const SO3& self, nb::typed<nb::dict, nb::any, nb::any>) {
        return SO3(self);
      },
      "memo"_a
    )
    .def(
      "__getstate__",
      [](const SO3& p) { return std::make_tuple(p.qx, p.qy, p.qz, p.qw); }
    )
    .def(
      "__setstate__",
      [](SO3& p, std::tuple<double, double, double, double> t) {
        new (&p) SO3 {
          .qx = std::get<0>(t),
          .qy = std::get<1>(t),
          .qz = std::get<2>(t),
          .qw = std::get<3>(t)
        };
      }
    )
    .doc() =
    "SO3 class for representing a 3D rotation using a quaternion. "
    "This is outfitted with some basic functionality, but mostly "
    "intended for storage and converting between types.";

  nb::class_<SE3>(m, "SE3")
    .def(
      nb::init<SO3, Eigen::Vector3d>(),
      "rot"_a,
      "trans"_a,
      "Create a SE3 from a rotation and translation."
    )
    .def_static("identity", &SE3::identity, "Create an identity SE3.")
    .def_static(
      "fromMat",
      &SE3::fromMat,
      "mat"_a,
      "Create a SE3 from a 4x4 transformation matrix."
    )
    .def_ro("rot", &SE3::rot, "Rotation as a SO3 object.")
    .def_ro("trans", &SE3::trans, "Translation as a 3D vector.")
    .def("toMat", &SE3::toMat, "Convert to a 4x4 matrix.")
    .def("inverse", &SE3::inverse, "Compute the inverse.")
    .def_static("exp", &SE3::exp, "xi"_a, "Create a SE3 from a 3D vector.")
    .def("log", &SE3::log, "Compute the logarithm of the transformation.")
    .def(nb::self * nb::self, "Compose two rigid body transformations.")
    .def(
      nb::self == nb::self,
      "Check for equality",
      nb::sig("def __eq__(self, arg: object, /) -> bool")
    )
    .def(
      nb::self != nb::self,
      "Check for inequality",
      nb::sig("def __ne__(self, arg: object, /) -> bool")
    )
    .def("__repr__", &SE3::toString)
    .def("__copy__", [](const SE3& self) { return SE3(self); })
    .def(
      "__deepcopy__",
      [](const SE3& self, nb::typed<nb::dict, nb::any, nb::any>) {
        return SE3(self);
      },
      "memo"_a
    )
    .def(
      "__getstate__",
      [](const SE3& p) {
        return std::make_tuple(
          p.rot.qx,
          p.rot.qy,
          p.rot.qz,
          p.rot.qw,
          p.trans[0],
          p.trans[1],
          p.trans[2]
        );
      }
    )
    .def(
      "__setstate__",
      [](
        SE3& p,
        std::tuple<double, double, double, double, double, double, double> t
      ) {
        new (&p) SE3(
          SO3 {
            .qx = std::get<0>(t),
            .qy = std::get<1>(t),
            .qz = std::get<2>(t),
            .qw = std::get<3>(t)
          },
          Eigen::Vector3d {std::get<4>(t), std::get<5>(t), std::get<6>(t)}
        );
      }
    )
    .doc() =
    "SE3 class for representing a 3D rigid body transformation "
    "using a quaternion and a translation vector. This is outfitted "
    "with some basic functionality, but mostly intended for storage "
    "and converting between types.";
}

} // namespace evalio

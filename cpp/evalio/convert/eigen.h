// Handle eigen conversions here.
#pragma once

#include <Eigen/Dense>

#include "evalio/convert/base.h"
#include "evalio/types.h"

namespace evalio {
// ------------------------- Points ------------------------- //
template<>
inline Point convert(const Eigen::Vector3d& in) {
  return {
    .x = in[0],
    .y = in[1],
    .z = in[2],
    .intensity = 0.0,
    .t = Duration::from_sec(0),
    .row = 0,
    .col = 0
  };
}

template<>
inline Eigen::Vector3d convert(const Point& in) {
  return {in.x, in.y, in.z};
}

inline Eigen::MatrixX3d convert(const std::vector<Point>& in) {
  Eigen::MatrixX3d out(in.size(), 3);
  for (size_t i = 0; i < in.size(); ++i) {
    out.row(i) = convert<Eigen::Vector3d>(in[i]);
  }
  return out;
}

// ------------------------- Poses ------------------------- //
template<>
inline SE3 convert(const Eigen::Isometry3d& in) {
  const auto t = in.translation();
  const auto q = Eigen::Quaterniond(in.linear());
  const auto rot = SO3 {.qx = q.x(), .qy = q.y(), .qz = q.z(), .qw = q.w()};
  return SE3(rot, t);
}

template<>
inline SE3 convert(const Eigen::Matrix4d& in) {
  const auto iso = Eigen::Isometry3d(in);
  return convert<SE3>(iso);
}
} // namespace evalio

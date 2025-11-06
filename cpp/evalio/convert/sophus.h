// Handle sophus conversions
#pragma once

#include <sophus/se3.hpp>

#include "evalio/convert/base.h"
#include "evalio/types.h"

namespace evalio {
template<>
inline SE3 convert(const Sophus::SE3d& in) {
  const auto t = in.translation();
  const auto q = in.unit_quaternion();
  const auto rot =
    evalio::SO3 {.qx = q.x(), .qy = q.y(), .qz = q.z(), .qw = q.w()};
  return evalio::SE3(rot, t);
}

template<>
inline Sophus::SE3d convert(const SE3& in) {
  return Sophus::SE3d(Sophus::SO3d(in.rot.to_eigen()), in.trans);
}

} // namespace evalio

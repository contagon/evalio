// Handle eigen conversions here.
#pragma once

#include <Eigen/Dense>

#include "evalio/convert/base.h"
#include "evalio/types.h"

namespace evalio {
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

} // namespace evalio

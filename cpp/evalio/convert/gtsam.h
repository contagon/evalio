// Handle GTSAM conversions.
#pragma once

#include <gtsam/geometry/Pose3.h>

#include "evalio/convert/base.h"
#include "evalio/types.h"

namespace evalio {

template<>
inline SE3 convert(const gtsam::Pose3& in) {
  return SE3(
    SO3::from_mat(in.rotation().matrix()),
    in.translation()
  );
}

template<>
inline gtsam::Pose3 convert(const SE3& in) {
  return gtsam::Pose3(
    gtsam::Rot3(in.rot.to_mat()),
    gtsam::Point3(in.trans)
  );
}

} // namespace evalio

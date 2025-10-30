// Helper module for converting between the various library types.
#pragma once

#include <vector>

#include "evalio/types.h"

namespace evalio {

/// @brief Generic conversion function between types.
///
/// Specializations may be imported from other files or defined by the user.
template<typename Out, typename In>
inline Out convert(const In& in) = delete;

/// @brief Convert a vector of In to a vector of Out using convert<Out>(In).
template<typename Out, typename In>
inline std::vector<Out> convert(const std::vector<In>& in) {
  std::vector<Out> out;
  out.reserve(in.size());
  for (const auto& item : in) {
    out.push_back(convert<Out>(item));
  }
  return out;
}

/// @brief Convert evalio::Point to timestamps as doubles.
template<>
inline double convert(const evalio::Point& in) {
  return in.t.to_sec();
}

} // namespace evalio

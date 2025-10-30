// Helper module for converting between the various library types.
#pragma once

#include "evalio/types.h"

namespace evalio {

/// @brief Generic conversion function between types.
///
/// Overloads and specializations may be imported from other files or defined by the user.
template<typename Out, typename In>
inline Out convert(const In& in) = delete;

/// @brief Convert a container of In to a container of Out using convert<Out>(in).
template<
  template<class...> class OutCont,
  class Out,
  template<class...> class InCont,
  class In>
inline OutCont<Out> convert(const InCont<In>& in) {
  OutCont<Out> out;
  out.reserve(in.size());
  for (const auto& item : in) {
    out.push_back(convert<Out, In>(item));
  }
  return out;
}

/// @brief Convert evalio::Point to timestamps as doubles.
template<>
inline double convert(const evalio::Point& in) {
  return in.t.to_sec();
}

} // namespace evalio

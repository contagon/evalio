// Helper module for converting between the various library types.
#pragma once

#include <map>

#include "evalio/types.h"

namespace evalio {

/// @brief Generic conversion function between types.
///
/// Overloads and specializations may be imported from other files or defined by the user.
template<typename Out, typename In>
inline Out convert(const In& in) {
  // Allow identity conversions by default
  if constexpr (std::is_same_v<Out, In>) {
    return in;
  } else {
    static_assert(
      sizeof(Out) == 0,
      "No conversion defined between the specified types."
    );
  }
}

/// @brief Convert evalio::Point to timestamps as doubles.
template<>
inline double convert(const evalio::Point& in) {
  return in.t.to_sec();
}

/// @brief Convert a container of In to a container of Out using convert<Out>(in).
///
/// Assumes the following are defined:
/// - In::size().
/// - In is iterable with range-based for loops.
/// - Out is default-constructible.
/// - Out::value_type.
/// - Out::reserve(size_t).
/// - Out::push_back(Out::value_type).
/// - convert<Out::value_type, In::value_type>(In::value_type).
template<class Out, class In>
inline Out convert_iter(const In& in) {
  Out out;
  out.reserve(in.size());
  for (const auto& item : in) {
    out.push_back(
      convert<typename Out::value_type, typename In::value_type>(item)
    );
  }
  return out;
}

/// @brief Convert a map of arbitrary containers to a map of std::vector<evalio::Point>.
///
/// Useful for converting to final evalio map types. Unlike other convert functions,
/// this one only requires the input type to be specified.
/// Assumes the following are defined:
/// - In::size().
/// - In is iterable with range-based for loops.
/// - convert<evalio::Point, In::value_type>(In::value_type).
template<class In>
inline std::map<std::string, std::vector<evalio::Point>>
convert_map(const std::map<std::string, In>& in) {
  std::map<std::string, std::vector<evalio::Point>> out;
  for (const auto& [key, val] : in) {
    out[key] = convert_iter<std::vector<evalio::Point>>(val);
  }
  return out;
}

} // namespace evalio

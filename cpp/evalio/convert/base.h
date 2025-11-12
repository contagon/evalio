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

/// @brief Convert a container of In to a container of Out using convert<Out::value_type>(in).
///
/// Assumes the following are defined:
/// - In::value_type.
/// - In::size().
/// - In is iterable with range-based for loops.
/// - Out is default-constructible.
/// - Out::value_type.
/// - Out::reserve(size_t).
/// - Out::push_back(Out::value_type).
/// - convert<Out::value_type, In::value_type>(in_value).
/// These should all generally be true to std containers.
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

// recursive case
template<typename T, typename... Args>
inline void _make_map_impl(
  std::map<std::string, std::vector<evalio::Point>>& out,
  std::string&& k,
  T&& v,
  Args&&... args
) {
  auto converted = convert_iter<std::vector<evalio::Point>>(std::forward<T>(v));
  out.insert({std::forward<std::string>(k), converted});
  if constexpr (sizeof...(args) > 0) {
    _make_map_impl(out, std::forward<Args>(args)...);
  }
}

/// @brief Create a map from variadic key-value pairs.
///
/// Allows a pretty usage like:
/// ```cpp
/// auto my_map = evalio::make_map("planar", planar_data);
/// ```
/// This allows easy construction of maps without having to specify the full type
/// or performing conversions by hand. Requires convert_iter<std::vector<Point>, T>.
/// Not 100% a conversion, but felt appropriate to include here.
template<typename T, typename... Args>
inline std::map<std::string, std::vector<evalio::Point>>
make_map(std::string&& k, T&& v, Args&&... args) {
  std::map<std::string, std::vector<evalio::Point>> out;
  _make_map_impl<T>(
    out,
    std::forward<std::string>(k),
    std::forward<T>(v),
    std::forward<Args>(args)...
  );
  return out;
}

} // namespace evalio

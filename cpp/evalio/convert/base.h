// Helper module for converting between the various library types.
#pragma once

#include <map>

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

/// @brief Convert a map of arbitrary containers to a map of std::vector<evalio::Point>.
///
/// Useful for converting to final evalio map types.
/// NOTE: Dict<In = std::vector<Point>> = std::map<std::string, In>
// TODO: This is a bit confusing... for other versions of convert, we have to specify output when calling, for this one we specify the input type...
template<template<class...> class InCont, class In>
inline std::map<std::string, std::vector<evalio::Point>>
convert(const std::map<std::string, InCont<In>>& in) {
  std::map<std::string, std::vector<evalio::Point>> out;
  for (const auto& [key, val] : in) {
    out[key] = convert<std::vector, evalio::Point>(val);
  }
  return out;
}

/// @brief Convert evalio::Point to timestamps as doubles.
template<>
inline double convert(const evalio::Point& in) {
  return in.t.to_sec();
}

} // namespace evalio

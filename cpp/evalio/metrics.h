#pragma once
#include <cmath>
#include <functional>
#include <string>

namespace evalio {

inline double norm2(double p, double i) {
  if (i < 1e-3) {
    i = 1e-3;
  }
  return i;
}

inline double norm1(double p, double i) {
  i = std::sqrt(i);
  if (i < 1e-3) {
    i = 1e-3;
  }
  return i;
}

inline double norm0(double p, double i) { return 1.0; }

inline double step(double p, double i) { return i < 10 ? 1.0 : 10.0; }

inline double add_norm12(double p, double i) { return (1.0 + std::sqrt(i)); }

inline double add_norm1(double p, double i) { return (1.0 + i); }

inline double add_norm2(double p, double i) { return (1.0 + i * i); }

inline std::function<double(double, double)> lookup(std::string name) {
  // clang-format off
  if (name == "norm0") return norm0;
  if (name == "norm1") return norm1;
  if (name == "norm2") return norm2;
  if (name == "step") return step;
  if (name == "add_norm12") return add_norm12;
  if (name == "add_norm1") return add_norm1;
  if (name == "add_norm2") return add_norm2;
  // clang-format on
  throw "Couldn't find metric";
}

} // namespace evalio
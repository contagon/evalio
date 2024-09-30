#include "evalio/pipelines/bindings.h"

#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "evalio/conversions.h"
#include "evalio/types_bindings.h"

PYBIND11_MODULE(_cpp, m) {
  auto m_types = m.def_submodule(
      "types",
      "Common types used for conversion between datasets and pipelines.");
  evalio::makeTypes(m_types);

  auto m_helpers = m.def_submodule(
      "_helpers", "Helper functions for internal evalio usage.");
  evalio::makeConversions(m_helpers);

  auto m_pipelines = m.def_submodule("pipelines", "Pipelines used in evalio.");
  evalio::makePipelines(m_pipelines);
}
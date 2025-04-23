#include <nanobind/nanobind.h>

#include "bindings/pipelines/bindings.h"
#include "bindings/ros_pc2.h"
#include "bindings/types.h"
#include "evalio/bindings.h"

namespace nb = nanobind;

NB_MODULE(_cpp, m) {
  m.def("abi_tag", []() { return nb::detail::abi_tag(); });

  auto m_types = m.def_submodule(
      "types",
      "Common types used for conversion between datasets and pipelines.");
  evalio::makeTypes(m_types);

  auto m_helpers = m.def_submodule(
      "_helpers", "Helper functions for internal evalio usage.");
  evalio::makeConversions(m_helpers);

  auto m_pipelines = m.def_submodule("pipelines", "Pipelines used in evalio.");
  evalio::makeBasePipeline(m_pipelines);
  evalio::makePipelines(m_pipelines);
}
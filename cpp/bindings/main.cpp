#include <nanobind/nanobind.h>

#include "bindings/pipeline.h"
#include "bindings/pipelines/bindings.h"
#include "bindings/ros_pc2.h"
#include "bindings/types.h"

namespace nb = nanobind;

NB_MODULE(_cpp, m) {
  nb::set_leak_warnings(false);

  m.def(
    "abi_tag",
    []() { return nb::detail::abi_tag(); },
    "Get the ABI tag of the current module. Useful for debugging when adding "
    "external pipelines."
  );

  auto m_types = m.def_submodule(
    "types",
    "Common types used for conversion between datasets and pipelines."
  );
  evalio::make_types(m_types);

  auto m_helpers =
    m.def_submodule("helpers", "Helper functions for internal evalio usage.");
  evalio::make_conversions(m_helpers);

  auto m_pipelines = m.def_submodule("pipelines", "Pipelines used in evalio.");
  evalio::make_base_pipeline(m_pipelines);
  evalio::make_pipelines(m_pipelines);
}

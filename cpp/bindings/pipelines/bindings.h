#pragma once

#include <nanobind/nanobind.h>

#include "evalio/pipeline.h"

namespace nb = nanobind;
using namespace nb::literals;

#ifdef EVALIO_KISS_ICP
#include "bindings/pipelines/kiss_icp.h"
#endif

#ifdef EVALIO_LIO_SAM
#include "bindings/pipelines/lio_sam.h"
#endif

namespace evalio {
inline void makePipelines(nb::module_ &m) {
  // List all the pipelines here
#ifdef EVALIO_KISS_ICP
  nb::class_<KissICP, evalio::Pipeline>(m, "KissICP")
      .def(nb::init<>())
      .def_static("name", &KissICP::name)
      .def_static("url", &KissICP::url)
      .def_static("default_params", &KissICP::default_params);

#endif

#ifdef EVALIO_LIO_SAM
  nb::class_<LioSam, evalio::Pipeline>(m, "LioSAM")
      .def(nb::init<>())
      .def_static("name", &LioSam::name)
      .def_static("url", &LioSam::url)
      .def_static("default_params", &LioSam::default_params);
#endif
}
} // namespace evalio
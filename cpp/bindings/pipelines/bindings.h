#pragma once

#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "evalio/pipeline.h"

namespace py = pybind11;
using namespace pybind11::literals;

#ifdef EVALIO_KISS_ICP
#include "bindings/pipelines/kiss_icp.h"
#endif

#ifdef EVALIO_LIO_SAM
#include "bindings/pipelines/lio_sam.h"
#endif

namespace evalio {
inline void makePipelines(py::module &m) {
  // List all the pipelines here
#ifdef EVALIO_KISS_ICP
  py::class_<KissICP, evalio::Pipeline>(m, "KissICP")
      .def(py::init<>())
      .def_static("name", &KissICP::name)
      .def_static("url", &KissICP::url)
      .def_static("default_params", &KissICP::default_params);

#endif

#ifdef EVALIO_LIO_SAM
  py::class_<LioSam, evalio::Pipeline>(m, "LioSAM")
      .def(py::init<>())
      .def_static("name", &LioSam::name)
      .def_static("url", &LioSam::url)
      .def_static("default_params", &LioSam::default_params);
#endif
}
} // namespace evalio
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

#ifdef EVALIO_LOAM
#include "bindings/pipelines/loam.h"
#endif

namespace evalio {
inline void makePipelines(nb::module_ &m) {
  // List all the pipelines here
#ifdef EVALIO_KISS_ICP
  nb::class_<KissICP, evalio::Pipeline>(m, "KissICP")
      .def(nb::init<>())
      .def_static("name", &KissICP::name)
      .def_static("default_params", &KissICP::default_params)
      .def_static("url", &KissICP::url)
      .def_static("version", &KissICP::version)
      .doc() =
      "KissICP LiDAR-only pipeline for point cloud registration. KissICP is "
      "designed to be simple and easy to use, while still providing good "
      "performance with minimal parameter tuning required across datasets.";
#endif

#ifdef EVALIO_LIO_SAM
  nb::class_<LioSam, evalio::Pipeline>(m, "LioSAM")
      .def(nb::init<>())
      .def_static("name", &LioSam::name)
      .def_static("default_params", &LioSam::default_params)
      .def_static("url", &LioSam::url)
      .def_static("version", &LioSam::version)
      .doc() =
      "Lidar-Inertial Smoothing and Mapping (LioSAM) pipeline. LioSAM is an "
      "extension of LOAM (=> uses planar and edge features) that additionally "
      "utilizes an IMU for initializing ICP steps and for dewarping points";
#endif

#ifdef EVALIO_LOAM
  nb::class_<LOAM, evalio::Pipeline>(m, "LOAM")
      .def(nb::init<>())
      .def_static("name", &LOAM::name)
      .def_static("default_params", &LOAM::default_params)
      .def_static("url", &LOAM::url)
      .def_static("version", &LOAM::version);

#endif
}
} // namespace evalio
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

#ifdef EVALIO_GENZ_ICP
  #include "bindings/pipelines/genz_icp.h"
#endif

#ifdef EVALIO_MAD_ICP
  #include "bindings/pipelines/mad_icp.h"
#endif

#ifdef EVALIO_CT_ICP
  #include "bindings/pipelines/ct_icp.h"
#endif

namespace evalio {
inline void makePipelines(nb::module_& m) {
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
    .def_static("version", &LOAM::version)
    .doc() =
    "Lidar Odometry and Mapping (LOAM) pipeline. LOAM is a baseline "
    "lidar-only odometry method that pioneered feature-based ICP. "
    "Our implementation permits both scan-to-scan or scan-to-map matching.";
#endif

#ifdef EVALIO_GENZ_ICP
  nb::class_<GenZICP, evalio::Pipeline>(m, "GenZICP")
    .def(nb::init<>())
    .def_static("name", &GenZICP::name)
    .def_static("default_params", &GenZICP::default_params)
    .def_static("url", &GenZICP::url)
    .def_static("version", &GenZICP::version)
    .doc() =
    "Genz-ICP LiDAR-only pipeline is an extension of KissICP that "
    "additionally estimates normals in the local submap voxel map for "
    "increased robustness. It also includes a novel weighting scheme for"
    " weighting point-to-plane and point-to-point correspondences.";
#endif

#ifdef EVALIO_MAD_ICP
  nb::class_<MadICP, evalio::Pipeline>(m, "MadICP")
    .def(nb::init<>())
    .def_static("name", &MadICP::name)
    .def_static("default_params", &MadICP::default_params)
    .def_static("url", &MadICP::url)
    .def_static("version", &MadICP::version)
    .doc() =
    "MAD-ICP LiDAR-only pipeline is an extension of KissICP that "
    "utilizes a novel kd-tree representation that implicitly computes "
    "normals to perform point-to-plane registration.";
#endif

#ifdef EVALIO_CT_ICP
  nb::class_<CTICP, evalio::Pipeline>(m, "CTICP")
    .def(nb::init<>())
    .def_static("name", &CTICP::name)
    .def_static("default_params", &CTICP::default_params)
    .def_static("url", &CTICP::url)
    .def_static("version", &CTICP::version)
    .doc() =
    "CT-ICP LiDAR-only pipeline performs continuous-time ICP over "
    "a small window of scans to perform more accurate dewarping performance. "
    "This is the version based on the 2022-ICRA paper.";
#endif
}
} // namespace evalio

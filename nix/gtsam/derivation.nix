{ lib
, stdenv
  # build system
, cmake
, ninja
  # dependencies
, boost
, fetchzip
, eigen
, metis
, gtest
, tbb_2021_11
, useQuaternions ? false
, runTests ? false
,
}:
stdenv.mkDerivation {
  pname = "gtsam";
  version = "4.2";

  src = fetchzip {
    url = "https://github.com/borglab/gtsam/archive/refs/tags/4.2.tar.gz";
    hash = "sha256-HjpGrHclpm2XsicZty/rX/RM/762wzmj4AAoEfni8es=";
  };

  nativeBuildInputs = [
    cmake
    ninja
  ];

  buildInputs = lib.optional runTests gtest;

  cmakeFlags = with lib.strings; [
    (cmakeOptionType "STRING" "CMAKE_BUILD_TYPE" "Release")
    (cmakeBool "GTSAM_USE_SYSTEM_EIGEN" true)
    (cmakeBool "GTSAM_WITH_TBB" true)
    (cmakeBool "GTSAM_USE_SYSTEM_METIS" true)
    (cmakeBool "GTSAM_USE_QUATERNIONS" useQuaternions)
    # most of these should be off by default, but we'll turn them off explicitly
    (cmakeBool "GTSAM_BUILD_DOCS" false)
    (cmakeBool "GTSAM_BUILD_PYTHON" false)
    (cmakeBool "GTSAM_INSTALL_MATLAB_TOOLBOX" false)
    (cmakeBool "GTSAM_BUILD_EXAMPLES" false)
    (cmakeBool "GTSAM_BUILD_EXAMPLES_ALWAYS" false)
  ];

  propagatedBuildInputs = [
    eigen
    boost
    metis
    tbb_2021_11
  ];

  patches = [ ./patch.txt ];
  doCheck = runTests;

  meta = with lib; {
    description = "Factor Graphs for Sensor Fusion in Robotics";
    longDescription = "C++ library that implements sensor fusion for robotics and computer vision applications, including SLAM (Simultaneous Localization and Mapping), VO (Visual Odometry), and SFM (Structure from Motion). It uses factor graphs and Bayes networks as the underlying computing paradigm rather than sparse matrices to optimize for the most probable configuration or an optimal plan. Coupled with a capable sensor front-end (not provided here), GTSAM powers many impressive autonomous systems, in both academia and industry.";
    homepage = "https://gtsam.org";
    license = licenses.bsd3;
    maintainers = with maintainers; [ contagon ];
  };
}

{ lib
, stdenv
, fetchFromGitHub
  # build system
, cmake
, ninja
  # dependencies
, eigen
, sophus
, fmt
, robin-map
, tbb_2021_11
}:

stdenv.mkDerivation rec {
  pname = "kiss-icp";
  version = "1.2.2";

  src = fetchFromGitHub {
    owner = "PRBonn";
    repo = "kiss-icp";
    rev = "v${version}";
    hash = "sha256-QrW+TZ41TgiFu8a2azLQ3XUQcalnuW69ieeuEEviP4o=";
  };

  # Patch to put install information in
  # https://cmake.org/cmake/help/latest/guide/tutorial/Adding%20Export%20Configuration.html
  patches = [ ./kiss_icp.patch ];

  nativeBuildInputs = [
    cmake
    ninja
  ];

  preConfigure = "cd cpp/kiss_icp";

  cmakeFlags = with lib.strings; [
    (cmakeBool "USE_SYSTEM_EIGEN3" true)
    (cmakeBool "USE_SYSTEM_TSL-ROBIN-MAP" true)
    (cmakeBool "USE_SYSTEM_SOPHUS" true)
    (cmakeBool "USE_SYSTEM_TBB" true)
  ];

  propagatedBuildInputs = [
    eigen
    robin-map
    sophus
    fmt # needed by sophus
    tbb_2021_11
  ];

  meta = {
    description = "KISS-ICP: Keep It Simple and Straightforward - Iterative Closest Point";
    homepage = "https://github.com/PRBonn/kiss-icp/tree/main";
    license = lib.licenses.mit;
    platforms = lib.platforms.all;
  };
}

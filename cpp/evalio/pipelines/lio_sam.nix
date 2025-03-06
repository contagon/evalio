{ lib
, stdenv
, fetchFromGitHub
  # build system
, cmake
, ninja
  # dependencies
, eigen
, gtsam
, opencv
, pcl
}:

# TODO: Should probably guarantee OpenMP is available somehow here
stdenv.mkDerivation {
  pname = "lio-sam";
  version = "0.1.0";

  src = if builtins.pathExists /home/contagon/Research/LIO-SAM then /home/contagon/Research/LIO-SAM else
  fetchFromGitHub {
    owner = "contagon";
    repo = "LIO-SAM";
    rev = "todo";
    hash = "";
  };

  nativeBuildInputs = [
    cmake
    ninja
  ];

  propagatedBuildInputs = [
    eigen
    gtsam
    opencv
    pcl
  ];

  meta = {
    description = "LIO-SAM: Lidar-Inertial Odometry and Mapping";
    homepage = "https://github.com/contagon/LIO-SAM/tree/master";
    license = lib.licenses.bsd3;
    platforms = lib.platforms.all;
  };
}

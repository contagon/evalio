{ lib
, callPackage
, python3
, cmake
, ninja
,
}:

let
  kiss_icp = callPackage ./cpp/evalio/pipelines/kiss_icp.nix { };
  rosbags = callPackage ./rosbags.nix { };
in
python3.pkgs.buildPythonPackage {
  pname = "evalio";
  version = "0.1.0";
  pyproject = true;

  src = ./.;

  # Example
  # https://github.com/NixOS/nixpkgs/blob/master/pkgs/development/python-modules/rapidfuzz/default.nix

  # TODO: Something isn't working quite right here still
  build-system = with python3.pkgs; [
    cmake
    ninja
    scikit-build-core
    pybind11
  ];

  # TODO: Need lots more here
  dependencies = with python3.pkgs; [
    # not sure which section to put pipelines in
    kiss_icp
    # python depends (are these even necessary?)
    argcomplete
    numpy
    pyyaml
    rosbags
    tabulate
    tqdm
  ];

  meta = {
    description = "Evaluate Lidar-Inertial Odometry";
    homepage = "https://github.com/contagon/evalio/";
    license = lib.licenses.mit;
    platforms = lib.platforms.all;
    maintainers = with lib.maintainers; [ contagon ];
  };
}

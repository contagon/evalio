{ lib
, callPackage
, python3
, cmake
, ninja
, eigen
, gis ? import
    (fetchTarball {
      url = "https://github.com/icetan/nix-git-ignore-source/archive/v1.0.0.tar.gz";
      sha256 = "1mnpab6x0bnshpp0acddylpa3dslhzd2m1kk3n0k23jqf9ddz57k";
    })
    { }
,
}:

let
  # python depends
  rosbags = callPackage ./rosbags.nix { };
  # pipelines
  kiss_icp = callPackage ./cpp/evalio/pipelines/kiss_icp.nix { };
in
python3.pkgs.buildPythonPackage {
  pname = "evalio";
  version = "0.1.0";
  pyproject = true;

  src = gis.gitIgnoreSource ./.;

  dontUseCmakeConfigure = true;

  build-system = with python3.pkgs; [
    cmake
    ninja
    pybind11
    scikit-build-core
    # TODO: Not sure why these are needed
    pathspec
    pyproject-metadata
  ];

  dependencies = with python3.pkgs; [
    # not sure which section to put pipelines in
    eigen
    kiss_icp
    # python depends
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

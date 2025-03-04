{ lib
, fetchPypi
, python3
}:

python3.pkgs.buildPythonPackage rec {
  pname = "rosbags";
  version = "0.10.7";

  src = fetchPypi {
    inherit pname version;
    hash = "sha256-PVH4S3rIt9QVflqlIHOgTXVK+lfIOLnWWO3XL84H+E0=";
  };

  build-system = with python3.pkgs; [
    # poetry-core
    setuptools
    setuptools_scm
  ];

  dependencies = with python3.pkgs; [
    # poetry-core
    lz4
    numpy
    zstandard
    ruamel-yaml
  ];

  pyproject = true;
  docheck = false;

  meta = with lib; {
    homepage = "https://gitlab.com/ternaris/rosbags";
    description = "Rosbags is the pure python library for everything rosbag";
    license = licenses.asl20;
    maintainers = with maintainers; [ etherswangel ];
  };
}

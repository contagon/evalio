from . import _cpp, datasets, pipelines, types, utils, stats
from ._cpp import abi_tag as _abi_tag

import warnings
from tqdm import TqdmExperimentalWarning
import atexit


# remove false nanobind reference leak warnings
# https://github.com/wjakob/nanobind/discussions/13
def cleanup():
    import typing

    for cleanup in typing._cleanups:  # type: ignore
        cleanup()


atexit.register(cleanup)

# Ignore tqdm rich warnings
warnings.filterwarnings("ignore", category=TqdmExperimentalWarning)

__version__ = "0.3.0"
__all__ = [
    "_abi_tag",
    "datasets",
    "_cpp",
    "pipelines",
    "stats",
    "types",
    "utils",
    "__version__",
]

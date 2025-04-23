from . import _cpp, datasets, pipelines, types, utils

# remove false nanobind reference leak warnings
# https://github.com/wjakob/nanobind/discussions/13
import atexit


def cleanup():
    import typing

    for cleanup in typing._cleanups:  # type: ignore
        cleanup()


atexit.register(cleanup)

__version__ = "0.2.0"
__all__ = [
    "datasets",
    "_cpp",
    "pipelines",
    "types",
    "utils",
    "__version__",
]

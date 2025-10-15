import atexit
from typing import cast
import warnings
import os
import importlib

from tqdm import TqdmExperimentalWarning

from . import _cpp, datasets, pipelines, stats, types, utils
from ._cpp import abi_tag as _abi_tag


# remove false nanobind reference leak warnings
# https://github.com/wjakob/nanobind/discussions/13
def cleanup():
    import typing

    for cleanup in typing._cleanups:  # type: ignore
        cleanup()


atexit.register(cleanup)

# Ignore tqdm rich warnings
warnings.filterwarnings("ignore", category=TqdmExperimentalWarning)


# Register any custom modules specified in the environment
def _register_custom_modules(module_name: str):
    # Make sure we only attempt to register each module once
    if not hasattr(_register_custom_modules, "attempted"):
        _register_custom_modules.attempted = set()  # type: ignore

    if module_name in _register_custom_modules.attempted:  # type: ignore
        return
    _register_custom_modules.attempted.add(module_name)  # type: ignore

    try:
        module = importlib.import_module(module_name)
        pl_out = pipelines.register_pipeline(module=module)
        ds_out = datasets.register_dataset(module=module)

        if cast(int, pl_out) + cast(int, ds_out) == 0:
            utils.print_warning(
                f"No pipelines or datasets found in custom module '{module_name}'"
            )

    except ImportError:
        utils.print_warning(f"Failed to import custom module '{module_name}'")


if "EVALIO_CUSTOM" in os.environ:
    for module_name in os.environ["EVALIO_CUSTOM"].split(","):
        module_name = module_name.strip()
        _register_custom_modules(module_name)


__version__ = "0.4.0"
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

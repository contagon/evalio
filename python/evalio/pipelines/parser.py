import importlib
from inspect import isclass
from types import ModuleType
from typing import Any, Optional

from evalio import pipelines
from evalio.pipelines import Pipeline

_PIPELINES: set[type[Pipeline]] = set()


# ------------------------- Handle Registration of Pipelines ------------------------- #
def _is_pipe(obj: Any) -> bool:
    return (
        isclass(obj) and issubclass(obj, Pipeline) and obj.__name__ != Pipeline.__name__
    )


def _search_module(module: ModuleType) -> set[type[Pipeline]]:
    return {c for c in module.__dict__.values() if _is_pipe(c)}


def register_pipeline(
    pipeline: Optional[type[Pipeline]] = None,
    module: Optional[ModuleType | str] = None,
):
    """Add a pipeline or a module containing pipelines to the registry.

    Args:
        pipeline (Optional[type[Pipeline]], optional): A specific pipeline class to add. Defaults to None.
        module (Optional[ModuleType  |  str], optional): The module to search for pipelines. Defaults to None.

    Raises:
        ValueError: If the module does not contain any pipelines.
        ValueError: If the pipeline is not a valid Pipeline subclass.
        ValueError: If both module and pipeline are None.
    """
    global _PIPELINES

    if module is not None:
        if isinstance(module, str):
            try:
                module = importlib.import_module(module)
            except ImportError:
                raise ValueError(f"Failed to import '{module}'")

        if len(new_pipes := _search_module(module)) > 0:
            _PIPELINES.update(new_pipes)
        else:
            raise ValueError(f"Module {module.__name__} does not contain any pipelines")

    if pipeline is not None:
        if _is_pipe(pipeline):
            _PIPELINES.add(pipeline)
        else:
            raise ValueError(f"{pipeline} is not a valid Pipeline subclass")


def all_pipelines() -> dict[str, type[Pipeline]]:
    """Get all registered pipelines.

    Returns:
        dict[str, type[Pipeline]]: A dictionary mapping pipeline names to their classes.
    """
    global _PIPELINES
    return {p.name(): p for p in _PIPELINES}


def get_pipeline(name: str) -> Optional[type[Pipeline]]:
    """Get a pipeline class by its name.

    Args:
        name (str): The name of the pipeline.

    Returns:
        Optional[type[Pipeline]]: The pipeline class, or None if not found.
    """
    return all_pipelines().get(name, None)


register_pipeline(module=pipelines)

# ------------------------- Handle yaml parsing ------------------------- #

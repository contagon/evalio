from __future__ import annotations

import importlib
from inspect import isclass
import itertools
from types import ModuleType
from typing import Any, Optional, cast, Sequence

from evalio import pipelines
from evalio.pipelines import Pipeline
from evalio.types import Param


_PIPELINES: set[type[Pipeline]] = set()


class PipelineNotFound(Exception):
    def __init__(self, name: str):
        super().__init__(f"Pipeline '{name}' not found")
        self.name = name


class InvalidPipelineConfig(Exception):
    def __init__(self, config: str):
        super().__init__(f"Invalid config: '{config}'")
        self.config = config


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


def get_pipeline(name: str) -> type[Pipeline] | PipelineNotFound:
    """Get a pipeline class by its name.

    Args:
        name (str): The name of the pipeline.

    Returns:
        Optional[type[Pipeline]]: The pipeline class, or None if not found.
    """
    return all_pipelines().get(name, PipelineNotFound(name))


register_pipeline(module=pipelines)


# ------------------------- Handle yaml parsing ------------------------- #
def _sweep(
    sweep: dict[str, Param],
    params: dict[str, Param],
    pipe: type[Pipeline],
) -> list[tuple[type[Pipeline], dict[str, Param]]]:
    keys, values = zip(*sweep.items())
    results: list[tuple[type[Pipeline], dict[str, Param]]] = []
    for options in itertools.product(*values):
        p = params.copy()
        for k, o in zip(keys, options):
            p[k] = o
        results.append((pipe, p))
    return results


ConfigError = PipelineNotFound | InvalidPipelineConfig


def parse_config(
    p: str | dict[str, Param] | Sequence[str | dict[str, Param]],
) -> list[tuple[type[Pipeline], dict[str, Param]]] | ConfigError:
    """Parse a pipeline configuration.

    Args:
        p (str | dict[str, Param] | Sequence[str | dict[str, Param]]): The pipeline configuration.

    Raises:
        ValueError: If the pipeline is not found.
        ValueError: If the configuration is invalid.

    Returns:
        list[tuple[type[Pipeline], dict[str, Param]]]: A list of tuples containing the pipeline class and its parameters.
    """
    if isinstance(p, str):
        pipe = get_pipeline(p)
        if isinstance(pipe, PipelineNotFound):
            return pipe
        return [(pipe, {})]

    elif isinstance(p, dict):
        name = p.pop("name", None)
        if name is None:
            return InvalidPipelineConfig(f"Need pipeline name: {str(p)}")

        pipe = get_pipeline(cast(str, name))
        if isinstance(pipe, PipelineNotFound):
            return pipe

        if "sweep" in p:
            sweep = cast(dict[str, Param], p.pop("sweep"))
            return _sweep(sweep, p, pipe)
        else:
            return [(pipe, p)]

    elif isinstance(p, list):
        results = [parse_config(x) for x in p]
        for r in results:
            if isinstance(r, ConfigError):
                return r
        results = cast(list[list[tuple[type[Pipeline], dict[str, Param]]]], results)
        return list(itertools.chain.from_iterable(results))

    else:
        return InvalidPipelineConfig(f"Invalid pipeline configuration {p}")

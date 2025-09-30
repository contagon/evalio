from __future__ import annotations

from copy import deepcopy
import importlib
from inspect import isclass
import itertools
from types import ModuleType
from typing import Any, Optional, cast, Sequence

from evalio import pipelines
from evalio.pipelines import Pipeline
from evalio.types import Param
from evalio.utils import CustomException


_PIPELINES: set[type[Pipeline]] = set()


class PipelineNotFound(CustomException):
    def __init__(self, name: str):
        super().__init__(f"Pipeline '{name}' not found")
        self.name = name


class InvalidPipelineConfig(CustomException):
    def __init__(self, config: str):
        super().__init__(f"Invalid config: '{config}'")
        self.config = config


class UnusedPipelineParam(CustomException):
    def __init__(self, param: str, pipeline: str):
        super().__init__(f"Parameter '{param}' is not used in pipeline '{pipeline}'")
        self.param = param
        self.pipeline = pipeline


class InvalidPipelineParamType(CustomException):
    def __init__(self, param: str, expected_type: type, actual_type: type):
        super().__init__(
            f"Parameter '{param}' has invalid type. Expected '{expected_type.__name__}', got '{actual_type.__name__}'"
        )
        self.param = param
        self.expected_type = expected_type
        self.actual_type = actual_type


PipelineConfigError = (
    PipelineNotFound
    | InvalidPipelineConfig
    | UnusedPipelineParam
    | InvalidPipelineParamType
)


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
) -> int | ImportError:
    """Add a pipeline or a module containing pipelines to the registry.

    Args:
        pipeline (Optional[type[Pipeline]], optional): A specific pipeline class to add. Defaults to None.
        module (Optional[ModuleType  |  str], optional): The module to search for pipelines. Defaults to None.
    """
    global _PIPELINES

    total = 0
    if module is not None:
        if isinstance(module, str):
            try:
                module = importlib.import_module(module)
            except ImportError as e:
                return e

        new_pipes = _search_module(module)
        _PIPELINES.update(new_pipes)
        total += len(new_pipes)

    if pipeline is not None and _is_pipe(pipeline):
        _PIPELINES.add(pipeline)
        total += 1

    return total


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
    name: str,
    pipe: type[Pipeline],
) -> list[tuple[str, type[Pipeline], dict[str, Param]]] | PipelineConfigError:
    keys, values = zip(*sweep.items())
    results: list[tuple[str, type[Pipeline], dict[str, Param]]] = []
    for options in itertools.product(*values):
        parsed_name = deepcopy(name)
        p = params.copy()
        for k, o in zip(keys, options):
            p[k] = o
            parsed_name += f"__{k}-{o}"
        err = validate_params(pipe, p)
        if err is not None:
            return err
        results.append((parsed_name, pipe, p))
    return results


def validate_params(
    pipe: type[Pipeline],
    params: dict[str, Param],
) -> None | PipelineConfigError:
    """Validate the parameters for a given pipeline.

    Args:
        pipe (type[Pipeline]): The pipeline class.
        params (dict[str, Param]): The parameters to validate.

    Returns:
        Optional[PipelineConfigError]: An error if validation fails, otherwise None.
    """
    default_params = pipe.default_params()
    for p in params:
        if p not in default_params:
            return UnusedPipelineParam(p, pipe.name())

        expected_type = type(default_params[p])
        actual_type = type(params[p])
        if actual_type != expected_type:
            return InvalidPipelineParamType(p, expected_type, actual_type)

    return None


def parse_config(
    p: str | dict[str, Param] | Sequence[str | dict[str, Param]],
) -> list[tuple[str, type[Pipeline], dict[str, Param]]] | PipelineConfigError:
    """Parse a pipeline configuration.

    Args:
        p (str | dict[str, Param] | Sequence[str | dict[str, Param]]): The pipeline configuration.

    Returns:
        list[tuple[type[Pipeline], dict[str, Param]]]: A list of tuples containing the pipeline class and its parameters.
    """
    if isinstance(p, str):
        pipe = get_pipeline(p)
        if isinstance(pipe, PipelineNotFound):
            return pipe
        return [(p, pipe, {})]

    elif isinstance(p, dict):
        pipe_name = p.pop("pipeline", None)
        if pipe_name is None:
            return InvalidPipelineConfig(f"Need pipeline name: {str(p)}")
        pipe_name = cast(str, pipe_name)

        name = p.pop("name", None)
        if name is None:
            name = pipe_name
        name = cast(str, name)

        pipe = get_pipeline(pipe_name)
        if isinstance(pipe, PipelineNotFound):
            return pipe

        if "sweep" in p:
            sweep = cast(dict[str, Param], p.pop("sweep"))
            return _sweep(sweep, p, name, pipe)
        else:
            err = validate_params(pipe, p)
            if err is not None:
                return err

            return [(name, pipe, p)]

    elif isinstance(p, list):
        results = [parse_config(x) for x in p]
        for r in results:
            if isinstance(r, PipelineConfigError):
                return r
        results = cast(
            list[list[tuple[str, type[Pipeline], dict[str, Param]]]], results
        )
        return list(itertools.chain.from_iterable(results))

    else:
        return InvalidPipelineConfig(f"Invalid pipeline configuration {p}")

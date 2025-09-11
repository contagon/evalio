from types import ModuleType
import functools
import itertools
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Sequence, cast

from inspect import isclass
from evalio.utils import print_warning
import yaml
import os
import importlib

import evalio
from evalio.datasets import Dataset
from evalio.pipelines import Pipeline
from evalio import Param


# ------------------------- Parsing input ------------------------- #
# TODO: Find a better way to handle lengths here
# TODO: Make an experiment class to wrap all of this?
@dataclass
class DatasetBuilder:
    dataset: Dataset
    length: Optional[int] = None

    @staticmethod
    def _search_module(module: ModuleType) -> dict[str, type[Dataset]]:
        return dict(
            (cls.dataset_name(), cls)
            for cls in module.__dict__.values()
            if isclass(cls)
            and issubclass(cls, Dataset)
            and cls.__name__ != evalio.datasets.Dataset.__name__
        )

    @staticmethod
    @functools.cache
    def all_datasets() -> dict[str, type[Dataset]]:
        datasets = DatasetBuilder._search_module(evalio.datasets)

        # Parse env variable for more
        if "EVALIO_CUSTOM" in os.environ:
            for dataset in os.environ["EVALIO_CUSTOM"].split(","):
                module: ModuleType = importlib.import_module(dataset)
                datasets |= DatasetBuilder._search_module(module)

        return datasets

    @classmethod
    @functools.cache
    def _get_dataset(cls, name: str) -> type[Dataset]:
        DatasetType = cls.all_datasets().get(name, None)
        if DatasetType is None:
            raise ValueError(f"Dataset {name} not found")
        return DatasetType

    @classmethod
    def parse(
        cls,
        d: None
        | str
        | dict[str, int | float | str]
        | Sequence[dict[str, int | float | str] | str],
    ) -> list["DatasetBuilder"]:
        # If empty just return
        if d is None:
            return []

        # If just given a dataset name
        if isinstance(d, str):
            name, seq = d.split("/")
            if seq == "*":
                return [
                    DatasetBuilder(cls._get_dataset(name)(seq))
                    for seq in cls._get_dataset(name).sequences()
                ]
            else:
                return [DatasetBuilder(cls._get_dataset(name)(seq))]

        # If given a dictionary
        elif isinstance(d, dict):
            name, seq = cast(str, d.pop("name")).split("/")
            length = cast(Optional[int], d.pop("length", None))
            assert len(d) == 0, f"Invalid dataset configuration {d}"
            if seq == "*":
                return [
                    DatasetBuilder(cls._get_dataset(name)(seq), length)
                    for seq in cls._get_dataset(name).sequences()
                ]
            else:
                return [DatasetBuilder(cls._get_dataset(name)(seq), length)]

        # If given a list, iterate
        elif isinstance(d, list):
            results = [DatasetBuilder.parse(x) for x in d]
            return list(itertools.chain.from_iterable(results))

        else:
            raise ValueError(f"Invalid dataset configuration {d}")

    def as_dict(self) -> dict[str, Param]:
        out: dict[str, Param] = {"name": self.dataset.full_name}
        if self.length is not None:
            out["length"] = self.length

        return out

    def is_downloaded(self) -> bool:
        return self.dataset.is_downloaded()

    def download(self) -> None:
        self.dataset.download()

    def build(self) -> Dataset:
        return self.dataset

    def __str__(self) -> str:
        return self.dataset


PIPELINE_NAME = Pipeline.__name__
PIPELINE_METHODS = [m for m in dir(Pipeline) if not m.startswith("_")]


@dataclass
class PipelineBuilder:
    name: str
    pipeline: type[Pipeline]
    params: dict[str, Param]

    def __post_init__(self):
        # Make sure all parameters are valid
        all_params = self.pipeline.default_params()
        for key in self.params.keys():
            if key not in all_params:
                raise ValueError(
                    f"Invalid parameter {key} for pipeline {self.pipeline.name()}"
                )

        # Save all params to file later
        all_params.update(self.params)
        self.params = all_params

    @staticmethod
    def _is_pipeline(obj: Any) -> bool:
        # First check the normal way to short circuit
        if issubclass(obj, Pipeline):
            return True

        # If Pipeline isn't a parent
        if not any(parent.__name__ == PIPELINE_NAME for parent in obj.__mro__):
            return False

        # If it's missing methods
        for method in PIPELINE_METHODS:
            if not hasattr(obj, method):
                return False

        return True

    @staticmethod
    def _search_module(module: ModuleType) -> dict[str, type[Pipeline]]:
        return dict(
            (cls.name(), cls)
            for cls in module.__dict__.values()
            if isclass(cls)
            and PipelineBuilder._is_pipeline(cls)
            and cls.__name__ != evalio.pipelines.Pipeline.__name__
        )

    @staticmethod
    @functools.lru_cache
    def all_pipelines() -> dict[str, type[Pipeline]]:
        pipelines = PipelineBuilder._search_module(evalio.pipelines)

        # Parse env variable for more
        if "EVALIO_CUSTOM" in os.environ:
            for dataset in os.environ["EVALIO_CUSTOM"].split(","):
                module = importlib.import_module(dataset)
                pipelines |= PipelineBuilder._search_module(module)

        return pipelines

    @classmethod
    @functools.lru_cache
    def _get_pipeline(cls, name: str) -> type[Pipeline]:
        PipelineType = cls.all_pipelines().get(name, None)
        if PipelineType is None:
            raise ValueError(f"Pipeline {name} not found")
        return PipelineType

    @classmethod
    def parse(
        cls,
        p: None | str | dict[str, int | float | str] | Sequence[dict[str, Param] | str],
    ) -> list["PipelineBuilder"]:
        # If empty just return
        if p is None:
            return []

        # If just given a pipeline name
        if isinstance(p, str):
            return [PipelineBuilder(p, cls._get_pipeline(p), {})]

        # If given a dictionary
        elif isinstance(p, dict):
            kind = p.pop("pipeline")
            name = cast(str, p.pop("name", kind))
            kind = cls._get_pipeline(kind)
            # If the dictionary has a sweep parameter in it
            if "sweep" in p:
                sweep = cast(dict[str, list[Param]], p.pop("sweep"))
                keys, values = zip(*sweep.items())
                results: list[PipelineBuilder] = []
                for options in itertools.product(*values):
                    parsed_name = deepcopy(name)
                    params = deepcopy(p)
                    for k, o in zip(keys, options):
                        params[k] = o
                        parsed_name += f"__{k}.{o}"
                    results.append(PipelineBuilder(parsed_name, kind, params))
                return results
            else:
                return [PipelineBuilder(name, kind, p)]

        # If given a list, iterate
        elif isinstance(p, list):
            pipes = [PipelineBuilder.parse(x) for x in p]
            return list(itertools.chain.from_iterable(pipes))

        else:
            raise ValueError(f"Invalid pipeline configuration {p}")

    def as_dict(self) -> dict[str, Param]:
        return {"name": self.name, "pipeline": self.pipeline.name(), **self.params}

    def build(self, dataset: Dataset) -> Pipeline:
        pipe = self.pipeline()
        # Set user params
        params = pipe.set_params(self.params)
        if len(params) > 0:
            for k, v in params.items():
                print_warning(
                    f"Pipeline {self.name} has unused parameters: {k}={v}. "
                    "Please check your configuration."
                )

        # Set dataset params
        pipe.set_imu_params(dataset.imu_params())
        pipe.set_lidar_params(dataset.lidar_params())
        pipe.set_imu_T_lidar(dataset.imu_T_lidar())
        # Initialize pipeline
        pipe.initialize()
        return pipe

    def __str__(self):
        return f"{self.name}"


def parse_config(
    config_file: Optional[Path],
) -> tuple[list[PipelineBuilder], list[DatasetBuilder], Optional[Path]]:
    if config_file is None:
        return ([], [], None)

    with open(config_file, "r") as f:
        params = yaml.safe_load(f)

    # get output directory
    out = Path(params["output_dir"]) if "output_dir" in params else None

    # process datasets & make sure they are downloaded by building
    datasets = DatasetBuilder.parse(params.get("datasets", None))
    for d in datasets:
        d.build()

    # process pipelines
    pipelines = PipelineBuilder.parse(params.get("pipelines", None))

    return pipelines, datasets, out

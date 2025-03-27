import itertools
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

from inspect import isclass
import yaml
import os
import importlib

from rich import print

import evalio
from evalio.datasets import Dataset
from evalio.pipelines import Pipeline


# TODO: Would it be better to have modules as some form of global variable we can register to?
# It's kind of a pain to have to pass it around everywhere
# Maybe a class variable in the parser?


# ------------------------- Parsing input ------------------------- #
# TODO: Find a better way to handle lengths here
# TODO: Make an experiment class to wrap all of this?
@dataclass
class DatasetBuilder:
    dataset: Dataset
    length: Optional[int] = None

    @staticmethod
    def _search_module(module) -> dict[str, type[Dataset]]:
        return dict(
            (cls.dataset_name(), cls)
            for cls in module.__dict__.values()
            if isclass(cls)
            and issubclass(cls, Dataset)
            and cls.__name__ != evalio.datasets.Dataset.__name__
        )

    @staticmethod
    def _all_datasets(
        custom_modules: Optional[list[str]] = None,
    ) -> dict[str, type[Dataset]]:
        datasets = DatasetBuilder._search_module(evalio.datasets)

        # Parse env variable for more
        if "EVALIO_CUSTOM" in os.environ:
            for dataset in os.environ["EVALIO_CUSTOM"].split(","):
                module = importlib.import_module(dataset)
                datasets |= DatasetBuilder._search_module(module)

        if custom_modules is not None:
            for c in custom_modules:
                module = importlib.import_module(c)
                new = DatasetBuilder._search_module(module)
                if len(new) == 0:
                    print(
                        f"[bold red]Warning:[/bold red] Unable to find any datasets in [green]{c}[/green]"
                    )
                datasets |= new

        return datasets

    @classmethod
    def _get_dataset(
        cls, name: str, custom_modules: Optional[list[str]] = None
    ) -> type[Dataset]:
        DatasetType = cls._all_datasets(custom_modules).get(name, None)
        if DatasetType is None:
            raise ValueError(f"Dataset {name} not found")
        return DatasetType

    @classmethod
    def parse(
        cls,
        d: dict | str | Sequence[dict | str],
        custom_modules: Optional[list[str]] = None,
    ) -> list["DatasetBuilder"]:
        # If empty just return
        if d is None:
            return []

        # If just given a dataset name
        if isinstance(d, str):
            name, seq = d.split("/")
            if seq == "*":
                return [
                    DatasetBuilder(cls._get_dataset(name, custom_modules)(seq))
                    for seq in cls._get_dataset(name, custom_modules).sequences()
                ]
            else:
                return [DatasetBuilder(cls._get_dataset(name, custom_modules)(seq))]

        # If given a dictionary
        elif isinstance(d, dict):
            name, seq = d.pop("name").split("/")
            length = d.pop("length", None)
            assert len(d) == 0, f"Invalid dataset configuration {d}"
            if seq == "*":
                return [
                    DatasetBuilder(cls._get_dataset(name, custom_modules)(seq), length)
                    for seq in cls._get_dataset(name, custom_modules).sequences()
                ]
            else:
                return [
                    DatasetBuilder(cls._get_dataset(name, custom_modules)(seq), length)
                ]

        # If given a list, iterate
        elif isinstance(d, list):
            results = [DatasetBuilder.parse(x, custom_modules) for x in d]
            return list(itertools.chain.from_iterable(results))

        else:
            raise ValueError(f"Invalid dataset configuration {d}")

    def as_dict(self) -> dict[str, str | int]:
        out: dict[str, str | int] = {"name": self.dataset.full_name}
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


@dataclass
class PipelineBuilder:
    name: str
    pipeline: type[Pipeline]
    params: dict

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
    def _search_module(module) -> dict[str, type[Pipeline]]:
        return dict(
            (cls.name(), cls)
            for cls in module.__dict__.values()
            if isclass(cls)
            and issubclass(cls, Pipeline)
            and cls.__name__ != evalio.pipelines.Pipeline.__name__
        )

    @staticmethod
    def _all_pipelines(
        custom_modules: Optional[list[str]] = None,
    ) -> dict[str, type[Pipeline]]:
        pipelines = PipelineBuilder._search_module(evalio.pipelines)

        # Parse env variable for more
        if "EVALIO_CUSTOM" in os.environ:
            for dataset in os.environ["EVALIO_CUSTOM"].split(","):
                module = importlib.import_module(dataset)
                pipelines |= PipelineBuilder._search_module(module)

        if custom_modules is not None:
            for c in custom_modules:
                module = importlib.import_module(c)
                new = PipelineBuilder._search_module(module)
                if len(new) == 0:
                    print(
                        f"[bold red]Warning:[/bold red] Unable to find any pipelines in [green]{c}[/green]"
                    )
                pipelines |= new

        return pipelines

    @classmethod
    def _get_pipeline(
        cls,
        name: str,
        custom_modules: Optional[list[str]] = None,
    ) -> type[Pipeline]:
        PipelineType = cls._all_pipelines(custom_modules).get(name, None)
        if PipelineType is None:
            raise ValueError(f"Pipeline {name} not found")
        return PipelineType

    @classmethod
    def parse(
        cls,
        p: dict | str | Sequence[dict | str],
        custom_modules: Optional[list[str]] = None,
    ) -> list["PipelineBuilder"]:
        # If empty just return
        if p is None:
            return []

        # If just given a pipeline name
        if isinstance(p, str):
            return [PipelineBuilder(p, cls._get_pipeline(p, custom_modules), {})]

        # If given a dictionary
        elif isinstance(p, dict):
            kind = p.pop("pipeline")
            name = p.pop("name", kind)
            kind = cls._get_pipeline(kind, custom_modules)
            # If the dictionary has a sweep parameter in it
            if "sweep" in p:
                sweep = p.pop("sweep")
                keys, values = zip(*sweep.items())
                results = []
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

    def as_dict(self) -> dict:
        return {"name": self.name, "pipeline": self.pipeline.name(), **self.params}

    def build(self, dataset: Dataset) -> Pipeline:
        pipe = self.pipeline()
        # Set user params
        pipe.set_params(self.params)
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
    out = Path(params["output_dir"])

    # process datasets & make sure they are downloaded by building
    datasets = DatasetBuilder.parse(params.get("datasets", None))
    for d in datasets:
        d.build()

    # process pipelines
    pipelines = PipelineBuilder.parse(params.get("pipelines", None))

    return pipelines, datasets, out

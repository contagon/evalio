import functools
import itertools
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import yaml

import evalio
from evalio.datasets import Dataset
from evalio.pipelines import Pipeline


# ------------------------- Finding types ------------------------- #
def find_types(module, skip=None) -> dict[str, type]:
    found: dict[str, type] = {}
    found |= dict(
        (cls.name(), cls)  # type:ignore
        for cls in module.__dict__.values()
        if isinstance(cls, type) and cls.__name__ != skip.__name__
    )

    return found


# ------------------------- Parsing input ------------------------- #
@dataclass
class DatasetBuilder:
    dataset: type[Dataset]
    seq: str
    length: Optional[int] = None

    @staticmethod
    @functools.cache
    def _all_datasets() -> dict[str, type[Dataset]]:
        return find_types(
            evalio.datasets,
            skip=evalio.datasets.Dataset,
        )

    @classmethod
    @functools.cache
    def _get_dataset(cls, name: str) -> type[Dataset]:
        DatasetType = cls._all_datasets().get(name, None)
        if DatasetType is None:
            raise ValueError(f"Dataset {name} not found")
        return DatasetType

    @classmethod
    def parse(cls, d: dict | str | Sequence[dict | str]) -> Sequence["DatasetBuilder"]:
        # If empty just return
        if d is None:
            return []

        # If just given a dataset name
        if isinstance(d, str):
            name, seq = d.split("/")
            if seq == "*":
                return [
                    DatasetBuilder(cls._get_dataset(name), seq)
                    for seq in cls._get_dataset(name).sequences()
                ]
            else:
                return [DatasetBuilder(cls._get_dataset(name), seq)]

        # If given a dictionary
        elif isinstance(d, dict):
            name, seq = d.pop("name").split("/")
            length = d.pop("length", None)
            assert len(d) == 0, f"Invalid dataset configuration {d}"
            if seq == "*":
                return [
                    DatasetBuilder(cls._get_dataset(name), seq, length)
                    for seq in cls._get_dataset(name).sequences()
                ]
            else:
                return [DatasetBuilder(cls._get_dataset(name), seq, length)]

        # If given a list, iterate
        elif isinstance(d, list):
            results = [DatasetBuilder.parse(x) for x in d]
            return list(itertools.chain.from_iterable(results))

        else:
            raise ValueError(f"Invalid dataset configuration {d}")

    def as_dict(self) -> dict[str, str | int]:
        out: dict[str, str | int] = {"name": f"{self.dataset.name()}/{self.seq}"}
        if self.length is not None:
            out["length"] = self.length

        return out

    def __post_init__(self):
        self.seq = self.dataset.process_seq(self.seq)

    def check_download(self) -> bool:
        return self.dataset.check_download(self.seq)

    def download(self) -> None:
        self.dataset.download(self.seq)

    def build(self) -> Dataset:
        return self.dataset(self.seq, self.length)

    def __str__(self):
        return f"{self.dataset.name()}/{self.seq}"


@dataclass
class PipelineBuilder:
    name: str
    pipeline: type[Pipeline]
    params: dict

    def __post_init__(self):
        # Make sure all parameters are valid
        all_params = self.pipeline.default_params()
        # TODO: Find a way to handle this gracefully for the wrapper
        # Maybe a function in the pipeline class?
        # for key in self.params.keys():
        #     if key not in all_params:
        #         raise ValueError(
        #             f"Invalid parameter {key} for pipeline {self.pipeline.name()}"
        #         )

        # Save all params to file later
        all_params.update(self.params)
        self.params = all_params

    @staticmethod
    @functools.lru_cache
    def _all_pipelines() -> dict[str, type[Pipeline]]:
        return find_types(
            evalio.pipelines,
            skip=evalio.pipelines.Pipeline,
        )

    @classmethod
    @functools.lru_cache
    def _get_pipeline(cls, name: str) -> type[Pipeline]:
        PipelineType = cls._all_pipelines().get(name, None)
        if PipelineType is None:
            raise ValueError(f"Pipeline {name} not found")
        return PipelineType

    @classmethod
    def parse(cls, p: dict | str | Sequence[dict | str]) -> Sequence["PipelineBuilder"]:
        # If empty just return
        if p is None:
            return []

        # If just given a pipeline name
        if isinstance(p, str):
            return [PipelineBuilder(p, cls._get_pipeline(p), {})]

        # If given a dictionary
        elif isinstance(p, dict):
            kind = p.pop("pipeline")
            name = p.pop("name", kind)
            kind = cls._get_pipeline(kind)
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
    config_file: Path,
) -> tuple[Sequence[PipelineBuilder], Sequence[DatasetBuilder], Path]:
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

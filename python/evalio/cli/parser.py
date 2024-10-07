from dataclasses import dataclass
from pathlib import Path

import yaml

import evalio
from evalio.datasets import Dataset
from evalio.pipelines import Pipeline


# ------------------------- Finding types ------------------------- #
def find_types(module, skip=None, include_nicknames=True) -> dict[str, type]:
    found = {}
    # Include by name
    found |= dict(
        (cls.name(), cls)
        for cls in module.__dict__.values()
        if isinstance(cls, type) and cls.__name__ != skip.__name__
    )
    # Include by nickname
    if include_nicknames:
        found |= dict(
            (cls.nickname(), cls)
            for cls in module.__dict__.values()
            if isinstance(cls, type) and cls.__name__ != skip.__name__
        )

    return found


def get_datasets(include_nicknames=True) -> dict[str, Dataset]:
    return find_types(
        evalio.datasets,
        skip=evalio.datasets.Dataset,
        include_nicknames=include_nicknames,
    )


def get_pipelines(include_nicknames=True) -> dict[str, Pipeline]:
    return find_types(
        evalio.pipelines,
        skip=evalio.pipelines.Pipeline,
        include_nicknames=include_nicknames,
    )


# ------------------------- Parsing input ------------------------- #
@dataclass
class DatasetBuilder:
    dataset: Dataset
    seq: str
    length: int = None

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
    pipeline: Pipeline
    params: dict

    def build(self, dataset: Dataset) -> Pipeline:
        pipe = self.pipeline()
        pipe.set_imu_params(dataset.imu_params())
        pipe.set_lidar_params(dataset.lidar_params())
        pipe.set_imu_T_lidar(dataset.imu_T_lidar())
        for key, value in self.params.items():
            pipe.set_param(key, value)
        pipe.initialize()
        return pipe

    def __str__(self):
        return f"{self.name}"


def parse_datasets(datasets: list[str | dict]) -> list[DatasetBuilder]:
    all_datasets = get_datasets()
    valid_datasets = []
    for d in datasets:
        if isinstance(d, dict):
            length = d.get("length", None)
            d = d["name"]
        elif isinstance(d, str):
            length = None

        name, seq = d.split("/")

        # Make sure dataset exists
        DatasetType = all_datasets.get(name, None)
        if DatasetType is None:
            raise ValueError(f"Dataset {name} not found")

        # Make sure sequence exists
        if seq == "*":
            for seq in DatasetType.sequences():
                valid_datasets.append(DatasetBuilder(DatasetType, seq, length))
        else:
            valid_datasets.append(DatasetBuilder(DatasetType, seq, length))

    return valid_datasets


def parse_pipelines(pipelines: list[dict]) -> list[PipelineBuilder]:
    all_pipelines = get_pipelines()
    valid_pipelines = []
    for p in pipelines:
        if isinstance(p, dict):
            kind = p.pop("pipeline")
            name = p.pop("name", kind)
        elif isinstance(p, str):
            kind = p
            name = kind
            p = {}

        # Make sure pipeline exists
        PipelineType = all_pipelines.get(kind, None)
        if PipelineType is None:
            raise ValueError(f"Pipeline {kind} not found")

        valid_pipelines.append(PipelineBuilder(name, PipelineType, p))

    return valid_pipelines


def parse_config(
    config_file: Path,
) -> tuple[list[PipelineBuilder], list[DatasetBuilder], Path]:
    with open(config_file, "r") as f:
        params = yaml.safe_load(f)

    # get output directory
    out = Path(params["output_dir"])

    # process datasets & make sure they are downloaded by building
    datasets = parse_datasets(params["datasets"])
    for d in datasets:
        d.build()

    # process pipelines
    pipelines = parse_pipelines(params["pipelines"])

    return pipelines, datasets, out

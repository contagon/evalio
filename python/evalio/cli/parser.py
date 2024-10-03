import evalio
import yaml
from pathlib import Path

from dataclasses import dataclass

from evalio.datasets import Dataset
from evalio._cpp.pipelines import Pipeline  # type: ignore


# ------------------------- Finding types ------------------------- #
def find_types(module, skip=None):
    found = {}
    # Include by name
    found |= dict(
        (cls.name(), cls)
        for cls in module.__dict__.values()
        if isinstance(cls, type) and cls.__name__ != skip.__name__
    )
    # Include by nickname
    found |= dict(
        (cls.nickname(), cls)
        for cls in module.__dict__.values()
        if isinstance(cls, type) and cls.__name__ != skip.__name__
    )
    return found


def get_datasets():
    return find_types(evalio.datasets, skip=evalio.datasets.Dataset)


def get_pipelines():
    return find_types(evalio.pipelines, skip=evalio.pipelines.Pipeline)


# ------------------------- Parsing input ------------------------- #
@dataclass
class DatasetBuilder:
    dataset: Dataset
    seq: str

    def __post_init__(self):
        self.seq = self.dataset.process_seq(self.seq)

    def check_download(self) -> bool:
        return self.dataset.check_download(self.seq)

    def download(self) -> None:
        self.dataset.download(self.seq)

    def build(self) -> Dataset:
        return self.dataset(self.seq)

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
            setattr(pipe, key, value)
        pipe.initialize()
        return pipe


def parse_datasets(datasets: list[str]) -> list[(Dataset, str)]:
    all_datasets = get_datasets()
    valid_datasets = []
    for d in datasets:
        name, seq = d.split("/")

        # Make sure dataset exists
        DatasetType = all_datasets.get(name, None)
        if DatasetType is None:
            raise ValueError(f"Dataset {name} not found")

        # Make sure sequence exists
        if seq == "*":
            for seq in DatasetType.sequences():
                valid_datasets.append(DatasetBuilder(DatasetType, seq))
        else:
            valid_datasets.append(DatasetBuilder(DatasetType, seq))

    return valid_datasets


def parse_pipelines(pipelines: list[dict]) -> list[Pipeline]:
    all_pipelines = get_pipelines()
    valid_pipelines = []
    for p in pipelines:
        # Make sure pipeline exists
        kind = p.pop("pipeline")
        PipelineType = all_pipelines.get(kind, None)
        if PipelineType is None:
            print(all_pipelines)
            raise ValueError(f"Pipeline {kind} not found")

        name = p.pop("name")
        valid_pipelines.append(PipelineBuilder(name, PipelineType, p))

    return valid_pipelines


def parse_config(
    config_file: Path,
) -> tuple[list[PipelineBuilder], list[DatasetBuilder], Path]:
    with open(config_file, "r") as f:
        params = yaml.safe_load(f)

    # get output directory
    out = params["output_dir"]

    # process datasets & make sure they are downloaded by building
    datasets = parse_datasets(params["datasets"])
    for d in datasets:
        d.build()

    # process pipelines
    pipelines = parse_pipelines(params["pipelines"])

    return pipelines, datasets, out

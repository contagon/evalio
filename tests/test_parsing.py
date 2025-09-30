import evalio.datasets as ds
from evalio.types import Param
from evalio.datasets.parser import DatasetConfig, DatasetConfigError
import evalio.pipelines as pl
from typing import Any, Sequence
import pytest

# ------------------------- Dataset Parsing ------------------------- #
seq = ds.NewerCollege2021.quad_easy
name = seq.full_name

# fmt: off
DATASETS: list[tuple[str | DatasetConfig | Sequence[str | DatasetConfig], DatasetConfigError | list[tuple[ds.Dataset, int]]]] = [
    # good ones
    (name, [(seq, len(seq))]),
    ({"name": name}, [(seq, len(seq))]),
    ({"name": name, "length": 100}, [(seq, 100)]),
    ({"name": f"{seq.dataset_name()}/*"}, [(s, len(s)) for s in seq.sequences()]),
    # bad ones
    ("newer_college_2020/bad", ds.SequenceNotFound("newer_college_2020/bad")),
    ("newer_college_123/*", ds.DatasetNotFound(name="newer_college_123")),
    ("newer_college_123/*", ds.DatasetNotFound(name="newer_college_123")),
    ({"name": "newer_college_2020/bad"}, ds.SequenceNotFound("newer_college_2020/bad")),
    ({"length": 100}, ds.InvalidDatasetConfig("Missing 'name' in dataset config")), # type: ignore
]
# fmt: on


# Test to ensure datasets are parsed correctly
@pytest.mark.parametrize("dataset_name, expected", DATASETS)
def test_dataset_parsing(
    dataset_name: str | DatasetConfig | Sequence[str | DatasetConfig],
    expected: DatasetConfigError | list[tuple[ds.Dataset, int]],
):
    dataset = ds.parse_config(dataset_name)
    assert dataset == expected


# ------------------------- Pipeline Parsing ------------------------- #
class FakePipeline(pl.Pipeline):
    @staticmethod
    def name() -> str:
        return "fake"

    @staticmethod
    def version() -> str:
        return "0.1.0"

    @staticmethod
    def default_params() -> dict[str, Param]:
        return {"param1": 1, "param2": "value"}


pl.register_pipeline(FakePipeline)

# fmt: off
PIPELINES: list[Any] = [
    # good ones
    ("fake", [(FakePipeline, {})]),
    ({"name": "fake"}, [(FakePipeline, {})]),
    ({"name": "fake", "param1": 5}, [(FakePipeline, {"param1": 5})]),
    (["fake", {"name": "fake", "param1": 3}], [(FakePipeline, {}), (FakePipeline, {"param1": 3})]),
    ({"name": "fake", "sweep": {"param1": [1, 2, 3]}}, [
        (FakePipeline, {"param1": 1}),
        (FakePipeline, {"param1": 2}),
        (FakePipeline, {"param1": 3}),
    ]),
    # bad ones
    ("unknown", pl.PipelineNotFound("unknown")),
    ({"name": "unknown"}, pl.PipelineNotFound("unknown")),
    ({"param1": 5}, pl.InvalidPipelineConfig("Need pipeline name: {'param1': 5}")), # type: ignore
    ({"name": "fake", "param3": 10}, pl.UnusedPipelineParam("param3", "fake")),
    ({"name": "fake", "param1": "wrong_type"}, pl.InvalidPipelineParamType("param1", int, str)),
    ({"name": "fake", "sweep": {"param1": [1.0, 2, 3]}}, pl.InvalidPipelineParamType("param1", int, float)),
    ({"name": "fake", "sweep": {"param3": [1.0, 2, 3]}}, pl.UnusedPipelineParam("param3", "fake")),
]
# fmt: on


# Test to ensure pipelines are parsed correctly
@pytest.mark.parametrize("pipeline_name, expected", PIPELINES)
def test_pipeline_parsing(
    pipeline_name: str | dict[str, Param] | Sequence[str | dict[str, Param]],
    expected: list[tuple[type[pl.Pipeline], dict[str, Param]]] | pl.PipelineConfigError,
):
    pipeline = pl.parse_config(pipeline_name)
    assert pipeline == expected
